/*
 * Unit tests for the motor drive: current sensing (core/motor/current.c) and the FOC step, gates and taps
 * (core/motor/motor.c). Expected values are worked out by hand from the rules in the sources; bit-exact
 * comparison with the stock drive is the job of the lock-step harness.
 */
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "euc/motor/motor.h"
#include "euc/quirks.h"
#include "euc/sil.h"
#include "unity.h"

#define MID_SCALE_RAW 2048U  /* 12-bit sample at zero current */
#define MID_SCALE_SUM 32768U /* 16 samples at mid-scale */

static const struct euc_motor_config board_config = {{31500, 2U}};
static const uint32_t profiles[2] = {EUC_SIL_PROFILE_COMPAT, EUC_SIL_PROFILE_CLEAN};

static void use_profile(uint32_t profile)
{
    euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profile));
}

void setUp(void)
{
    use_profile(EUC_SIL_PROFILE_COMPAT);
}

void tearDown(void)
{
}

static void rest_on_halls(struct euc_motor_state *state)
{
    struct euc_hall_out hall_out;
    for (unsigned i = 0U; i < 400U; i++) {
        euc_motor_hall_step(state, 5U, &hall_out);
    }
}

/* Power-on with mid-scale offsets, the wheel at rest in hall state 5 (long enough for the one-time enable).
 */
static void power_on(struct euc_motor_state *state, uint8_t drive_mode)
{
    euc_motor_init(state, drive_mode, &board_config);
    const struct euc_current_offsets_in offsets = {MID_SCALE_SUM, MID_SCALE_SUM, MID_SCALE_SUM, 1U, 0U, 0U};
    euc_current_set_offsets(&state->current, &offsets);
    rest_on_halls(state);
}

static struct euc_motor_foc_in idle_input(void)
{
    const struct euc_motor_foc_in in = {{MID_SCALE_RAW, MID_SCALE_RAW}, 0, false, false};
    return in;
}

/* Current sensing ------------------------------------------------------------------------------------- */

static void test_offsets_per_profile(void)
{
    struct euc_current_state state;
    const struct euc_current_offsets_in measured = {32000U, 32100U, 32900U, 3U, 30000U, 0xFFFFU};
    struct euc_current_offsets_in tenth_boot = measured;
    tenth_boot.boot_count = 10U;

    use_profile(EUC_SIL_PROFILE_COMPAT);
    euc_current_init(&state);
    euc_current_set_offsets(&state, &measured);
    TEST_ASSERT_EQUAL_INT32(32000, state.offset_a);
    TEST_ASSERT_EQUAL_INT32(32100, state.offset_b); /* phase B from ADC1 */
    euc_current_set_offsets(&state, &tenth_boot);
    TEST_ASSERT_EQUAL_INT32(30000, state.offset_a); /* stored offset replaces the measured one */
    TEST_ASSERT_EQUAL_INT32(32100, state.offset_b); /* 0xFFFF: not set */

    use_profile(EUC_SIL_PROFILE_CLEAN);
    euc_current_set_offsets(&state, &tenth_boot);
    TEST_ASSERT_EQUAL_INT32(32000, state.offset_a);
    TEST_ASSERT_EQUAL_INT32(32900, state.offset_b); /* phase B from ADC2, its run-time converter */
}

static void test_current_signs(void)
{
    struct euc_current_state state;
    struct euc_current_out out;
    euc_current_init(&state);
    const struct euc_current_offsets_in offsets = {MID_SCALE_SUM, MID_SCALE_SUM, MID_SCALE_SUM, 1U, 0U, 0U};
    euc_current_set_offsets(&state, &offsets);
    /* +100 raw on A, +50 on B: a = 1600, b = 800. Ia = -a (opposite sensor sign), Ib = b, c = a - b. */
    euc_current_read(&state, &board_config.current, (struct euc_current_sample){2148U, 2098U}, &out);
    TEST_ASSERT_EQUAL_INT16(-1600, out.ia_q15);
    TEST_ASSERT_EQUAL_INT16(800, out.ib_q15);
    TEST_ASSERT_EQUAL_INT16(1600, state.a);
    TEST_ASSERT_EQUAL_INT16(800, state.b);
    TEST_ASSERT_EQUAL_INT16(800, state.c);
    TEST_ASSERT_EQUAL_INT16(1600, state.peak);
}

static void test_phase_a_saturation_per_profile(void)
{
    const struct euc_current_offsets_in zero_offset = {0U, MID_SCALE_SUM, MID_SCALE_SUM, 1U, 0U, 0U};
    const struct euc_current_offsets_in full_offset = {65520U, MID_SCALE_SUM, MID_SCALE_SUM, 1U, 0U, 0U};
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        const bool compat = (profiles[p] == EUC_SIL_PROFILE_COMPAT);
        struct euc_current_state state;
        struct euc_current_out out;
        euc_current_init(&state);
        state.readings_since_boot = 1000U;
        euc_current_set_offsets(&state, &zero_offset);
        /* a = 65520: Ia = -a saturates to -32768; stock swaps the branches and gives +32767. */
        euc_current_read(&state, &board_config.current, (struct euc_current_sample){4095U, MID_SCALE_RAW},
                         &out);
        TEST_ASSERT_EQUAL_INT16(compat ? INT16_MAX : INT16_MIN, out.ia_q15);
        euc_current_set_offsets(&state, &full_offset);
        euc_current_read(&state, &board_config.current, (struct euc_current_sample){0U, MID_SCALE_RAW}, &out);
        TEST_ASSERT_EQUAL_INT16(compat ? INT16_MIN : INT16_MAX, out.ia_q15);
    }
}

static void test_raw_copies_wrap_or_saturate(void)
{
    const struct euc_current_offsets_in zero_offset = {0U, MID_SCALE_SUM, MID_SCALE_SUM, 1U, 0U, 0U};
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        struct euc_current_state state;
        struct euc_current_out out;
        euc_current_init(&state);
        state.readings_since_boot = 1000U;
        euc_current_set_offsets(&state, &zero_offset);
        euc_current_read(&state, &board_config.current, (struct euc_current_sample){2500U, MID_SCALE_RAW},
                         &out);
        /* a = 40000: wraps to -25536 in compat, which also hides it from the over-current count. */
        if (profiles[p] == EUC_SIL_PROFILE_COMPAT) {
            TEST_ASSERT_EQUAL_INT16(-25536, state.a);
            TEST_ASSERT_EQUAL_UINT16(0U, state.over_limit_readings);
        } else {
            TEST_ASSERT_EQUAL_INT16(INT16_MAX, state.a);
            TEST_ASSERT_EQUAL_UINT16(1U, state.over_limit_readings);
        }
    }
}

/* The trip is off in both profiles until the bench measures a level (NO_OVERCURRENT_TRIP on in clean too);
 * the code stays tested with the quirk bit cleared. */
static void test_over_current_counted_unless_the_trip_is_on(void)
{
    /* b = 16 x 4048 - 32768 = 32000 > 31500. */
    const struct euc_current_sample high = {MID_SCALE_RAW, 4048U};
    const uint64_t clean = euc_sil_quirk_profile_mask(EUC_SIL_PROFILE_CLEAN);
    const uint64_t masks[3] = {euc_sil_quirk_profile_mask(EUC_SIL_PROFILE_COMPAT), clean,
                               clean & ~(UINT64_C(1) << EUC_QUIRK_ID_NO_OVERCURRENT_TRIP)};
    for (size_t p = 0U; p < 3U; p++) {
        euc_sil_quirk_set_mask(masks[p]);
        struct euc_current_state state;
        struct euc_current_out out;
        euc_current_init(&state);
        state.readings_since_boot = 1000U;
        const struct euc_current_offsets_in offsets = {
            MID_SCALE_SUM, MID_SCALE_SUM, MID_SCALE_SUM, 1U, 0U, 0U};
        euc_current_set_offsets(&state, &offsets);
        euc_current_read(&state, &board_config.current, high, &out);
        TEST_ASSERT_EQUAL_UINT16(1U, state.over_limit_readings);
        TEST_ASSERT_FALSE(state.overcurrent_trip);
        euc_current_read(&state, &board_config.current, high, &out);
        TEST_ASSERT_EQUAL_UINT16(2U, state.over_limit_readings);
        TEST_ASSERT_EQUAL(p == 2U, state.overcurrent_trip);
        euc_current_read(&state, &board_config.current,
                         (struct euc_current_sample){MID_SCALE_RAW, MID_SCALE_RAW}, &out);
        TEST_ASSERT_EQUAL_UINT16(0U, state.over_limit_readings);
    }
}

static void test_sensor_check_at_start(void)
{
    struct euc_current_state state;
    struct euc_current_out out;
    euc_current_init(&state);
    const struct euc_current_offsets_in offsets = {MID_SCALE_SUM, MID_SCALE_SUM, MID_SCALE_SUM, 1U, 0U, 0U};
    euc_current_set_offsets(&state, &offsets);
    const struct euc_current_sample high = {MID_SCALE_RAW + 400U, MID_SCALE_RAW}; /* a = 6400 > 6000 */
    euc_current_read(&state, &board_config.current, high, &out); /* reading 0 is not checked */
    TEST_ASSERT_FALSE(out.start_check_failed);
    euc_current_read(&state, &board_config.current, high, &out);
    TEST_ASSERT_TRUE(out.start_check_failed);
    TEST_ASSERT_TRUE(state.sensor_fault);

    euc_current_init(&state);
    euc_current_set_offsets(&state, &offsets);
    for (unsigned i = 0U; i < 1000U; i++) {
        euc_current_read(&state, &board_config.current,
                         (struct euc_current_sample){MID_SCALE_RAW, MID_SCALE_RAW}, &out);
    }
    euc_current_read(&state, &board_config.current, high, &out); /* the check window is over */
    TEST_ASSERT_FALSE(out.start_check_failed);
    TEST_ASSERT_EQUAL_UINT16(1000U, state.readings_since_boot);
}

/* Timing, command, speed limit, PWM % ------------------------------------------------------------------ */

static void test_timing_per_mode(void)
{
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        const struct euc_motor_timing modes[2] = {euc_motor_timing_for_mode(0U),
                                                  euc_motor_timing_for_mode(1U)};
        TEST_ASSERT_EQUAL_UINT16(10000U, modes[0].pwm_frequency_hz);
        TEST_ASSERT_EQUAL_UINT16(3600U, modes[0].pwm_period_counts);
        TEST_ASSERT_EQUAL_UINT16(12000U, modes[1].pwm_frequency_hz);
        TEST_ASSERT_EQUAL_UINT16(3000U, modes[1].pwm_period_counts);
        TEST_ASSERT_EQUAL_UINT8(29U, modes[1].dead_time_counts);
        TEST_ASSERT_EQUAL_UINT8((profiles[p] == EUC_SIL_PROFILE_COMPAT) ? 1U : 29U,
                                modes[0].dead_time_counts);
        for (size_t m = 0U; m < 2U; m++) {
            const uint32_t arr = modes[m].pwm_period_counts;
            const uint64_t t = modes[m].svpwm_t;
            const uint64_t t_sqrt3 = modes[m].svpwm_t_sqrt3;
            TEST_ASSERT_EQUAL_UINT32(72000000U / (2U * arr), modes[m].pwm_frequency_hz);
            TEST_ASSERT_EQUAL_UINT64(4U * (uint64_t)arr, t);
            TEST_ASSERT_TRUE((t_sqrt3 * t_sqrt3) <= (3U * t * t)); /* floor(T sqrt(3)) */
            TEST_ASSERT_TRUE(((t_sqrt3 + 1U) * (t_sqrt3 + 1U)) > (3U * t * t));
        }
        TEST_ASSERT_EQUAL_UINT16(12000U, euc_motor_timing_for_mode(7U).pwm_frequency_hz); /* any non-zero */
    }
}

static void test_set_command_is_checked(void)
{
    struct euc_motor_state state;
    power_on(&state, EUC_MOTOR_CURRENT_MODE);
    euc_motor_set_command(&state, 12.5F);
    TEST_ASSERT_EQUAL_FLOAT(12.5F, state.command);
    euc_motor_set_command(&state, NAN);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, state.command);
    euc_motor_set_command(&state, -INFINITY);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, state.command);
    euc_motor_set_command(&state, 5000.0F);
    TEST_ASSERT_EQUAL_FLOAT(1050.0F, state.command);
    euc_motor_set_command(&state, -5000.0F);
    TEST_ASSERT_EQUAL_FLOAT(-1050.0F, state.command);
}

static void test_q_integral_limit_follows_speed(void)
{
    struct euc_motor_state state;
    power_on(&state, EUC_MOTOR_CURRENT_MODE);
    TEST_ASSERT_EQUAL_INT32(0x0FA00000, state.pid_q.integral_max); /* power-on */
    euc_motor_rtc_step(&state, -50);
    TEST_ASSERT_EQUAL_INT32(50 * 0x140000, state.pid_q.integral_max);
    TEST_ASSERT_EQUAL_INT32(-50 * 0x140000, state.pid_q.integral_min);
    euc_motor_rtc_step(&state, 300);
    TEST_ASSERT_EQUAL_INT32(200 * 0x140000, state.pid_q.integral_max);
    euc_motor_rtc_step(&state, 0);
    TEST_ASSERT_EQUAL_INT32(0, state.pid_q.integral_max); /* standstill: PD only */
}

static void test_pwm_percent(void)
{
    struct euc_motor_state state;
    power_on(&state, EUC_MOTOR_CURRENT_MODE);
    const int16_t vq[5] = {31500, 629, 314, -630, -31500};
    const int8_t percent[5] = {-100, -1, 0, 2, 100};
    for (size_t i = 0U; i < 5U; i++) {
        state.vq_active = vq[i];
        TEST_ASSERT_EQUAL_INT8(percent[i], euc_motor_pwm_percent(&state));
    }
}

/* FOC step --------------------------------------------------------------------------------------------- */

static void test_bridge_turns_on_once_after_standstill(void)
{
    struct euc_motor_state state;
    struct euc_motor_foc_out out;
    const struct euc_motor_foc_in in = idle_input();
    euc_motor_init(&state, EUC_MOTOR_CURRENT_MODE, &board_config);
    const struct euc_current_offsets_in offsets = {MID_SCALE_SUM, MID_SCALE_SUM, MID_SCALE_SUM, 1U, 0U, 0U};
    euc_current_set_offsets(&state.current, &offsets);
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_FALSE(out.outputs_enabled);
    TEST_ASSERT_FALSE(out.start_balance);

    rest_on_halls(&state);
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_TRUE(out.outputs_enabled);
    TEST_ASSERT_TRUE(out.start_balance);
    /* Zero current, zero command: every phase at 50 %. */
    TEST_ASSERT_EQUAL_UINT16(1500U, out.duty.a);
    TEST_ASSERT_EQUAL_UINT16(1500U, out.duty.b);
    TEST_ASSERT_EQUAL_UINT16(1500U, out.duty.c);
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_TRUE(out.outputs_enabled);
    TEST_ASSERT_FALSE(out.start_balance);
}

static void test_current_mode_regulation(void)
{
    struct euc_motor_state state;
    struct euc_motor_foc_out out;
    struct euc_motor_foc_in in = idle_input();
    power_on(&state, EUC_MOTOR_CURRENT_MODE);
    euc_motor_set_command(&state, 100.0F); /* Iq* = 2000 */
    euc_motor_foc_step(&state, &in, &out);
    /* e = 2000: P 2000 x 3000 / 1024 = 5859, I 30 x 2000 / 8192 = 7, D 5000 x 2000 / 8192 = 1220. */
    TEST_ASSERT_EQUAL_INT16(7086, state.v_qd.q_q15);
    TEST_ASSERT_EQUAL_INT16(0, state.v_qd.d_q15);
    TEST_ASSERT_EQUAL_INT16(7086, state.vq_active);
    TEST_ASSERT_EQUAL_INT32(60000, state.pid_q.integral);
    TEST_ASSERT_EQUAL_INT16(3, state.id_gain);
    TEST_ASSERT_TRUE(out.duty.a != 1500U);

    in.speed_dm_per_s = -7;
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_EQUAL_INT16(10, state.id_gain); /* 2 x |v|, capped at 10 */
}

static void test_voltage_mode_slew_and_write_back(void)
{
    struct euc_motor_state state;
    struct euc_motor_foc_out out;
    const struct euc_motor_foc_in in = idle_input();
    power_on(&state, EUC_MOTOR_VOLTAGE_MODE);
    euc_motor_set_command(&state, 100.0F);
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_EQUAL_FLOAT(60.0F, state.command);  /* slew 60 per step, written back */
    TEST_ASSERT_EQUAL_INT16(1800, state.vq_active); /* 30 x 60 */
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_EQUAL_FLOAT(60.0F, state.command); /* no new command from the balance */
    euc_motor_set_command(&state, 100.0F);
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_EQUAL_FLOAT(100.0F, state.command);
    TEST_ASSERT_EQUAL_INT16(3000, state.vq_active);
    euc_motor_set_command(&state, -1050.0F);
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_EQUAL_FLOAT(40.0F, state.command);
}

static void test_voltage_mode_backs_off_above_280_a(void)
{
    struct euc_motor_state state;
    struct euc_motor_foc_out out;
    power_on(&state, EUC_MOTOR_VOLTAGE_MODE);
    state.current.readings_since_boot = 1000U; /* past the start check */
    state.command_previous = 500.0F;
    euc_motor_set_command(&state, 500.0F);
    /* Hall state 5 (150 deg). Ia = -28864, Ib = 14432 (beta about 0): Iq = Ia cos 150 = about 25000 counts.
     */
    struct euc_motor_foc_in in = idle_input();
    in.sample = (struct euc_current_sample){3852U, 2950U};
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_TRUE(state.iq_01a > 2800);
    /* 500 - (|Iq| - 21000) is far below 500 - 60: the slew holds it at 440. */
    TEST_ASSERT_TRUE((500 - ((int32_t)state.i_qd.q_q15 - 21000)) < 440);
    TEST_ASSERT_EQUAL_FLOAT(440.0F, state.command);
    TEST_ASSERT_EQUAL_INT16(13200, state.vq_active);
}

static void test_stop_brake(void)
{
    struct euc_motor_state state;
    struct euc_motor_foc_out out;
    struct euc_motor_foc_in in = idle_input();
    power_on(&state, EUC_MOTOR_CURRENT_MODE);
    euc_motor_set_command(&state, 100.0F);
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_TRUE(state.pid_q.integral != 0);

    /* Disabled while moving fast: still regulating. */
    in.drive_disabled = true;
    in.speed_dm_per_s = 10;
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_TRUE(state.vq_active != 0);
    TEST_ASSERT_FALSE(state.brake_latched);

    /* Disabled and slow: brake. Low sides on, the bridge stays enabled. */
    in.speed_dm_per_s = -4;
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_TRUE(state.brake_latched);
    TEST_ASSERT_EQUAL_INT16(0, state.vq_active);
    TEST_ASSERT_EQUAL_UINT16(0U, out.duty.a);
    TEST_ASSERT_EQUAL_UINT16(0U, out.duty.b);
    TEST_ASSERT_EQUAL_UINT16(0U, out.duty.c);
    TEST_ASSERT_TRUE(out.outputs_enabled);

    /* The next step clears the integrals; the latch holds even when the wheel speeds up. */
    in.speed_dm_per_s = 10;
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_EQUAL_INT32(0, state.pid_q.integral);
    TEST_ASSERT_EQUAL_INT32(0, state.pid_d.integral);
    TEST_ASSERT_EQUAL_INT16(0, state.vq_active);
    TEST_ASSERT_EQUAL_UINT16(0U, out.duty.a);

    /* Enabled again: regulating, latch released. */
    in.drive_disabled = false;
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_FALSE(state.brake_latched);
    TEST_ASSERT_TRUE(state.vq_active != 0);
}

static void test_hard_cut_is_latched(void)
{
    struct euc_motor_state state;
    struct euc_motor_foc_out out;
    struct euc_motor_foc_in in = idle_input();
    power_on(&state, EUC_MOTOR_CURRENT_MODE);
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_TRUE(out.outputs_enabled);
    in.hard_cut_request = true;
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_FALSE(out.outputs_enabled);
    in.hard_cut_request = false;
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_FALSE(out.outputs_enabled);
    TEST_ASSERT_EQUAL_UINT16(0U, out.duty.a);
    TEST_ASSERT_EQUAL_UINT16(0U, out.duty.b);
    TEST_ASSERT_EQUAL_UINT16(0U, out.duty.c);
}

static void test_hall_fault_turns_the_bridge_off(void)
{
    struct euc_motor_state state;
    struct euc_motor_foc_out out;
    struct euc_hall_out hall_out;
    const struct euc_motor_foc_in in = idle_input();
    power_on(&state, EUC_MOTOR_CURRENT_MODE);
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_TRUE(out.outputs_enabled);
    bool raised = false;
    for (unsigned i = 0U; i < 1600U; i++) {
        euc_motor_hall_step(&state, 7U, &hall_out); /* invalid state */
        raised = raised || hall_out.fault_raised;
    }
    TEST_ASSERT_TRUE(raised);
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_FALSE(out.outputs_enabled);
}

static void test_start_check_stops_the_drive(void)
{
    struct euc_motor_state state;
    struct euc_motor_foc_out out;
    struct euc_motor_foc_in in = idle_input();
    power_on(&state, EUC_MOTOR_CURRENT_MODE);
    euc_motor_foc_step(&state, &in, &out);        /* reading 0 */
    in.sample.phase_b_raw = MID_SCALE_RAW + 400U; /* b = 6400 */
    euc_motor_foc_step(&state, &in, &out);
    TEST_ASSERT_TRUE(out.stop_drive);
    TEST_ASSERT_EQUAL_HEX8(EUC_MOTOR_ERROR_CURRENT_SENSOR, out.error_bits);
    TEST_ASSERT_FALSE(out.outputs_enabled);
    TEST_ASSERT_TRUE(state.hard_cut);
}

static void test_over_current_trip_cuts_when_switched_on(void)
{
    const uint64_t clean = euc_sil_quirk_profile_mask(EUC_SIL_PROFILE_CLEAN);
    const uint64_t masks[3] = {euc_sil_quirk_profile_mask(EUC_SIL_PROFILE_COMPAT), clean,
                               clean & ~(UINT64_C(1) << EUC_QUIRK_ID_NO_OVERCURRENT_TRIP)};
    for (size_t p = 0U; p < 3U; p++) {
        euc_sil_quirk_set_mask(masks[p]);
        struct euc_motor_state state;
        struct euc_motor_foc_out out;
        struct euc_motor_foc_in in = idle_input();
        power_on(&state, EUC_MOTOR_CURRENT_MODE);
        state.current.readings_since_boot = 1000U;
        in.sample.phase_b_raw = 4048U; /* b = 32000 */
        euc_motor_foc_step(&state, &in, &out);
        euc_motor_foc_step(&state, &in, &out);
        TEST_ASSERT_EQUAL(p != 2U, out.outputs_enabled); /* only with the trip switched on */
    }
}

/* Clean: a balance command that is not renewed for 3 ticks (72 FOC steps at 12 kHz) ramps to 0 at 4200 per
 * second (0.35 per step) and latches (STALE_COMMAND_HELD). Compat keeps the last command. */
static void test_stale_command_ramps_to_zero_in_clean(void)
{
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        struct euc_motor_state state;
        struct euc_motor_foc_out out;
        const struct euc_motor_foc_in in = idle_input();
        power_on(&state, EUC_MOTOR_CURRENT_MODE);
        TEST_ASSERT_EQUAL_UINT16(72U, state.stale_limit_steps);
        for (uint32_t i = 0U; i < 500U; i++) {
            euc_motor_foc_step(&state, &in, &out); /* no command yet (boot): never stale */
        }
        TEST_ASSERT_FALSE(state.command_stale);
        euc_motor_set_command(&state, 500.0F);
        for (uint32_t i = 0U; i < 72U; i++) {
            euc_motor_foc_step(&state, &in, &out);
        }
        TEST_ASSERT_FALSE(state.command_stale);
        TEST_ASSERT_EQUAL_FLOAT(500.0F, state.command);
        euc_motor_foc_step(&state, &in, &out); /* the 73rd step: 3 ticks and more */
        if (profiles[p] == EUC_SIL_PROFILE_COMPAT) {
            TEST_ASSERT_FALSE(state.command_stale);
            TEST_ASSERT_EQUAL_FLOAT(500.0F, state.command);
            continue;
        }
        TEST_ASSERT_TRUE(state.command_stale);
        TEST_ASSERT_FLOAT_WITHIN(0.001F, 500.0F - 0.35F, state.command);
        for (uint32_t i = 0U; i < 1500U; i++) { /* 500 / 0.35 = 1429 steps, 0.12 s */
            euc_motor_foc_step(&state, &in, &out);
        }
        TEST_ASSERT_EQUAL_FLOAT(0.0F, state.command);
        euc_motor_set_command(&state, 800.0F); /* latched: a late command does not drive again */
        euc_motor_foc_step(&state, &in, &out);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, state.command);
    }
}

static void test_fresh_commands_keep_the_command(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_motor_state state;
    struct euc_motor_foc_out out;
    const struct euc_motor_foc_in in = idle_input();
    power_on(&state, EUC_MOTOR_VOLTAGE_MODE);
    TEST_ASSERT_EQUAL_UINT16(60U, state.stale_limit_steps); /* 10 kHz */
    for (uint32_t tick = 0U; tick < 100U; tick++) {
        euc_motor_set_command(&state, -300.0F);
        for (uint32_t i = 0U; i < 20U; i++) { /* about one tick of FOC steps */
            euc_motor_foc_step(&state, &in, &out);
        }
    }
    TEST_ASSERT_FALSE(state.command_stale);
}

static void test_iq_lpf_wraps_above_65_a_in_compat(void)
{
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        struct euc_motor_state state;
        struct euc_motor_foc_out out;
        struct euc_motor_foc_in in = idle_input();
        power_on(&state, EUC_MOTOR_CURRENT_MODE);
        state.current.readings_since_boot = 1000U;
        /* Ia = -5792, Ib = 2896 at 150 deg: Iq about 5016 counts (67 A). */
        in.sample = (struct euc_current_sample){2410U, 2229U};
        euc_motor_foc_step(&state, &in, &out);
        const int64_t product = (int64_t)state.i_qd.q_q15 * 437500;
        TEST_ASSERT_TRUE(product > INT32_MAX);
        const int64_t wrapped = product - INT64_C(4294967296);
        const int64_t milliamps = ((profiles[p] == EUC_SIL_PROFILE_COMPAT) ? wrapped : product) / 32768;
        TEST_ASSERT_EQUAL_INT32((int32_t)(milliamps / 10000), state.iq_lpf_ma);
        TEST_ASSERT_EQUAL_INT32(((int32_t)state.i_qd.q_q15 * 4375) / 32768, state.iq_01a);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_offsets_per_profile);
    RUN_TEST(test_current_signs);
    RUN_TEST(test_phase_a_saturation_per_profile);
    RUN_TEST(test_raw_copies_wrap_or_saturate);
    RUN_TEST(test_over_current_counted_unless_the_trip_is_on);
    RUN_TEST(test_sensor_check_at_start);
    RUN_TEST(test_timing_per_mode);
    RUN_TEST(test_set_command_is_checked);
    RUN_TEST(test_q_integral_limit_follows_speed);
    RUN_TEST(test_pwm_percent);
    RUN_TEST(test_bridge_turns_on_once_after_standstill);
    RUN_TEST(test_current_mode_regulation);
    RUN_TEST(test_voltage_mode_slew_and_write_back);
    RUN_TEST(test_voltage_mode_backs_off_above_280_a);
    RUN_TEST(test_stop_brake);
    RUN_TEST(test_hard_cut_is_latched);
    RUN_TEST(test_hall_fault_turns_the_bridge_off);
    RUN_TEST(test_start_check_stops_the_drive);
    RUN_TEST(test_over_current_trip_cuts_when_switched_on);
    RUN_TEST(test_stale_command_ramps_to_zero_in_clean);
    RUN_TEST(test_fresh_commands_keep_the_command);
    RUN_TEST(test_iq_lpf_wraps_above_65_a_in_compat);
    return UNITY_END();
}
