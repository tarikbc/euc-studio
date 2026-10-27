/*
 * Unit tests for core/protocol/telemetry.c.
 *
 * The oracle is the real capture tests/data/begode-extreme/idle-2026-10-15.hex (BLE notifications from the
 * maintainer's wheel). Every frame is decoded into struct euc_tlm_in by a small helper here, encoded again by
 * the firmware encoder, and must come back byte for byte. The group order is checked against the capture too.
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "euc/protocol/telemetry.h"
#include "unity.h"

#define CAPTURE_PATH       EUC_TEST_DATA_DIR "/begode-extreme/idle-2026-10-15.hex"
#define CAPTURE_MAX_BYTES  16384U
#define CAPTURE_FRAMES     380U
#define CAPTURE_GROUPS     (CAPTURE_FRAMES / EUC_TLM_GROUP_FRAMES)
#define CAPTURE_LINE_BYTES 256U

static uint8_t capture_stream[CAPTURE_MAX_BYTES];
static size_t capture_stream_bytes;
static uint8_t capture_frames[CAPTURE_FRAMES][EUC_TLM_FRAME_BYTES];
static size_t capture_frame_count;
static const char *capture_error; /* set by load_capture_stream(); reported by the first test */

/* Capture file: one notification per line, "<seconds> <hex byte> <hex byte> ...". Empty lines and lines that
 * start with '#' (the recorder's header) are skipped. */
static void load_capture_stream(void)
{
    FILE *file = fopen(CAPTURE_PATH, "r");
    if (file == NULL) {
        capture_error = "cannot open " CAPTURE_PATH;
        return;
    }
    char line[CAPTURE_LINE_BYTES];
    capture_stream_bytes = 0U;
    while ((capture_error == NULL) && (fgets(line, (int)sizeof(line), file) != NULL)) {
        const char *start = line;
        while ((*start == ' ') || (*start == '\t')) {
            start++;
        }
        if ((*start == '\0') || (*start == '\n') || (*start == '\r') || (*start == '#')) {
            continue;
        }
        const char *cursor = strchr(start, ' '); /* skip the timestamp */
        while (cursor != NULL) {
            char *end = NULL;
            const unsigned long value = strtoul(cursor, &end, 16);
            if (end == cursor) {
                break;
            }
            if ((value > 0xFFUL) || (capture_stream_bytes >= CAPTURE_MAX_BYTES)) {
                capture_error = "capture line holds a value that is not a byte, or too many bytes";
                break;
            }
            capture_stream[capture_stream_bytes] = (uint8_t)value;
            capture_stream_bytes++;
            cursor = end;
        }
    }
    (void)fclose(file);
}

/* Frames are 55 AA ... 5A 5A 5A 5A; a byte that does not start one is skipped (none in this capture). */
static void reassemble_frames(void)
{
    static const uint8_t footer[4] = {0x5AU, 0x5AU, 0x5AU, 0x5AU};
    size_t offset = 0U;
    capture_frame_count = 0U;
    while ((offset + EUC_TLM_FRAME_BYTES) <= capture_stream_bytes) {
        const uint8_t *candidate = &capture_stream[offset];
        if ((candidate[0] == 0x55U) && (candidate[1] == 0xAAU) && (memcmp(&candidate[20], footer, 4U) == 0) &&
            (capture_frame_count < CAPTURE_FRAMES)) {
            memcpy(capture_frames[capture_frame_count], candidate, EUC_TLM_FRAME_BYTES);
            capture_frame_count++;
            offset += EUC_TLM_FRAME_BYTES;
        } else {
            offset++;
        }
    }
}

static uint16_t field_u16(const uint8_t *frame, size_t field)
{
    return (uint16_t)(((unsigned)frame[2U + (2U * field)] << 8U) | frame[3U + (2U * field)]);
}

static int16_t field_s16(const uint8_t *frame, size_t field)
{
    return (int16_t)field_u16(frame, field);
}

/* Test-side decoder: the inverse of the encoder for one frame, written from the protocol description. */
static void decode_frame(const uint8_t *frame, struct euc_tlm_in *in)
{
    const uint8_t type = frame[18];
    const uint8_t page = frame[19];
    if (type == EUC_TLM_FRAME_LIVE) {
        const uint16_t echo = field_u16(frame, 6U);
        in->battery_voltage_dv = (uint16_t)(field_u16(frame, 0U) / 5U);
        in->speed_cm_per_s = field_s16(frame, 1U);
        in->range_estimate_km = field_u16(frame, 2U);
        in->trip_distance_m = field_u16(frame, 3U);
        in->phase_current_ca = field_s16(frame, 4U);
        in->board_temperature_mpu6050 = field_s16(frame, 5U);
        in->turn_compensation = (uint8_t)(((uint32_t)echo >> 10U) & 0x0FU);
        in->pedal_trim = (uint8_t)(((uint32_t)echo >> 5U) & 0x0FU);
        in->pwm_tiltback_enabled = (((uint32_t)echo >> 3U) & 1U) != 0U;
        in->drive_mode = (uint8_t)(echo & 1U);
        in->beeper_volume = (uint8_t)field_u16(frame, 7U);
    } else if (type == EUC_TLM_FRAME_BMS) {
        in->pwm_alarm_percent = (uint8_t)field_u16(frame, 0U);
        in->frame01_speed_copy_dm_per_s = field_s16(frame, 1U);
        in->battery_voltage_dv = field_u16(frame, 2U);
        in->bms_blocks[page].current_da = field_s16(frame, 3U);
        in->bms_temperature_max_c = field_s16(frame, 4U);
        in->bms_temperature_min_c = field_s16(frame, 5U);
        in->bms_blocks[page].word42_raw = field_u16(frame, 6U);
        in->bms_blocks[page].status_word = field_u16(frame, 7U);
    } else if ((type == EUC_TLM_FRAME_CELLS_PACK_A) || (type == EUC_TLM_FRAME_CELLS_PACK_B)) {
        const size_t first_cell = ((type == EUC_TLM_FRAME_CELLS_PACK_B) ? 32U : 0U) + (8U * page);
        for (size_t cell = 0U; cell < 8U; cell++) {
            in->cell_voltage_mv[first_cell + cell] = field_u16(frame, cell);
        }
    } else if (type == EUC_TLM_FRAME_STATUS) {
        const uint16_t modes = field_u16(frame, 2U);
        in->odometer_m = ((uint32_t)field_u16(frame, 0U) << 16U) | field_u16(frame, 1U);
        in->pedal_mode = (uint8_t)(((uint32_t)modes >> 13U) & 3U);
        in->speed_alarm_mode = (uint8_t)(((uint32_t)modes >> 10U) & 3U);
        in->roll_cutoff_mode = (uint8_t)(((uint32_t)modes >> 7U) & 3U);
        in->units_miles = (modes & 1U) != 0U;
        in->auto_power_off_remaining_s = field_u16(frame, 3U);
        in->tiltback_speed_kmh = field_u16(frame, 4U);
        in->wm_value = (uint8_t)field_u16(frame, 5U);
        in->alert_flags = frame[14];
        in->light_state = frame[15];
    } else {
        const uint16_t echo = field_u16(frame, 1U);
        in->battery_current_ca = field_s16(frame, 0U);
        in->wc_value = (uint8_t)(((uint32_t)echo >> 4U) & 0x0FU);
        in->wx_value = (uint8_t)(echo & 0x0FU);
        in->pwm_percent = (int8_t)field_s16(frame, 3U);
    }
}

void setUp(void)
{
}

void tearDown(void)
{
}

static void test_capture_reassembles_into_380_frames(void)
{
    if (capture_error != NULL) {
        TEST_FAIL_MESSAGE(capture_error);
    }
    TEST_ASSERT_EQUAL_size_t(CAPTURE_FRAMES * EUC_TLM_FRAME_BYTES, capture_stream_bytes);
    TEST_ASSERT_EQUAL_size_t(CAPTURE_FRAMES, capture_frame_count);
}

static void test_every_capture_frame_encodes_back_to_the_same_bytes(void)
{
    for (size_t index = 0U; index < capture_frame_count; index++) {
        const uint8_t *captured = capture_frames[index];
        struct euc_tlm_in in;
        uint8_t encoded[EUC_TLM_FRAME_BYTES];
        char message[48];
        memset(&in, 0, sizeof(in));
        decode_frame(captured, &in);
        euc_tlm_encode_frame(&in, (enum euc_tlm_frame_type)captured[18], captured[19], encoded);
        (void)snprintf(message, sizeof(message), "frame %zu (type %02x sub %02x)", index, captured[18],
                       captured[19]);
        TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(captured, encoded, EUC_TLM_FRAME_BYTES, message);
    }
}

static void test_group_order_matches_the_capture(void)
{
    /* The capture starts at page 0 of pack B (frame 03). */
    struct euc_tlm_state state;
    struct euc_tlm_in in;
    struct euc_tlm_group group;
    memset(&in, 0, sizeof(in));
    euc_tlm_init(&state, 0U, true);
    for (size_t group_index = 0U; group_index < CAPTURE_GROUPS; group_index++) {
        euc_tlm_build_group(&state, &in, &group);
        for (size_t slot = 0U; slot < EUC_TLM_GROUP_FRAMES; slot++) {
            const uint8_t *captured = capture_frames[(group_index * EUC_TLM_GROUP_FRAMES) + slot];
            TEST_ASSERT_EQUAL_HEX8(captured[18], group.frames[slot][18]);
            TEST_ASSERT_EQUAL_HEX8(captured[19], group.frames[slot][19]);
        }
    }
}

static void test_capture_groups_encode_back_from_one_input_per_group(void)
{
    /* Same as the per-frame check, but through euc_tlm_build_group with one input for the whole group. */
    struct euc_tlm_state state;
    struct euc_tlm_group group;
    euc_tlm_init(&state, 0U, true);
    for (size_t group_index = 0U; group_index < CAPTURE_GROUPS; group_index++) {
        struct euc_tlm_in in;
        memset(&in, 0, sizeof(in));
        for (size_t slot = 0U; slot < EUC_TLM_GROUP_FRAMES; slot++) {
            decode_frame(capture_frames[(group_index * EUC_TLM_GROUP_FRAMES) + slot], &in);
        }
        euc_tlm_build_group(&state, &in, &group);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(capture_frames[group_index * EUC_TLM_GROUP_FRAMES], group.frames,
                                     EUC_TLM_GROUP_FRAMES * EUC_TLM_FRAME_BYTES);
    }
}

static void test_a_group_is_due_every_151_ticks(void)
{
    struct euc_tlm_state state;
    euc_tlm_init(&state, 0U, false);
    for (unsigned group = 0U; group < 3U; group++) {
        for (unsigned tick = 1U; tick < EUC_TLM_GROUP_PERIOD_TICKS; tick++) {
            euc_tlm_count_tick(&state);
            TEST_ASSERT_FALSE(euc_tlm_group_due(&state));
        }
        euc_tlm_count_tick(&state);
        TEST_ASSERT_TRUE(euc_tlm_group_due(&state));
        TEST_ASSERT_FALSE(euc_tlm_group_due(&state));
    }
}

static void test_live_frame_clamps_voltage_and_bms_frame_does_not(void)
{
    struct euc_tlm_in in;
    uint8_t frame[EUC_TLM_FRAME_BYTES];
    memset(&in, 0, sizeof(in));
    in.battery_voltage_dv = 1400U; /* 140.0 V */
    euc_tlm_encode_frame(&in, EUC_TLM_FRAME_LIVE, 0U, frame);
    TEST_ASSERT_EQUAL_UINT16(1344U * 5U, field_u16(frame, 0U));
    euc_tlm_encode_frame(&in, EUC_TLM_FRAME_BMS, 2U, frame);
    TEST_ASSERT_EQUAL_UINT16(1400U, field_u16(frame, 2U));
    TEST_ASSERT_EQUAL_HEX8(0x02U, frame[19]);
}

static void test_pwm_percent_is_sign_extended(void)
{
    struct euc_tlm_in in;
    uint8_t frame[EUC_TLM_FRAME_BYTES];
    memset(&in, 0, sizeof(in));
    in.pwm_percent = -1;
    euc_tlm_encode_frame(&in, EUC_TLM_FRAME_POWER, 0U, frame);
    TEST_ASSERT_EQUAL_HEX16(0xFFFFU, field_u16(frame, 3U));
    in.pwm_percent = 87;
    euc_tlm_encode_frame(&in, EUC_TLM_FRAME_POWER, 0U, frame);
    TEST_ASSERT_EQUAL_HEX16(87U, field_u16(frame, 3U));
}

static void test_status_frame_packs_modes_and_alerts(void)
{
    struct euc_tlm_in in;
    uint8_t frame[EUC_TLM_FRAME_BYTES];
    memset(&in, 0, sizeof(in));
    in.odometer_m = 1468670U;
    in.pedal_mode = 2U;
    in.speed_alarm_mode = 3U;
    in.roll_cutoff_mode = 1U;
    in.units_miles = true;
    in.alert_flags = EUC_TLM_ALERT_OVER_TEMPERATURE | EUC_TLM_ALERT_PWM_HIGH;
    in.light_state = 2U;
    euc_tlm_encode_frame(&in, EUC_TLM_FRAME_STATUS, 0U, frame);
    TEST_ASSERT_EQUAL_HEX16(0x0016U, field_u16(frame, 0U));
    TEST_ASSERT_EQUAL_HEX16(0x68FEU, field_u16(frame, 1U));
    TEST_ASSERT_EQUAL_HEX16((2U << 13U) | (3U << 10U) | (1U << 7U) | 1U, field_u16(frame, 2U));
    TEST_ASSERT_EQUAL_HEX8(0x21U, frame[14]);
    TEST_ASSERT_EQUAL_HEX8(0x02U, frame[15]);
    TEST_ASSERT_EQUAL_HEX16(0x0012U, field_u16(frame, 7U));
    TEST_ASSERT_EQUAL_HEX8(EUC_TLM_FIXED_SUB_INDEX, frame[19]);
}

static void test_cell_pages_map_to_cell_numbers(void)
{
    struct euc_tlm_in in;
    uint8_t frame[EUC_TLM_FRAME_BYTES];
    memset(&in, 0, sizeof(in));
    for (size_t cell = 0U; cell < EUC_TLM_CELLS; cell++) {
        in.cell_voltage_mv[cell] = (uint16_t)(4000U + cell);
    }
    euc_tlm_encode_frame(&in, EUC_TLM_FRAME_CELLS_PACK_B, 3U, frame);
    TEST_ASSERT_EQUAL_UINT16(4000U + 56U, field_u16(frame, 0U));
    TEST_ASSERT_EQUAL_UINT16(4000U + 63U, field_u16(frame, 7U));
    euc_tlm_encode_frame(&in, EUC_TLM_FRAME_CELLS_PACK_A, 1U, frame);
    TEST_ASSERT_EQUAL_UINT16(4000U + 8U, field_u16(frame, 0U));
    TEST_ASSERT_EQUAL_HEX8(0x01U, frame[19]);
}

/* IMU frame (option, docs/protocol/begode.md "IMU frame") -------------------------------------------------
 */

static struct euc_tlm_imu imu_at_rest(void)
{
    struct euc_tlm_imu imu;
    memset(&imu, 0, sizeof(imu));
    imu.accel[2] = 8192; /* 1 g up */
    imu.gyro_config = EUC_TLM_GYRO_500DPS;
    return imu;
}

static bool payload_holds_a_frame_start(const uint8_t *frame)
{
    for (size_t at = 2U; (at + 1U) < 18U; at++) {
        if ((frame[at] == 0x55U) && (frame[at + 1U] == 0xAAU)) {
            return true;
        }
    }
    return false;
}

static void test_imu_frame_layout_and_units(void)
{
    struct euc_tlm_imu imu = imu_at_rest();
    imu.pitch_deg = 1.234F;
    imu.roll_deg = -0.5F;
    imu.gyro[0] = 655;    /* 10.0 deg/s at 65.5 counts per deg/s */
    imu.gyro[1] = -131;   /* -2.0 deg/s */
    imu.gyro[2] = 66;     /* 1.008 deg/s */
    imu.accel[0] = -4096; /* -0.5 g */
    imu.accel[1] = 1;     /* 0.12 mg */
    imu.sequence = 0xABU;
    uint8_t frame[EUC_TLM_FRAME_BYTES];
    euc_tlm_encode_imu(&imu, frame);

    TEST_ASSERT_EQUAL_HEX8(0x55U, frame[0]);
    TEST_ASSERT_EQUAL_HEX8(0xAAU, frame[1]);
    const int16_t expected[8] = {123, -50, 100, -20, 10, -500, 0, 1000};
    for (size_t field = 0U; field < 8U; field++) {
        TEST_ASSERT_EQUAL_INT16(expected[field], field_s16(frame, field));
    }
    TEST_ASSERT_EQUAL_HEX8(EUC_TLM_FRAME_IMU, frame[18]);
    TEST_ASSERT_EQUAL_HEX8(0xABU, frame[19]);
    for (size_t at = 20U; at < 24U; at++) {
        TEST_ASSERT_EQUAL_HEX8(0x5AU, frame[at]);
    }
}

static void test_imu_gyro_scale_follows_the_range(void)
{
    struct euc_tlm_imu imu = imu_at_rest();
    imu.gyro_config = 0x18U; /* +-2000 deg/s, 16.4 counts per deg/s (the clean profile) */
    imu.gyro[0] = 164;
    imu.gyro[1] = INT16_MIN;
    uint8_t frame[EUC_TLM_FRAME_BYTES];
    euc_tlm_encode_imu(&imu, frame);
    TEST_ASSERT_EQUAL_INT16(100, field_s16(frame, 2U));
    TEST_ASSERT_EQUAL_INT16(-19980, field_s16(frame, 3U)); /* -1998.0 deg/s: the full range fits */

    imu.gyro_config = EUC_TLM_GYRO_500DPS;
    imu.gyro[1] = INT16_MAX;
    euc_tlm_encode_imu(&imu, frame);
    TEST_ASSERT_EQUAL_INT16(5003, field_s16(frame, 3U)); /* 500.3 deg/s */
}

static void test_imu_values_round_and_clamp(void)
{
    struct euc_tlm_imu imu = imu_at_rest();
    imu.pitch_deg = 0.006F;
    imu.roll_deg = -0.006F;
    uint8_t frame[EUC_TLM_FRAME_BYTES];
    euc_tlm_encode_imu(&imu, frame);
    TEST_ASSERT_EQUAL_INT16(1, field_s16(frame, 0U));
    TEST_ASSERT_EQUAL_INT16(-1, field_s16(frame, 1U));

    imu.pitch_deg = 400.0F;
    imu.roll_deg = -400.0F;
    euc_tlm_encode_imu(&imu, frame);
    TEST_ASSERT_EQUAL_INT16(INT16_MAX, field_s16(frame, 0U));
    TEST_ASSERT_EQUAL_INT16(INT16_MIN, field_s16(frame, 1U));

    imu.pitch_deg = NAN;
    euc_tlm_encode_imu(&imu, frame);
    TEST_ASSERT_EQUAL_INT16(0, field_s16(frame, 0U));
}

/* Real values never make 55 AA (accel +-4000 mg, gyro +-20000, pitch +-9000, roll +-18000 never have the high
 * byte 55 or AA); angles outside the physical range reach the rule. */
static void test_imu_payload_never_holds_a_frame_start(void)
{
    struct euc_tlm_imu imu = imu_at_rest();
    imu.pitch_deg = 219.30F; /* 21930 = 55 AA inside one field */
    uint8_t frame[EUC_TLM_FRAME_BYTES];
    euc_tlm_encode_imu(&imu, frame);
    TEST_ASSERT_FALSE(payload_holds_a_frame_start(frame));
    TEST_ASSERT_EQUAL_INT16(21931, field_s16(frame, 0U));

    imu.pitch_deg = 0.85F;  /* 85 = 00 55 */
    imu.roll_deg = -219.5F; /* -21950 = AA 42: 55 AA across two fields */
    euc_tlm_encode_imu(&imu, frame);
    TEST_ASSERT_FALSE(payload_holds_a_frame_start(frame));
    TEST_ASSERT_EQUAL_INT16(86, field_s16(frame, 0U));
    TEST_ASSERT_EQUAL_INT16(-21950, field_s16(frame, 1U));
}

static void test_imu_frames_yield_to_the_stock_telemetry(void)
{
    const uint32_t enough = EUC_TLM_FRAME_BYTES + EUC_TLM_IMU_RESERVE_BYTES;
    TEST_ASSERT_TRUE(euc_tlm_imu_may_send(enough, false, UINT32_MAX));
    TEST_ASSERT_FALSE(euc_tlm_imu_may_send(enough - 1U, false, UINT32_MAX)); /* the next group goes first */
    TEST_ASSERT_FALSE(euc_tlm_imu_may_send(512U, true, UINT32_MAX));         /* telemetry paused */
    TEST_ASSERT_FALSE(euc_tlm_imu_may_send(512U, false, EUC_TLM_IMU_QUIET_TICKS - 1U)); /* after V or N */
    TEST_ASSERT_TRUE(euc_tlm_imu_may_send(512U, false, EUC_TLM_IMU_QUIET_TICKS));
}

int main(void)
{
    load_capture_stream();
    reassemble_frames();

    UNITY_BEGIN();
    RUN_TEST(test_capture_reassembles_into_380_frames);
    RUN_TEST(test_every_capture_frame_encodes_back_to_the_same_bytes);
    RUN_TEST(test_group_order_matches_the_capture);
    RUN_TEST(test_capture_groups_encode_back_from_one_input_per_group);
    RUN_TEST(test_a_group_is_due_every_151_ticks);
    RUN_TEST(test_live_frame_clamps_voltage_and_bms_frame_does_not);
    RUN_TEST(test_pwm_percent_is_sign_extended);
    RUN_TEST(test_status_frame_packs_modes_and_alerts);
    RUN_TEST(test_cell_pages_map_to_cell_numbers);
    RUN_TEST(test_imu_frame_layout_and_units);
    RUN_TEST(test_imu_gyro_scale_follows_the_range);
    RUN_TEST(test_imu_values_round_and_clamp);
    RUN_TEST(test_imu_payload_never_holds_a_frame_start);
    RUN_TEST(test_imu_frames_yield_to_the_stock_telemetry);
    return UNITY_END();
}
