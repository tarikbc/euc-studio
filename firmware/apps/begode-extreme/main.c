/*
 * Begode Extreme main-board firmware: start-up in the stock order, then the main loop. Context wiring:
 * app.h; the RTC tick and the interrupt handlers: control.c.
 *
 * Start-up order (as the stock image): clocks; backup registers (boot count, trip, light); settings; GPIO
 * with the latch low; BLE serial; transport-lock check; the power-on hold (latch high, or stay off); motor
 * timer, ADC and current offsets; interrupt priorities; buzzer; IMU; BMS serial; then either the IMU
 * calibration (no calibrated marker) or the IMU offsets, LED strip and the 496 Hz tick; display; and finally
 * the FOC interrupt.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"
#include "board.h"
#include "euc/math/float_ops.h"
#include "euc/quirks.h"
#include "euc/sensors/sensors.h"
#include "euc/settings/settings.h"
#include "euc/settings/storage.h"
#include "stm32f1/device.h"
#include "stm32f1/port.h"

#define BKP_TRIP         1U
#define BKP_BOOT_COUNT   2U
#define BKP_SETTINGS     3U /* DR3..DR10 */
#define BKP_LIGHT_MODE   11U
#define ERASED           0xFFFFU
#define ONE_G_COUNTS     0x2000
#define BOOT_SETTLE_MS   100U
#define OFFSET_SETTLE_MS 10U

/* Stock busy-waits on USART1 while it prints its boot text; compat waits as long, so the 496 Hz tick starts
 * in step with the stock firmware (quirk BOOT_TEXT_WAIT; neither profile sends the text, quirk DEBUG_PRINTF).
 */
#define BOOT_TEXT_BEFORE_LATCH_US 1344U
#define BOOT_TEXT_AFTER_IMU_US    1730U

#define INIT_CURRENT_READINGS 3U
#define INIT_READING_GAP_MS   10U
#define ADC_DATA_TO_RAW       3U
#define BOOT_BEEP_TICKS       100U

#define CALIBRATION_ROUNDS        5U
#define CALIBRATION_ROUND_POLLS   110U
#define CALIBRATION_QUIET_POLLS   10U
#define CALIBRATION_POLL_MS       4U
#define CALIBRATION_SAMPLES       500.0F
#define CALIBRATION_TONE          200U
#define CALIBRATION_LOOP_MS       20U
#define CALIBRATION_RELEASE_POLLS 3U

struct app app;

struct euc_current_sample app_current_sample(void)
{
    const struct euc_current_sample sample = {(uint16_t)(stm32f1_adc1_injected(1U) >> ADC_DATA_TO_RAW),
                                              (uint16_t)(stm32f1_adc2_injected(1U) >> ADC_DATA_TO_RAW)};
    return sample;
}

/* Stock: three phase-current readings 20 ms apart once the IMU and BMS are set up. They count toward the
 * start-up sensor check, and their 60 ms set when the 496 Hz tick starts. */
static void init_current_readings(void)
{
    for (uint32_t i = 0U; i < INIT_CURRENT_READINGS; i++) {
        stm32f1_delay_ms(INIT_READING_GAP_MS);
        struct euc_current_out phases;
        euc_current_read(&app.motor.current, &app.motor.config.current, app_current_sample(), &phases);
        if (phases.start_check_failed) {
            app.foc_hard_cut_request = true;
            app.disable_request = true;
            app.raise_alerts |= EUC_ALERT_CURRENT_SENSOR;
        }
        stm32f1_delay_ms(INIT_READING_GAP_MS);
    }
}

uint8_t app_hall_lines(void)
{
    const uint32_t a = stm32f1_gpio_read(GPIOB, BOARD_PIN_HALL_A) ? 1U : 0U;
    const uint32_t b = stm32f1_gpio_read(GPIOB, BOARD_PIN_HALL_B) ? 1U : 0U;
    const uint32_t c = stm32f1_gpio_read(GPIOB, BOARD_PIN_HALL_C) ? 1U : 0U;
    return (uint8_t)(a | (b << 1U) | (c << 2U));
}

bool app_power_button_pressed(void)
{
    return !stm32f1_gpio_read(GPIOB, BOARD_PIN_POWER_BUTTON);
}

static void latch(bool on)
{
    stm32f1_gpio_write(GPIOB, BOARD_PIN_LATCH, on);
}

static void boot_text_wait(uint32_t us)
{
    if (EUC_QUIRK(BOOT_TEXT_WAIT)) {
        stm32f1_delay_us(us);
    }
}

static void board_gpio_init(void)
{
    stm32f1_gpio_clocks_on();
    stm32f1_gpio_mode(GPIOB, BOARD_PIN_HALL_A, STM32F1_PIN_INPUT_PULL);
    stm32f1_gpio_mode(GPIOB, BOARD_PIN_HALL_B, STM32F1_PIN_INPUT_PULL);
    stm32f1_gpio_mode(GPIOB, BOARD_PIN_HALL_C, STM32F1_PIN_INPUT_PULL);
    stm32f1_gpio_pull(GPIOB, BOARD_PIN_HALL_A, false);
    stm32f1_gpio_pull(GPIOB, BOARD_PIN_HALL_B, false);
    stm32f1_gpio_pull(GPIOB, BOARD_PIN_HALL_C, false);
    stm32f1_gpio_mode(GPIOB, BOARD_PIN_LIGHT, STM32F1_PIN_OUTPUT_50MHZ);
    stm32f1_gpio_mode(GPIOB, BOARD_PIN_LATCH, STM32F1_PIN_OUTPUT_50MHZ);
    latch(false);
    stm32f1_gpio_mode(GPIOB, BOARD_PIN_POWER_BUTTON, STM32F1_PIN_INPUT_PULL);
    stm32f1_gpio_pull(GPIOB, BOARD_PIN_POWER_BUTTON, false);
    stm32f1_gpio_mode(GPIOA, BOARD_PIN_HANDLE, STM32F1_PIN_INPUT_FLOATING);
}

static void load_settings(void)
{
    uint16_t backup[EUC_SETTINGS_RECORD_HALFWORDS];
    for (uint32_t i = 0U; i < EUC_SETTINGS_RECORD_HALFWORDS; i++) {
        backup[i] = stm32f1_backup_read(BKP_SETTINGS + i);
    }
    struct euc_storage_write write;
    euc_storage_load(&app.storage, ld_settings_page, backup, &app.settings, &write);
    if (write.erase_page) {
        stm32f1_flash_erase_page((uint32_t)(uintptr_t)ld_settings_page);
    }
    if (write.program_slot) {
        stm32f1_flash_program(
            (uint32_t)(uintptr_t)&ld_settings_page[(uint32_t)write.slot * EUC_SETTINGS_RECORD_HALFWORDS],
            write.record, EUC_SETTINGS_RECORD_HALFWORDS);
    }
}

/* The power-on hold check: stay on (latch high) or keep the latch low until the button is released. */
static void power_on_hold(uint16_t boot_count)
{
    struct euc_power_boot_state hold;
    struct euc_hall_out hall_out;
    euc_power_boot_init(&hold, boot_count);
    stm32f1_delay_ms(BOOT_SETTLE_MS);
    enum euc_power_boot_decision decision = EUC_POWER_BOOT_WAIT;
    while (decision == EUC_POWER_BOOT_WAIT) {
        if (!hold.warm_start) {
            stm32f1_delay_ms(euc_power_boot_poll_ms(&hold));
        }
        euc_motor_hall_step(&app.motor, app_hall_lines(), &hall_out);
        decision = euc_power_boot_poll(&hold, app_power_button_pressed(), app.motor.hall.position);
        if (hold.warm_start) {
            stm32f1_delay_ms(euc_power_boot_poll_ms(&hold));
        }
    }
    if (decision == EUC_POWER_BOOT_TURN_OFF) {
        for (;;) {
            latch(false); /* the controller loses power when the rider lets go of the button */
        }
    }
    boot_text_wait(BOOT_TEXT_BEFORE_LATCH_US);
    latch(true);
}

static void imu_offsets_load(void)
{
    for (uint32_t axis = 0U; axis < EUC_IMU_AXES; axis++) {
        app.balance_in.accel_offset[axis] = (int16_t)ld_imu_offsets_page[axis];
        app.balance_in.gyro_offset[axis] =
            euc_imu_gyro_offset_from_page((int16_t)ld_imu_offsets_page[axis + EUC_IMU_AXES]);
    }
    app.balance_in.accel_offset[2] = (int16_t)(app.balance_in.accel_offset[2] - ONE_G_COUNTS);
}

/* No calibrated marker: average 500 IMU samples on a level, still wheel, store them, then wait for the
 * power button to switch off (the stock calibration mode). */
static void imu_calibrate(void)
{
    float sums[6] = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    int16_t words[7];
    for (uint32_t round = 0U; round < CALIBRATION_ROUNDS; round++) {
        stm32f1_buzzer_set(CALIBRATION_TONE);
        for (uint32_t poll = 0U; poll < CALIBRATION_ROUND_POLLS; poll++) {
            stm32f1_delay_ms(CALIBRATION_POLL_MS);
            if (poll >= CALIBRATION_QUIET_POLLS) {
                stm32f1_buzzer_set(0U);
                stm32f1_imu_read(words);
                for (uint32_t i = 0U; i < 3U; i++) {
                    sums[i] = euc_f32_add((float)words[i], sums[i]);
                    sums[i + 3U] = euc_f32_add((float)words[i + 4U], sums[i + 3U]);
                }
            }
        }
    }
    uint16_t means[6];
    int16_t signed_means[6];
    for (uint32_t i = 0U; i < 6U; i++) {
        const float mean = euc_f32_div(sums[i], CALIBRATION_SAMPLES);
        signed_means[i] = (int16_t)(int32_t)mean;
        if (i >= EUC_IMU_AXES) {
            signed_means[i] =
                euc_imu_gyro_offset_to_page(signed_means[i]); /* flash keeps +-500 deg/s counts */
        }
        means[i] = (uint16_t)signed_means[i];
    }
    static const uint16_t marker[2] = {0x0055U, 0x00AAU};
    stm32f1_flash_rewrite((uint32_t)(uintptr_t)ld_calibrated_page, marker, 2U);
    if (euc_sensors_imu_calibration_valid(signed_means)) {
        stm32f1_flash_erase_page((uint32_t)(uintptr_t)ld_imu_offsets_page);
        stm32f1_flash_program((uint32_t)(uintptr_t)ld_imu_offsets_page, means, 6U);
    }
    /* The button switches the wheel off once it was seen released, as in the stock calibration mode. */
    uint32_t released_polls = 0U;
    for (;;) {
        if (!app_power_button_pressed()) {
            released_polls++;
        } else if (released_polls >= CALIBRATION_RELEASE_POLLS) {
            latch(false);
        } else {
            released_polls = 0U;
        }
        stm32f1_delay_ms(CALIBRATION_LOOP_MS);
    }
}

static void init(void)
{
    if (!stm32f1_clock_init()) {
        /* Clean: no crystal, no control. The latch stays low, so the wheel goes off with the button. */
        for (;;) {
        }
    }

    stm32f1_backup_init();
    if (!EUC_QUIRK(NO_WATCHDOG) && stm32f1_reset_was_watchdog()) {
        /* The tick or the FOC died on the last run, so the drive state is unknown: the bridge stays off until
         * the next power-on, and the hard-cut reminder beeps. */
        app.foc_hard_cut_request = true;
        app.disable_request = true;
    }
    euc_lights_init(&app.lights, stm32f1_backup_read(BKP_LIGHT_MODE), 0U);
    euc_strip_init(&app.strip);
    const uint16_t boot_count = (uint16_t)(stm32f1_backup_read(BKP_BOOT_COUNT) + 1U);
    stm32f1_backup_write(BKP_BOOT_COUNT, boot_count);
    app.trip_units = stm32f1_backup_read(BKP_TRIP);
    load_settings();
    euc_lights_set_strip_pattern(&app.lights, app.settings.wm_value);

    euc_motor_init(&app.motor, app.settings.drive_mode,
                   &(const struct euc_motor_config){
                       {BOARD_PHASE_OVERCURRENT_TRIP_Q15, BOARD_PHASE_OVERCURRENT_TRIP_SAMPLES}});
    board_gpio_init();
    stm32f1_ble_init();
    euc_cmd_init(&app.commands);
    euc_tlm_init(&app.telemetry, 0U, false);
    euc_balance_init(&app.balance);
    euc_alarm_init(&app.alarms);
    euc_power_init(&app.power);
    euc_bms_init(&app.bms);
    euc_sensors_init(&app.sensors);
    euc_service_init(&app.service);
    euc_supervisor_init(&app.supervisor);
    euc_riderless_init(&app.riderless);
    if (app.settings.lock_state == EUC_SETTINGS_LOCKED) {
        app.foc_hard_cut_request = true;
        app.disable_request = true;
    }

    power_on_hold(boot_count);

    stm32f1_nvic_init();
    stm32f1_adc_init();
    struct stm32f1_current_sums sums;
    stm32f1_adc_measure_offsets(&sums); /* bridge off: the timer is not running yet */
    stm32f1_pwm_init(app.motor.timing.pwm_period_counts, app.motor.timing.dead_time_counts);
    const struct euc_current_offsets_in offsets = {sums.adc1_rank1,          sums.adc1_rank2,
                                                   sums.adc2_rank1,          boot_count,
                                                   stm32f1_backup_read(12U), stm32f1_backup_read(13U)};
    euc_current_set_offsets(&app.motor.current, &offsets);
    stm32f1_nvic_enable(USART1_IRQn, 2U, 1U);
    stm32f1_buzzer_init();

    stm32f1_imu_bus_init();
    stm32f1_delay_ms(OFFSET_SETTLE_MS);
    if (!stm32f1_imu_init(euc_imu_gyro_config())) {
        app.foc_hard_cut_request = true;
        app.disable_request = true;
        app.raise_alerts |= EUC_ALERT_IMU_FAULT;
        app.settings.imu_fault = 1U;
    }
    boot_text_wait(BOOT_TEXT_AFTER_IMU_US);
    stm32f1_bms_init(app.bms_reply, EUC_BMS_REPLY_BYTES);
    init_current_readings();

    if (ld_calibrated_page[0] == ERASED) {
        imu_calibrate();
    }
    imu_offsets_load();
    stm32f1_led_strip_init();
    stm32f1_display_init();
    stm32f1_rtc_tick_init();
    euc_beeper_beep(&app.alarms.beeper, BOOT_BEEP_TICKS);
    stm32f1_adc_enable_interrupt();
}

int main(void)
{
    init();
    for (;;) {
        app_main_loop_pass();
    }
}
