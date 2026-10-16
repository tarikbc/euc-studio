/*
 * Interrupt handlers, the 496 Hz RTC tick and the main-loop pass (see app.h for the contexts).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"
#include "board.h"
#include "euc/display/display.h"
#include "euc/math/fixed.h"
#include "stm32f1/device.h"
#include "stm32f1/port.h"

#define BKP_TRIP              1U
#define BKP_SETTINGS          3U
#define BALANCE_RESTART_TICKS 10
#define DISPLAY_PERIOD_TICKS  100U
#define SERVICE_WAIT_MS       200U
#define UNITS_MILES           1U

/* Interrupts ----------------------------------------------------------------------------------------------
 */

void TIM8_UP_IRQHandler(void);
void ADC1_2_IRQHandler(void);
void USART1_IRQHandler(void);
void RTC_IRQHandler(void);

static void hall_step(void)
{
    struct euc_hall_out out;
    euc_motor_hall_step(&app.motor, app_hall_lines(), &out);
    if (out.odometer_step) {
        app.odometer_steps++;
    }
    if (out.fault_raised || out.line_fault_raised) {
        app.motor_alerts |= (uint8_t)EUC_ALERT_HALL_SENSOR;
    }
}

void TIM8_UP_IRQHandler(void)
{
    stm32f1_pwm_clear_update();
    stm32f1_gpio_write(GPIOB, BOARD_PIN_LIGHT, euc_lights_pwm_step(&app.lights));
    if (stm32f1_pwm_counting_down()) {
        /* overflow: the sample at the centre of the off-time is not used by the FOC */
        stm32f1_adc_start();
        hall_step();
    } else {
        hall_step();
        app.foc_pending = true;
        stm32f1_adc_start();
    }
}

void ADC1_2_IRQHandler(void)
{
    if ((!stm32f1_adc_take_end_of_conversion()) || (!app.foc_pending)) {
        return;
    }
    app.foc_pending = false;

    struct euc_motor_foc_in in;
    in.sample = app_current_sample();
    in.speed_dm_per_s = app.foc_speed_dm_per_s;
    in.drive_disabled = app.foc_drive_disabled;
    in.hard_cut_request = app.foc_hard_cut_request;
    struct euc_motor_foc_out out;
    euc_motor_foc_step(&app.motor, &in, &out);

    uint16_t compare[3];
    compare[BOARD_MOTOR_PHASE_A_TIM8_CHANNEL - 1U] = out.duty.a;
    compare[BOARD_MOTOR_PHASE_B_TIM8_CHANNEL - 1U] = out.duty.b;
    compare[BOARD_MOTOR_PHASE_C_TIM8_CHANNEL - 1U] = out.duty.c;
    stm32f1_pwm_set(compare[0], compare[1], compare[2]);
    if (out.outputs_enabled != app.bridge_on) {
        stm32f1_pwm_outputs(out.outputs_enabled);
        app.bridge_on = out.outputs_enabled;
    }
    if (out.start_balance) {
        app.start_balance = true;
    }
    if (out.stop_drive) {
        app.stop_drive = true;
    }
    app.motor_alerts |= out.error_bits;
}

void USART1_IRQHandler(void)
{
    uint8_t byte = 0U;
    if (stm32f1_ble_service(&byte)) {
        const struct euc_cmd_in in = {app.balance.speed_cm_per_s};
        struct euc_cmd_out out;
        euc_cmd_receive_byte(&app.commands, &in, byte, &out);
        app_apply_commands(&out);
    }
}

void RTC_IRQHandler(void)
{
    stm32f1_rtc_tick_acknowledge();
    app_rtc_tick();
}

/* Commands -------------------------------------------------------------------------------------------------
 */

static void apply_service(enum euc_cmd_action_kind kind)
{
    if (kind == EUC_CMD_FIRMWARE_UPDATE_REQUEST) {
        euc_service_request(&app.service, EUC_SERVICE_FIRMWARE_UPDATE);
    } else if (kind == EUC_CMD_CALIBRATION_REQUEST) {
        euc_service_request(&app.service, EUC_SERVICE_CALIBRATION);
    } else {
        euc_service_request(&app.service, EUC_SERVICE_DRIVE_MODE_TOGGLE);
    }
}

void app_apply_commands(const struct euc_cmd_out *out)
{
    struct euc_settings *s = &app.settings;
    for (uint32_t i = 0U; i < out->action_count; i++) {
        const uint8_t value = out->actions[i].value;
        switch (out->actions[i].kind) {
        case EUC_CMD_HORN:
            app.horn_request = true;
            break;
        case EUC_CMD_LIGHTS:
            euc_lights_set_mode(&app.lights, value);
            break;
        case EUC_CMD_PEDAL_MODE:
            s->pedal_mode = value;
            break;
        case EUC_CMD_SPEED_ALARM_MODE:
            s->speed_alarm_mode = value;
            break;
        case EUC_CMD_ROLL_CUTOFF_MODE:
            s->roll_cutoff_mode = value;
            break;
        case EUC_CMD_UNITS:
            s->units_miles = value;
            break;
        case EUC_CMD_PWM_TILTBACK:
            s->pwm_tiltback = value;
            break;
        case EUC_CMD_TILTBACK_SPEED_KMH:
            s->tiltback_speed_kmh = value;
            break;
        case EUC_CMD_PWM_ALARM_PERCENT:
            s->pwm_alarm_percent = value;
            break;
        case EUC_CMD_BEEPER_VOLUME:
            s->beeper_volume = value;
            break;
        case EUC_CMD_WM_VALUE:
            s->wm_value = value;
            euc_lights_set_strip_pattern(&app.lights, value);
            break;
        case EUC_CMD_TURN_COMPENSATION:
            s->turn_compensation = value;
            break;
        case EUC_CMD_PEDAL_TRIM:
            s->pedal_trim = value;
            break;
        case EUC_CMD_WC_VALUE:
            s->wc_value = value;
            break;
        case EUC_CMD_ROLL_LIMIT_STEP:
            s->roll_limit_step = value;
            break;
        default:
            apply_service(out->actions[i].kind);
            break;
        }
    }
}

/* RTC tick -------------------------------------------------------------------------------------------------
 */

static int32_t to_display_units(int32_t value)
{
    return (app.settings.units_miles == UNITS_MILES) ? euc_sensors_miles_i32(value) : value;
}

static uint32_t to_display_units_u32(uint32_t value)
{
    return (app.settings.units_miles == UNITS_MILES) ? euc_sensors_miles_u32(value) : value;
}

static void display_tick(void)
{
    app.display_ticks++;
    if (app.display_ticks < DISPLAY_PERIOD_TICKS) {
        return;
    }
    app.display_ticks = 0U;
    const struct euc_display_in in = {
        sat_i32_to_i16(to_display_units(app.balance.speed_mm_per_s)),
        to_display_units_u32(euc_settings_odometer_units(&app.settings)),
        app.settings.pedal_mode,
        app.sensors.board_temperature_filtered_c,
        app.sensors.battery_voltage_cv,
        app.alarm_out.time_base,
    };
    app.display_speed = (uint8_t)(euc_display_speed(in.speed_mm_per_s) & 0xFFU);
    uint8_t grids[EUC_DISPLAY_GRIDS];
    euc_display_render(&in, grids);
    stm32f1_display_send(grids);
}

/* LED strip: the tick part reads the time base before this tick's alarm task, the render the one after. */
static void strip_tick(void)
{
    const struct euc_strip_tick_in in = {app.alarm_out.time_base, app.display_speed,
                                         app.balance.speed_mm_per_s};
    euc_strip_tick(&app.strip, &in);
}

static void strip_render(void)
{
    const struct euc_strip_render_in in = {app.alarm_out.time_base, app.display_speed,
                                           app.balance_out.roll_deg, app.lights.strip_pattern};
    euc_strip_render(&app.strip, &in);
    const uint32_t changed = euc_strip_take_changed(&app.strip);
    for (uint32_t i = 0U; i < EUC_STRIP_LEDS; i++) {
        if (((changed >> i) & 1U) != 0U) {
            const struct euc_strip_pixel pixel = app.strip.pixels[i];
            stm32f1_led_strip_set(i, pixel.red, pixel.green, pixel.blue);
        }
    }
}

/* Events from the motor interrupts. */
static void take_motor_events(void)
{
    static uint16_t odometer_steps_taken;

    if (app.start_balance) {
        app.start_balance = false;
        euc_balance_restart_countdown(&app.balance, BALANCE_RESTART_TICKS);
    }
    if (app.stop_drive) {
        app.stop_drive = false;
        app.balance.command = 0.0F;
        app.disable_request = true;
    }
    const uint16_t seen = app.odometer_steps;
    if (seen != odometer_steps_taken) {
        while (odometer_steps_taken != seen) {
            euc_settings_add_odometer_unit(&app.settings, &app.trip_units);
            odometer_steps_taken++;
        }
        stm32f1_backup_write(BKP_TRIP, app.trip_units);
    }
}

static void read_sensors(void)
{
    euc_sensors_battery(&app.sensors, stm32f1_adc1_injected(2U));
    euc_sensors_bus_current(&app.sensors, stm32f1_adc2_injected(2U));
    int16_t words[7];
    stm32f1_imu_read(words);
    for (uint32_t axis = 0U; axis < EUC_IMU_AXES; axis++) {
        app.balance_in.accel_raw[axis] = words[axis];
        app.balance_in.gyro_raw[axis] = words[axis + 4U];
    }
    euc_sensors_board_temperature(&app.sensors, stm32f1_adc1_injected(3U));
}

static void balance(void)
{
    struct euc_balance_in *in = &app.balance_in;
    const struct euc_settings *s = &app.settings;
    in->settings.pedal_mode = s->pedal_mode;
    in->settings.drive_mode = app.motor.drive_mode;
    in->settings.tiltback_speed_kmh = s->tiltback_speed_kmh;
    in->settings.pwm_tiltback_enabled = s->pwm_tiltback != 0U;
    in->settings.pedal_trim = s->pedal_trim;
    in->settings.turn_compensation = s->turn_compensation;
    in->settings.roll_limit_step = s->roll_limit_step;
    in->settings.roll_cutoff_mode = s->roll_cutoff_mode;
    in->battery_voltage_cv = app.sensors.battery_voltage_cv;
    in->board_temperature_mpu6050 = app.sensors.board_temperature_mpu6050;
    in->handle_button_released = stm32f1_gpio_read(GPIOA, BOARD_PIN_HANDLE);
    in->hall_steps_per_window = app.motor.hall.steps_per_window;
    in->hall_steps_11_windows = app.motor.hall.steps_11_windows;
    in->hall_period_speed = app.motor.hall.period_speed;
    in->motor_current_lpf_ma = app.motor.iq_lpf_ma;
    in->motor_voltage_q = app.motor.vq_active;
    in->pwm_alarm_active = app.alarm_out.pwm_alarm;
    in->hard_inhibit = app.motor.hard_cut || app.foc_hard_cut_request;
    in->hall_fault = app.motor.hall.fault;
    in->hall_degraded = app.motor.hall.line_fault;
    in->disable_request = app.disable_request;
    app.disable_request = false;

    euc_motor_rtc_step(&app.motor, app.balance.speed_dm_per_s);
    euc_balance_tick(&app.balance, in, &app.balance_out);
    euc_motor_set_command(&app.motor, app.balance_out.motor_command);
    app.foc_speed_dm_per_s = app.balance.speed_dm_per_s;
    app.foc_drive_disabled = app.balance_out.motor_disabled;
    for (uint32_t i = 0U; i < app.balance_out.beep_count; i++) {
        euc_beeper_beep(&app.alarms.beeper, app.balance_out.beep_ticks[i]);
    }
}

static void service(void)
{
    const bool no_rider = (app.balance.start_state != 1) || app.balance.motor_disabled;
    const struct euc_service_in in = {app.balance.command, no_rider, app_power_button_pressed()};
    struct euc_service_out out;
    euc_service_tick(&app.service, &in, &out);
    if (out.reply != EUC_SERVICE_REPLY_NONE) {
        app.service_reply = (uint8_t)out.reply;
    }
    if (out.beep_ticks != 0U) {
        euc_beeper_beep(&app.alarms.beeper, out.beep_ticks);
    }
    if (out.motor_off) {
        app.foc_hard_cut_request = true;
        app.disable_request = true;
    }
    if (out.enter_bootloader) {
        app.bootloader_request = true;
    }
    if (out.erase_calibration) {
        app.erase_calibration_request = true;
        app.alarms.hard_cut_ticks = 0U;
    }
    if (out.toggle_drive_mode) {
        app.settings.drive_mode = (app.settings.drive_mode != EUC_MOTOR_CURRENT_MODE)
                                      ? EUC_MOTOR_CURRENT_MODE
                                      : EUC_MOTOR_VOLTAGE_MODE;
        app.save_request = true;
        app.power_off_wait = true; /* stock: 200 ms between the save and the latch */
        app.power_off_request = true;
    }
}

static void alarms(void)
{
    const struct euc_alarm_in in = {
        .speed_dm_per_s = app.balance.speed_dm_per_s,
        .battery_voltage_cv = app.sensors.battery_voltage_cv,
        .board_temperature_mpu6050 = app.sensors.board_temperature_mpu6050,
        .motor_voltage_q = app.motor.vq_active,
        .beeper_volume = app.settings.beeper_volume,
        .pwm_alarm_percent = app.settings.pwm_alarm_percent,
        .speed_alarm_mode = app.settings.speed_alarm_mode,
        .hard_cut = app.motor.hard_cut || app.foc_hard_cut_request,
        .powering_off = app.power.powering_off,
        .hall_fault = app.motor.hall.fault,
        .bms_fault = app.bms.fault,
        .horn_request = app.horn_request,
        .handle_double_beep = app.balance_out.handle_double_beep,
        .transport_locked = app.settings.lock_state == EUC_SETTINGS_LOCKED,
        .raise_alerts = (uint8_t)(app.raise_alerts | app.motor_alerts),
    };
    app.horn_request = false;
    app.raise_alerts = 0U;
    euc_alarm_tick(&app.alarms, &in, &app.alarm_out);
    app.settings.pwm_alarm_percent = app.alarm_out.pwm_alarm_percent;
}

static void lights_and_buttons(void)
{
    const bool locked = app.settings.lock_state == EUC_SETTINGS_LOCKED;
    /* A press that confirms a service sequence is not a light-mode press. */
    const bool light_press = app_power_button_pressed() && !euc_service_confirming(&app.service);
    const struct euc_lights_in light_in = {light_press, app.balance.speed_dm_per_s, app.alarm_out.time_base,
                                           locked};
    euc_lights_tick(&app.lights, &light_in, &app.lights_out);
    if (app.lights_out.wm_changed) {
        app.settings.wm_value = app.lights_out.strip_pattern;
    }

    const struct euc_power_in power_in = {
        app_power_button_pressed(),     !stm32f1_gpio_read(GPIOA, BOARD_PIN_HANDLE),
        app.balance.speed_cm_per_s,     app.motor.hall.position,
        app.balance_out.motor_disabled, app.settings.lock_state,
    };
    euc_power_tick(&app.power, &power_in, &app.power_out);
    app.settings.lock_state = app.power_out.lock_state;
    if (app.power_out.hard_cut) {
        app.foc_hard_cut_request = true;
    }
    if (app.power_out.disable_drive) {
        app.disable_request = true;
    }
    for (uint32_t i = 0U; i < app.power_out.beep_count; i++) {
        euc_beeper_beep(&app.alarms.beeper, app.power_out.beep_ticks[i]);
    }
    if (app.power_out.save_settings) {
        app.save_request = true;
    }
    if (app.power_out.power_off) {
        app.power_off_request = true;
    }
}

static void storage(void)
{
    if (app.storage_write_pending) {
        return;
    }
    const struct euc_storage_tick_in in = {app.balance.speed_cm_per_s, app.balance_out.motor_disabled};
    struct euc_storage_write write;
    euc_storage_tick(&app.storage, &app.settings, &in, &write);
    if (write.write_backup || write.program_slot) {
        app.storage_write = write;
        app.storage_write_pending = true;
    }
}

void app_rtc_tick(void)
{
    app.rtc_ticks++;
    strip_tick();
    euc_tlm_count_tick(&app.telemetry);
    euc_bms_tick(&app.bms);
    display_tick();
    take_motor_events();
    read_sensors();

    const struct euc_cmd_in cmd_in = {app.balance.speed_cm_per_s};
    struct euc_cmd_out cmd_out;
    euc_cmd_tick(&app.commands, &cmd_in, &cmd_out);
    app_apply_commands(&cmd_out);

    balance();
    service();
    alarms();
    lights_and_buttons();
    storage();
    stm32f1_buzzer_set(app.alarms.beeper.compare);
    strip_render();
}

/* Main loop
 * -------------------------------------------------------------------------------------------------- */

static void send_reply(enum euc_cmd_reply reply)
{
    static const char banner[] = BOARD_FIRMWARE_BANNER;
    static const char model[] = BOARD_MODEL_REPLY;
    if (reply == EUC_CMD_REPLY_VERSION) {
        (void)stm32f1_ble_send((const uint8_t *)banner, sizeof(banner) - 1U);
    } else if (reply == EUC_CMD_REPLY_NAME) {
        (void)stm32f1_ble_send((const uint8_t *)model, sizeof(model) - 1U);
    } else {
        /* Speed-alarm acknowledgements and the debug dump are stock debug text: not sent (DEBUG_PRINTF). */
    }
}

/* Clean service-sequence replies (euc/service/service.h). */
static void send_service_reply(void)
{
    static const char confirm[] = EUC_SERVICE_REPLY_CONFIRM_TEXT;
    static const char ok[] = EUC_SERVICE_REPLY_OK_TEXT;
    static const char denied[] = EUC_SERVICE_REPLY_DENIED_TEXT;
    const uint8_t reply = app.service_reply;
    app.service_reply = (uint8_t)EUC_SERVICE_REPLY_NONE;
    if (reply == (uint8_t)EUC_SERVICE_REPLY_CONFIRM) {
        (void)stm32f1_ble_send((const uint8_t *)confirm, sizeof(confirm) - 1U);
    } else if (reply == (uint8_t)EUC_SERVICE_REPLY_OK) {
        (void)stm32f1_ble_send((const uint8_t *)ok, sizeof(ok) - 1U);
    } else if (reply == (uint8_t)EUC_SERVICE_REPLY_DENIED) {
        (void)stm32f1_ble_send((const uint8_t *)denied, sizeof(denied) - 1U);
    } else {
        /* nothing to send */
    }
}

static void telemetry_inputs(struct euc_tlm_in *in)
{
    const struct euc_settings *s = &app.settings;
    (void)memset(in, 0, sizeof(*in));
    const int32_t voltage_dv = app.sensors.battery_voltage_cv / 10;
    in->battery_voltage_dv = (uint16_t)voltage_dv;
    in->speed_cm_per_s = sat_i32_to_i16(to_display_units(app.balance.speed_mm_per_s) / 10);
    in->range_estimate_km = (uint16_t)to_display_units_u32(app.sensors.range_estimate_km);
    in->trip_distance_m = (uint16_t)to_display_units_u32(app.trip_units);
    in->phase_current_ca = sat_i32_to_i16(app.motor.iq_01a * 10);
    const float board = app.sensors.board_temperature_mpu6050;
    in->board_temperature_mpu6050 = ((board > -32768.0F) && (board < 32767.0F)) ? (int16_t)board : 0;
    in->turn_compensation = s->turn_compensation;
    in->pedal_trim = s->pedal_trim;
    in->pwm_tiltback_enabled = s->pwm_tiltback != 0U;
    in->drive_mode = s->drive_mode;
    in->beeper_volume = s->beeper_volume;
    in->pwm_alarm_percent = s->pwm_alarm_percent;
    in->frame01_speed_copy_dm_per_s = app.balance.frame01_speed_copy_dm_per_s;
    in->bms_temperature_max_c = app.bms.temperature_max_c;
    in->bms_temperature_min_c = app.bms.temperature_min_c;
    for (uint32_t block = 0U; block < EUC_TLM_BMS_BLOCKS; block++) {
        in->bms_blocks[block].current_da = app.bms.blocks[block].current_da;
        in->bms_blocks[block].status_word = app.bms.blocks[block].status_word;
        in->bms_blocks[block].word42_raw = app.bms.blocks[block].word42_raw;
    }
    (void)memcpy(in->cell_voltage_mv, app.bms.cell_mv, sizeof(in->cell_voltage_mv));
    in->odometer_m = to_display_units_u32(euc_settings_odometer_units(s));
    in->pedal_mode = s->pedal_mode;
    in->speed_alarm_mode = s->speed_alarm_mode;
    in->roll_cutoff_mode = s->roll_cutoff_mode;
    in->units_miles = s->units_miles == UNITS_MILES;
    in->auto_power_off_remaining_s = app.power_out.auto_off_remaining_s;
    in->tiltback_speed_kmh = s->tiltback_speed_kmh;
    in->wm_value = s->wm_value;
    in->alert_flags = app.alarm_out.alerts;
    in->light_state = app.lights_out.light_state;
    const double battery_current = (double)app.sensors.bus_current_filtered_ca * 1.2;
    in->battery_current_ca =
        ((battery_current > -32768.0) && (battery_current < 32767.0)) ? (int16_t)battery_current : 0;
    in->wc_value = s->wc_value;
    in->wx_value = s->roll_limit_step;
    in->pwm_percent = euc_motor_pwm_percent(&app.motor);
}

static void telemetry(void)
{
    if (!euc_tlm_group_due(&app.telemetry)) {
        return;
    }
    struct euc_cmd_slot_out slot;
    euc_cmd_telemetry_slot(&app.commands, &slot);
    send_reply(slot.reply);
    send_service_reply();
    if (slot.send_frames) {
        static struct euc_tlm_in in;
        static struct euc_tlm_group group;
        telemetry_inputs(&in);
        euc_tlm_build_group(&app.telemetry, &in, &group);
        for (uint32_t frame = 0U; frame < EUC_TLM_GROUP_FRAMES; frame++) {
            (void)stm32f1_ble_send(group.frames[frame], EUC_TLM_FRAME_BYTES);
        }
    }
}

static void bms_link(void)
{
    const struct euc_bms_in in = {stm32f1_bms_bytes_received(EUC_BMS_REPLY_BYTES), app.bms_reply};
    struct euc_bms_out out;
    euc_bms_poll(&app.bms, &in, &out);
    if (out.send_request) {
        stm32f1_bms_request(euc_bms_request, EUC_BMS_REQUEST_BYTES, EUC_BMS_REPLY_BYTES);
    }
}

static void storage_write(const struct euc_storage_write *write)
{
    if (write->write_backup) {
        for (uint32_t i = 0U; i < EUC_SETTINGS_RECORD_HALFWORDS; i++) {
            stm32f1_backup_write(BKP_SETTINGS + i, write->record[i]);
        }
    }
    const uint32_t page = (uint32_t)(uintptr_t)ld_settings_page;
    if (write->erase_page) {
        stm32f1_flash_erase_page(page);
    }
    if (write->program_slot) {
        const uint32_t slot = page + ((uint32_t)write->slot * EUC_SETTINGS_RECORD_HALFWORDS * 2U);
        if (write->rewrite_page) {
            stm32f1_flash_rewrite(slot, write->record, EUC_SETTINGS_RECORD_HALFWORDS);
        } else {
            stm32f1_flash_program(slot, write->record, EUC_SETTINGS_RECORD_HALFWORDS);
        }
    }
}

void app_main_loop_pass(void)
{
    bms_link();
    telemetry();

    if (app.storage_write_pending) {
        storage_write(&app.storage_write);
        app.storage_write_pending = false;
    }
    if (app.save_request) {
        app.save_request = false;
        struct euc_storage_write write;
        euc_storage_save(&app.storage, &app.settings, &write);
        storage_write(&write);
    }
    if (app.erase_calibration_request) {
        app.erase_calibration_request = false;
        stm32f1_flash_erase_page((uint32_t)(uintptr_t)ld_calibrated_page);
    }
    if (app.power_off_request) {
        app.power_off_request = false;
        if (app.power_off_wait) {
            stm32f1_delay_ms(SERVICE_WAIT_MS);
        }
        stm32f1_gpio_write(GPIOB, BOARD_PIN_LATCH, false);
    }
    if (app.bootloader_request) {
        /* Motor already off (service). The bootloader finds the erased request page and waits for the image.
         */
        stm32f1_flash_erase_page((uint32_t)(uintptr_t)ld_update_request_page);
        stm32f1_delay_ms(SERVICE_WAIT_MS);
        stm32f1_system_reset();
    }
}
