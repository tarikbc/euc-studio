/*
 * Table-driven tests for core/protocol/commands.c.
 *
 * Each case is a script run against a fresh parser in the compat or clean profile (the host build switches
 * quirks at run time through the SIL port). Script tokens, separated by spaces:
 *   one character    send that byte                      e.g.  b   W   "   !
 *   tick:N           run N RTC ticks (2.016 ms each)
 *   speed:N          set the wheel speed, 0.01 m/s, signed
 *   slot             one telemetry group time
 * The driver logs every action, reply and slot decision as text, and the case compares the log:
 *   action          <name>=<value>                        e.g.  pedal_mode=2
 *   reply           reply=<name>
 *   telemetry slot  slot=frames | slot=skip | slot=<reply> | slot=frames+<reply>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "euc/protocol/commands.h"
#include "euc/sil.h"
#include "unity.h"

#define LOG_BYTES   512U
#define TOKEN_BYTES 32U

enum profile { COMPAT, CLEAN };

struct script_case {
    const char *name;
    enum profile profile;
    const char *script;
    const char *expected_log;
};

static const char *const action_names[] = {
    [EUC_CMD_HORN] = "horn",
    [EUC_CMD_LIGHTS] = "lights",
    [EUC_CMD_PEDAL_MODE] = "pedal_mode",
    [EUC_CMD_SPEED_ALARM_MODE] = "speed_alarm_mode",
    [EUC_CMD_ROLL_CUTOFF_MODE] = "roll_cutoff_mode",
    [EUC_CMD_UNITS] = "units",
    [EUC_CMD_PWM_TILTBACK] = "pwm_tiltback",
    [EUC_CMD_TILTBACK_SPEED_KMH] = "tiltback_kmh",
    [EUC_CMD_PWM_ALARM_PERCENT] = "pwm_alarm_percent",
    [EUC_CMD_BEEPER_VOLUME] = "beeper_volume",
    [EUC_CMD_WM_VALUE] = "wm",
    [EUC_CMD_TURN_COMPENSATION] = "turn_compensation",
    [EUC_CMD_PEDAL_TRIM] = "pedal_trim",
    [EUC_CMD_WC_VALUE] = "wc",
    [EUC_CMD_ROLL_LIMIT_STEP] = "roll_limit_step",
    [EUC_CMD_CALIBRATION_REQUEST] = "calibration_request",
    [EUC_CMD_FIRMWARE_UPDATE_REQUEST] = "firmware_update_request",
    [EUC_CMD_DRIVE_MODE_TOGGLE_REQUEST] = "drive_mode_toggle_request",
};

static const char *const reply_names[] = {
    [EUC_CMD_REPLY_NONE] = "none",
    [EUC_CMD_REPLY_VERSION] = "version",
    [EUC_CMD_REPLY_NAME] = "name",
    [EUC_CMD_REPLY_SPEED_ALARM_ON] = "speed_alarm_on",
    [EUC_CMD_REPLY_LEVEL1_ALARM_OFF] = "level1_alarm_off",
    [EUC_CMD_REPLY_LEVEL2_ALARM_OFF] = "level2_alarm_off",
    [EUC_CMD_REPLY_LEVEL3_ALARM_OFF] = "level3_alarm_off",
    [EUC_CMD_REPLY_STOP_FIRST] = "stop_first",
    [EUC_CMD_REPLY_DEBUG_DUMP] = "debug_dump",
};

static char log_text[LOG_BYTES];

static void log_append(const char *entry)
{
    const size_t used = strlen(log_text);
    (void)snprintf(&log_text[used], LOG_BYTES - used, "%s%s", (used > 0U) ? " " : "", entry);
}

static void log_out(const struct euc_cmd_out *out)
{
    char entry[TOKEN_BYTES * 2U];
    for (uint8_t index = 0U; index < out->action_count; index++) {
        (void)snprintf(entry, sizeof(entry), "%s=%u", action_names[out->actions[index].kind],
                       (unsigned)out->actions[index].value);
        log_append(entry);
    }
    if (out->reply != EUC_CMD_REPLY_NONE) {
        (void)snprintf(entry, sizeof(entry), "reply=%s", reply_names[out->reply]);
        log_append(entry);
    }
}

static void log_slot(const struct euc_cmd_slot_out *slot)
{
    char entry[TOKEN_BYTES * 2U];
    if (slot->reply == EUC_CMD_REPLY_NONE) {
        (void)snprintf(entry, sizeof(entry), "slot=%s", slot->send_frames ? "frames" : "skip");
    } else {
        (void)snprintf(entry, sizeof(entry), "slot=%s%s", slot->send_frames ? "frames+" : "",
                       reply_names[slot->reply]);
    }
    log_append(entry);
}

static void select_profile(enum profile profile)
{
    euc_sil_quirk_set_mask(
        euc_sil_quirk_profile_mask((profile == COMPAT) ? EUC_SIL_PROFILE_COMPAT : EUC_SIL_PROFILE_CLEAN));
}

/* Runs one script on a fresh parser and leaves the log in log_text. */
static void run_script(enum profile profile, const char *script)
{
    struct euc_cmd_state state;
    struct euc_cmd_in in = {.speed_cm_per_s = 0};
    struct euc_cmd_out out;
    char token[TOKEN_BYTES];
    const char *cursor = script;

    select_profile(profile);
    euc_cmd_init(&state);
    log_text[0] = '\0';

    while (*cursor != '\0') {
        size_t length = strcspn(cursor, " ");
        TEST_ASSERT_TRUE(length < TOKEN_BYTES);
        memcpy(token, cursor, length);
        token[length] = '\0';
        cursor += length;
        cursor += (*cursor == ' ') ? 1 : 0;

        if (length == 1U) {
            euc_cmd_receive_byte(&state, &in, (uint8_t)token[0], &out);
            log_out(&out);
        } else if (strncmp(token, "tick:", 5U) == 0) {
            const long ticks = strtol(&token[5], NULL, 10);
            for (long tick = 0; tick < ticks; tick++) {
                euc_cmd_tick(&state, &in, &out);
                log_out(&out);
            }
        } else if (strncmp(token, "speed:", 6U) == 0) {
            in.speed_cm_per_s = (int16_t)strtol(&token[6], NULL, 10);
        } else if (strcmp(token, "slot") == 0) {
            struct euc_cmd_slot_out slot;
            euc_cmd_telemetry_slot(&state, &slot);
            log_slot(&slot);
        } else {
            TEST_FAIL_MESSAGE(token);
        }
    }
}

static const struct script_case cases[] = {
    /* Single-byte actions applied on reception, no speed guard */
    {"lights", COMPAT, "Q E T", "lights=1 lights=0 lights=2"},
    {"roll cut-off modes", COMPAT, "> = <", "roll_cutoff_mode=0 roll_cutoff_mode=1 roll_cutoff_mode=2"},
    {"units", COMPAT, "m g", "units=1 units=0"},
    {"pwm tiltback", COMPAT, "speed:900 e x", "pwm_tiltback=1 pwm_tiltback=0"},

    /* Ride settings while moving (SETTINGS_WHILE_MOVING): stock applies them, clean only at standstill */
    {"roll mode while moving", COMPAT, "speed:300 >", "roll_cutoff_mode=0"},
    {"roll modes refused while moving", CLEAN, "speed:300 > = <",
     "reply=stop_first reply=stop_first reply=stop_first"},
    {"roll mode at the speed limit", CLEAN, "speed:-49 <", "roll_cutoff_mode=2"},
    {"pwm tiltback refused while moving", CLEAN, "speed:50 e x", "reply=stop_first reply=stop_first"},
    {"pwm tiltback at standstill", CLEAN, "e x", "pwm_tiltback=1 pwm_tiltback=0"},
    {"W settings while moving", COMPAT, "speed:300 W R 3 W U 5 W X 2 W C 1 W M 1",
     "turn_compensation=3 pedal_trim=5 roll_limit_step=2 wc=1 wm=1"},
    {"W settings refused while moving", CLEAN, "speed:300 W R 3 W U 5 W X 2 W C 1 W M 1",
     "reply=stop_first reply=stop_first reply=stop_first reply=stop_first reply=stop_first"},
    {"W settings at standstill", CLEAN, "W R 3 W U 5 W X 2 W C 1 W M 1",
     "turn_compensation=3 pedal_trim=5 roll_limit_step=2 wc=1 wm=1"},
    {"volume, lights, units and horn while moving", CLEAN, "speed:900 W B 4 Q g b tick:1",
     "beeper_volume=4 lights=1 units=0 horn=0"},

    /* Pending commands, handled at the next tick */
    {"horn", COMPAT, "b tick:1", "horn=0"},
    {"horn while moving", COMPAT, "speed:900 b tick:1", "horn=0"},
    {"pedal modes", COMPAT, "h tick:1 f tick:1 s tick:1", "pedal_mode=2 pedal_mode=1 pedal_mode=0"},
    {"pedal mode at the speed limit", COMPAT, "speed:-49 h tick:1", "pedal_mode=2"},
    {"pedal mode dropped while moving", COMPAT, "speed:50 h tick:1 speed:0 tick:1", ""},
    {"tiltback off", COMPAT, "\" tick:1", "tiltback_kmh=200"},
    {"tiltback off deferred until slow", COMPAT, "speed:300 \" tick:5 speed:0 tick:1", "tiltback_kmh=200"},
    {"tiltback off dropped while moving", CLEAN, "speed:300 \" tick:5 speed:0 tick:1", ""},
    {"last byte wins", COMPAT, "b h tick:1", "pedal_mode=2"},
    {"no byte is lost", CLEAN, "b h tick:2", "horn=0 pedal_mode=2"},
    {"queue keeps eight commands", CLEAN, "b b b b b b b b b tick:9",
     "horn=0 horn=0 horn=0 horn=0 horn=0 horn=0 horn=0 horn=0"},

    /* Speed-alarm modes: need |v| <= 49 and 101 ticks at rest */
    {"alarm all on", COMPAT, "tick:101 o tick:1", "speed_alarm_mode=0 reply=speed_alarm_on"},
    {"alarm level 1 off", COMPAT, "tick:101 u tick:1", "speed_alarm_mode=1 reply=level1_alarm_off"},
    {"alarm level 2 off", COMPAT, "tick:101 i tick:1", "speed_alarm_mode=2 reply=level2_alarm_off"},
    {"alarm level 3 off", COMPAT, "tick:101 I tick:1", "speed_alarm_mode=3 reply=level3_alarm_off"},
    {"alarm waits for rest", COMPAT, "o tick:100", ""},
    {"alarm applies after rest", COMPAT, "o tick:101", "speed_alarm_mode=0 reply=speed_alarm_on"},
    {"alarm not rested", CLEAN, "o tick:1", "reply=stop_first"},
    {"alarm refused while moving", COMPAT, "speed:50 o tick:1", "reply=stop_first"},
    {"level 3 refused silently", COMPAT, "speed:50 I tick:1", ""},

    /* 'W' sequences */
    {"tiltback speed", COMPAT, "W Y 2 5", "tiltback_kmh=25"},
    {"tiltback speed 00", COMPAT, "W Y 0 0", "tiltback_kmh=0"},
    {"tiltback speed ignored while moving", COMPAT, "speed:50 W Y 2 5", ""},
    {"pwm alarm clamps low", COMPAT, "W P 3 0", "pwm_alarm_percent=50"},
    {"pwm alarm clamps high", COMPAT, "W P 9 9", "pwm_alarm_percent=90"},
    {"pwm alarm", COMPAT, "W P 7 5", "pwm_alarm_percent=75"},
    {"beeper volume", COMPAT, "W B 7", "beeper_volume=7"},
    {"wm 1", COMPAT, "W M 1", "wm=1"},
    {"wm only 0 or 1", COMPAT, "W M 5", "wm=0"},
    {"turn compensation", COMPAT, "W R 3", "turn_compensation=3"},
    {"pedal trim", COMPAT, "W U 4", "pedal_trim=4"},
    {"wc", COMPAT, "W C 9", "wc=9"},
    {"roll limit step", COMPAT, "W X 2", "roll_limit_step=2"},
    {"single digit commands ignore speed", COMPAT, "speed:900 W B 3", "beeper_volume=3"},
    {"armed sub-commands stay armed", COMPAT, "W Y 2 5 C 7", "tiltback_kmh=25 wc=7"},
    {"strict sequence after completion", CLEAN, "W Y 2 5 C 7", "tiltback_kmh=25"},
    {"other bytes do not disarm", COMPAT, "W b B 5", "beeper_volume=5"},
    {"other bytes abort the sequence", CLEAN, "W b B 5", ""},
    {"WY entry holds back single-digit commands", COMPAT, "W Y B 2 5", "tiltback_kmh=25 beeper_volume=5"},
    {"one letter per sequence", CLEAN, "W Y B 2 5", ""},

    /* Service sequences: second byte after a minimum gap, nothing in between, |v| <= 9 */
    {"firmware update", COMPAT, "! tick:100 @", "firmware_update_request=0"},
    {"firmware update too early", COMPAT, "! tick:99 @", ""},
    {"early second byte cancels", COMPAT, "! tick:10 @ tick:100 @", ""},
    {"other byte cancels", COMPAT, "! tick:50 p tick:50 @", ""},
    {"calibration", COMPAT, "c tick:50 y", "calibration_request=0"},
    {"calibration first byte while moving", COMPAT, "speed:10 c tick:50 speed:0 y", ""},
    {"calibration second byte while moving", COMPAT, "c tick:50 speed:10 y", ""},
    {"calibration at the speed limit", COMPAT, "speed:-9 c tick:50 y", "calibration_request=0"},
    {"drive mode toggle", COMPAT, "+ tick:50 -", "drive_mode_toggle_request=0"},
    {"drive mode toggle too early", COMPAT, "+ tick:49 -", ""},
    {"repeated first byte restarts the gap", COMPAT, "+ tick:40 + tick:40 - tick:10 + tick:50 -",
     "drive_mode_toggle_request=0"},
    {"service sequences in clean", CLEAN, "! tick:100 @", "firmware_update_request=0"},

    /* Telemetry slot: pause, text replies */
    {"no byte, frames", COMPAT, "slot slot", "slot=frames slot=frames"},
    {"pause after any byte", COMPAT, "b slot slot slot slot", "slot=skip slot=skip slot=frames slot=frames"},
    {"no pause", CLEAN, "b slot", "slot=frames"},
    {"version replaces a group", COMPAT, "V slot slot slot slot",
     "slot=skip slot=skip slot=version slot=frames"},
    {"name replaces a group", COMPAT, "N slot slot slot", "slot=skip slot=skip slot=name"},
    {"version lost to a later byte", COMPAT, "V x slot slot slot",
     "pwm_tiltback=0 slot=skip slot=skip slot=frames"},
    {"version kept", CLEAN, "V x slot", "pwm_tiltback=0 slot=frames+version"},
    {"version and name", CLEAN, "V N slot slot", "slot=frames+version slot=frames+name"},
    {"debug dump after the second k", COMPAT, "k slot slot slot k slot slot slot",
     "slot=skip slot=skip slot=skip slot=skip slot=skip slot=debug_dump"},
    {"debug dump in clean", CLEAN, "k k slot", "slot=frames+debug_dump"},
    {"tick commands do not need the slot", COMPAT, "V tick:1 slot slot slot",
     "slot=skip slot=skip slot=version"},
};

void setUp(void)
{
}

void tearDown(void)
{
    select_profile(COMPAT);
}

static void test_script_cases(void)
{
    for (size_t index = 0U; index < (sizeof(cases) / sizeof(cases[0])); index++) {
        run_script(cases[index].profile, cases[index].script);
        TEST_ASSERT_EQUAL_STRING_MESSAGE(cases[index].expected_log, log_text, cases[index].name);
    }
}

static void test_init_cancels_every_service_sequence(void)
{
    struct euc_cmd_state state;
    euc_cmd_init(&state);
    for (size_t id = 0U; id < (size_t)EUC_CMD_SEQ_COUNT; id++) {
        TEST_ASSERT_EQUAL_INT16(-1, state.sequence_gap_ticks[id]);
    }
}

static void test_second_byte_alone_does_nothing(void)
{
    run_script(COMPAT, "tick:200 @ tick:200 y tick:200 -");
    TEST_ASSERT_EQUAL_STRING("", log_text);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_script_cases);
    RUN_TEST(test_init_cancels_every_service_sequence);
    RUN_TEST(test_second_byte_alone_does_nothing);
    return UNITY_END();
}
