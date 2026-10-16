/*
 * Begode BLE command parser. Behaviour and the reasons for each rule: docs/protocol/begode.md, "Commands".
 */
#include "euc/protocol/commands.h"

#include <stddef.h>

#include "euc/quirks.h"

/* The stock firmware marks an empty command slot with ','. */
#define SLOT_EMPTY ((uint8_t)',')

#define SETTINGS_SPEED_LIMIT_CM_PER_S 49
#define SERVICE_SPEED_LIMIT_CM_PER_S  9

/* Speed-alarm mode commands wait until the wheel has stood still this long (about 0.2 s). */
#define ALARM_COMMAND_REST_TICKS 101U

/* A received byte skips two telemetry group times; the third one sends. */
#define TELEMETRY_HOLDOFF_GROUPS 3U

#define TILTBACK_OFF_KMH      200U
#define PWM_ALARM_MIN_PERCENT 50U
#define PWM_ALARM_MAX_PERCENT 90U
#define DEBUG_DUMP_K_COUNT    2U
#define WM_MAX_VALUE          1U

struct sequence_rule {
    uint8_t first_byte;
    uint8_t second_byte;
    int16_t min_gap_ticks; /* the second byte counts only after this many ticks with no byte between */
    enum euc_cmd_action_kind request;
};

/* Result of offering a pending byte to the tick-time commands. */
enum tick_result {
    TICK_NOT_A_TICK_COMMAND, /* the byte belongs to someone else (text request, digit, ...) */
    TICK_DONE,               /* acted on or refused: remove it */
    TICK_KEEP                /* waiting for its guard: leave it pending */
};

/* Helpers ---------------------------------------------------------------------------------------------- */

static void out_clear(struct euc_cmd_out *out)
{
    out->action_count = 0U;
    out->reply = EUC_CMD_REPLY_NONE;
}

static void out_add_action(struct euc_cmd_out *out, enum euc_cmd_action_kind kind, uint8_t value)
{
    if (out->action_count < EUC_CMD_MAX_ACTIONS) {
        out->actions[out->action_count].kind = kind;
        out->actions[out->action_count].value = value;
        out->action_count++;
    }
}

static bool speed_at_most(const struct euc_cmd_in *in, int32_t limit_cm_per_s)
{
    const int32_t speed_cm_per_s = in->speed_cm_per_s;
    return (speed_cm_per_s <= limit_cm_per_s) && (speed_cm_per_s >= -limit_cm_per_s);
}

static bool is_digit(uint8_t byte)
{
    return (byte >= (uint8_t)'0') && (byte <= (uint8_t)'9');
}

/* 'W' sub-commands ------------------------------------------------------------------------------------- */

static void w_reset_all(struct euc_cmd_state *state)
{
    for (size_t sub = 0U; sub < (size_t)EUC_CMD_W_SUB_COUNT; sub++) {
        state->w_seen[sub] = false;
        state->w_armed[sub] = false;
    }
    state->wy_digit_count = 0U;
    state->wp_digit_count = 0U;
}

static void w_finish(struct euc_cmd_state *state, enum euc_cmd_w_sub sub)
{
    state->w_seen[sub] = false;
    state->w_armed[sub] = false;
    if (!EUC_QUIRK(W_ARM_STICKY)) {
        w_reset_all(state);
    }
}

static void w_arm(struct euc_cmd_state *state, enum euc_cmd_w_sub sub)
{
    if (!EUC_QUIRK(W_ARM_STICKY)) {
        /* Strict: 'W', one letter, then its digits. Nothing else stays armed. */
        w_reset_all(state);
        state->w_seen[sub] = true;
    }
    state->w_armed[sub] = true;
    /* Every sub-command letter restarts the WY digit count, which also gates the single-digit commands. */
    state->wy_digit_count = 0U;
    if (sub == EUC_CMD_W_P) {
        state->wp_digit_count = 0U;
    }
}

/* Two-digit sub-commands (WY, WP): returns the value once the second digit arrives. */
static bool w_two_digits(uint8_t *digit_count, uint8_t *tens, uint8_t digit, uint8_t *value)
{
    if (*digit_count == 0U) {
        *tens = digit;
        *digit_count = 1U;
        return false;
    }
    *digit_count = 0U;
    *value = (uint8_t)((*tens * 10U) + digit);
    return true;
}

static uint8_t clamp_u8(uint8_t value, uint8_t min_value, uint8_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    return (value > max_value) ? max_value : value;
}

static enum euc_cmd_action_kind w_single_digit_action(enum euc_cmd_w_sub sub)
{
    switch (sub) {
    case EUC_CMD_W_M:
        return EUC_CMD_WM_VALUE;
    case EUC_CMD_W_B:
        return EUC_CMD_BEEPER_VOLUME;
    case EUC_CMD_W_R:
        return EUC_CMD_TURN_COMPENSATION;
    case EUC_CMD_W_U:
        return EUC_CMD_PEDAL_TRIM;
    case EUC_CMD_W_C:
        return EUC_CMD_WC_VALUE;
    default:
        return EUC_CMD_ROLL_LIMIT_STEP;
    }
}

/*
 * A digit for an armed sub-command. Returns true when the parser used it.
 *
 * Ride settings over BLE (SETTINGS_WHILE_MOVING): the stock firmware applies the single-digit settings at any
 * speed, and the BLE link has no pairing, so anyone in range can change them mid-ride. Clean applies them
 * only below the settings speed limit and answers "stop first" otherwise. The beeper volume ('W' 'B') stays
 * allowed.
 */
static bool w_digit(struct euc_cmd_state *state, enum euc_cmd_w_sub sub, uint8_t digit,
                    bool settings_speed_ok, struct euc_cmd_out *out)
{
    uint8_t value = 0U;
    if (sub == EUC_CMD_W_Y) {
        if (w_two_digits(&state->wy_digit_count, &state->wy_tens, digit, &value)) {
            /* No range check: 00..99 are all accepted, as in the stock firmware. */
            out_add_action(out, EUC_CMD_TILTBACK_SPEED_KMH, value);
            w_finish(state, sub);
        }
        return true;
    }
    if (sub == EUC_CMD_W_P) {
        if (w_two_digits(&state->wp_digit_count, &state->wp_tens, digit, &value)) {
            out_add_action(out, EUC_CMD_PWM_ALARM_PERCENT,
                           clamp_u8(value, PWM_ALARM_MIN_PERCENT, PWM_ALARM_MAX_PERCENT));
            w_finish(state, sub);
        }
        return true;
    }
    if (state->wy_digit_count != 0U) {
        return false; /* a WY entry in progress takes the digits */
    }
    if ((sub != EUC_CMD_W_B) && !settings_speed_ok && !EUC_QUIRK(SETTINGS_WHILE_MOVING)) {
        out->reply = EUC_CMD_REPLY_STOP_FIRST;
        w_finish(state, sub);
        return true;
    }
    value = digit;
    if ((sub == EUC_CMD_W_M) && (value > WM_MAX_VALUE)) {
        value = 0U; /* WM accepts only 0 and 1; any other digit stores 0 */
    }
    out_add_action(out, w_single_digit_action(sub), value);
    w_finish(state, sub);
    return true;
}

/* Runs every sub-command parser on one byte, in stock order. Returns true if any of them used the byte. */
static bool w_parse(struct euc_cmd_state *state, const struct euc_cmd_in *in, uint8_t byte,
                    struct euc_cmd_out *out)
{
    /* Letter that arms each sub-command, in enum euc_cmd_w_sub order. */
    static const uint8_t w_letters[EUC_CMD_W_SUB_COUNT] = {'Y', 'M', 'B', 'R', 'P', 'U', 'C', 'X'};
    const bool settings_speed_ok = speed_at_most(in, SETTINGS_SPEED_LIMIT_CM_PER_S);
    bool used = false;
    for (size_t index = 0U; index < (size_t)EUC_CMD_W_SUB_COUNT; index++) {
        const enum euc_cmd_w_sub sub = (enum euc_cmd_w_sub)index;
        const bool speed_guarded = (sub == EUC_CMD_W_Y) || (sub == EUC_CMD_W_P);
        if (speed_guarded && !settings_speed_ok) {
            continue; /* WY and WP ignore every byte while the wheel moves */
        }
        if (byte == (uint8_t)'W') {
            state->w_seen[sub] = true;
            state->w_armed[sub] = false;
            used = true;
        } else if ((byte == w_letters[sub]) && state->w_seen[sub]) {
            w_arm(state, sub);
            used = true;
        } else if (is_digit(byte) && state->w_armed[sub]) {
            used = w_digit(state, sub, (uint8_t)(byte - (uint8_t)'0'), settings_speed_ok, out) || used;
        } else {
            /* not for this sub-command */
        }
    }
    return used;
}

/* Service sequences ------------------------------------------------------------------------------------ */

static void sequences_parse(struct euc_cmd_state *state, const struct euc_cmd_in *in, uint8_t byte,
                            struct euc_cmd_out *out)
{
    static const struct sequence_rule sequence_rules[EUC_CMD_SEQ_COUNT] = {
        [EUC_CMD_SEQ_FIRMWARE_UPDATE] = {'!', '@', 100, EUC_CMD_FIRMWARE_UPDATE_REQUEST},
        [EUC_CMD_SEQ_CALIBRATION] = {'c', 'y', 50, EUC_CMD_CALIBRATION_REQUEST},
        [EUC_CMD_SEQ_DRIVE_MODE_TOGGLE] = {'+', '-', 50, EUC_CMD_DRIVE_MODE_TOGGLE_REQUEST},
    };
    const bool service_speed_ok = speed_at_most(in, SERVICE_SPEED_LIMIT_CM_PER_S);
    for (size_t id = 0U; id < (size_t)EUC_CMD_SEQ_COUNT; id++) {
        const struct sequence_rule *rule = &sequence_rules[id];
        int16_t *gap_ticks = &state->sequence_gap_ticks[id];
        if (byte == rule->first_byte) {
            /* Starts (or restarts) the minimum gap; moving cancels instead. */
            *gap_ticks = service_speed_ok ? rule->min_gap_ticks : -1;
        } else if ((byte == rule->second_byte) && (*gap_ticks == 0) && service_speed_ok) {
            out_add_action(out, rule->request, 0U);
            *gap_ticks = -1;
        } else {
            *gap_ticks = -1; /* any other byte, or the second byte too early, cancels */
        }
    }
}

/* Single-byte actions applied at once ------------------------------------------------------------------ */

static void direct_action(const struct euc_cmd_in *in, uint8_t byte, struct euc_cmd_out *out)
{
    const bool ride_setting = (byte == (uint8_t)'>') || (byte == (uint8_t)'=') || (byte == (uint8_t)'<') ||
                              (byte == (uint8_t)'e') || (byte == (uint8_t)'x');
    if (ride_setting && !EUC_QUIRK(SETTINGS_WHILE_MOVING) &&
        !speed_at_most(in, SETTINGS_SPEED_LIMIT_CM_PER_S)) {
        out->reply = EUC_CMD_REPLY_STOP_FIRST; /* clean: roll cut-off and PWM tiltback only at standstill */
        return;
    }
    switch (byte) {
    case (uint8_t)'E':
        out_add_action(out, EUC_CMD_LIGHTS, 0U);
        break;
    case (uint8_t)'Q':
        out_add_action(out, EUC_CMD_LIGHTS, 1U);
        break;
    case (uint8_t)'T':
        out_add_action(out, EUC_CMD_LIGHTS, 2U);
        break;
    case (uint8_t)'>':
        out_add_action(out, EUC_CMD_ROLL_CUTOFF_MODE, 0U);
        break;
    case (uint8_t)'=':
        out_add_action(out, EUC_CMD_ROLL_CUTOFF_MODE, 1U);
        break;
    case (uint8_t)'<':
        out_add_action(out, EUC_CMD_ROLL_CUTOFF_MODE, 2U);
        break;
    case (uint8_t)'e':
        out_add_action(out, EUC_CMD_PWM_TILTBACK, 1U);
        break;
    case (uint8_t)'x':
        out_add_action(out, EUC_CMD_PWM_TILTBACK, 0U);
        break;
    case (uint8_t)'g':
        out_add_action(out, EUC_CMD_UNITS, 0U);
        break;
    case (uint8_t)'m':
        out_add_action(out, EUC_CMD_UNITS, 1U);
        break;
    default:
        break;
    }
}

/* Pending commands ------------------------------------------------------------------------------------- */

static bool is_tick_command(uint8_t byte)
{
    switch (byte) {
    case (uint8_t)'b':
    case (uint8_t)'"':
    case (uint8_t)'h':
    case (uint8_t)'f':
    case (uint8_t)'s':
    case (uint8_t)'o':
    case (uint8_t)'u':
    case (uint8_t)'i':
    case (uint8_t)'I':
        return true;
    default:
        return false;
    }
}

/* Clean profile: tick commands queue up and text requests are flags, so no byte erases another. */
static void store_without_slot(struct euc_cmd_state *state, uint8_t byte)
{
    if (byte == (uint8_t)'V') {
        state->version_requested = true;
    } else if (byte == (uint8_t)'N') {
        state->name_requested = true;
    } else if (byte == (uint8_t)'k') {
        if (state->debug_dump_count < DEBUG_DUMP_K_COUNT) {
            state->debug_dump_count++;
        }
    } else if (is_tick_command(byte) && (state->queue_count < EUC_CMD_QUEUE_BYTES)) {
        const uint8_t tail = (uint8_t)((state->queue_head + state->queue_count) % EUC_CMD_QUEUE_BYTES);
        state->queue[tail] = byte;
        state->queue_count++;
    } else {
        /* not a pending command, or the queue is full: dropped */
    }
}

static enum tick_result speed_alarm_command(const struct euc_cmd_state *state, const struct euc_cmd_in *in,
                                            uint8_t byte, bool can_wait, struct euc_cmd_out *out)
{
    static const enum euc_cmd_reply accepted_reply[4] = {
        EUC_CMD_REPLY_SPEED_ALARM_ON, EUC_CMD_REPLY_LEVEL1_ALARM_OFF, EUC_CMD_REPLY_LEVEL2_ALARM_OFF,
        EUC_CMD_REPLY_LEVEL3_ALARM_OFF};
    uint8_t mode = 3U;
    if (byte == (uint8_t)'o') {
        mode = 0U;
    } else if (byte == (uint8_t)'u') {
        mode = 1U;
    } else if (byte == (uint8_t)'i') {
        mode = 2U;
    } else {
        /* 'I': level 3 off */
    }
    const bool moving = !speed_at_most(in, SETTINGS_SPEED_LIMIT_CM_PER_S);
    if (!moving && (state->rest_ticks >= ALARM_COMMAND_REST_TICKS)) {
        out_add_action(out, EUC_CMD_SPEED_ALARM_MODE, mode);
        out->reply = accepted_reply[mode];
        return TICK_DONE;
    }
    if (!moving && can_wait) {
        return TICK_KEEP; /* stopped, but not for long enough yet */
    }
    if (mode != 3U) {
        out->reply = EUC_CMD_REPLY_STOP_FIRST; /* 'I' is refused without a reply */
    }
    return TICK_DONE;
}

/* Offers one pending byte to the commands that run every tick. */
static enum tick_result tick_command(const struct euc_cmd_state *state, const struct euc_cmd_in *in,
                                     uint8_t byte, bool can_wait, struct euc_cmd_out *out)
{
    const bool settings_speed_ok = speed_at_most(in, SETTINGS_SPEED_LIMIT_CM_PER_S);
    switch (byte) {
    case (uint8_t)'b':
        out_add_action(out, EUC_CMD_HORN, 0U);
        return TICK_DONE;
    case (uint8_t)'"':
        if (settings_speed_ok) {
            out_add_action(out, EUC_CMD_TILTBACK_SPEED_KMH, TILTBACK_OFF_KMH);
            return TICK_DONE;
        }
        return (can_wait && EUC_QUIRK(DEFERRED_TILTBACK_OFF)) ? TICK_KEEP : TICK_DONE;
    case (uint8_t)'h':
    case (uint8_t)'f':
    case (uint8_t)'s':
        if (settings_speed_ok) {
            const uint8_t pedal_mode = (byte == (uint8_t)'h') ? 2U : ((byte == (uint8_t)'f') ? 1U : 0U);
            out_add_action(out, EUC_CMD_PEDAL_MODE, pedal_mode);
        }
        return TICK_DONE; /* refused silently while moving */
    case (uint8_t)'o':
    case (uint8_t)'u':
    case (uint8_t)'i':
    case (uint8_t)'I':
        return speed_alarm_command(state, in, byte, can_wait, out);
    default:
        return TICK_NOT_A_TICK_COMMAND;
    }
}

/* Entry points ----------------------------------------------------------------------------------------- */

void euc_cmd_init(struct euc_cmd_state *state)
{
    *state = (struct euc_cmd_state){0};
    state->slot_byte = SLOT_EMPTY;
    for (size_t id = 0U; id < (size_t)EUC_CMD_SEQ_COUNT; id++) {
        state->sequence_gap_ticks[id] = -1;
    }
}

void euc_cmd_receive_byte(struct euc_cmd_state *state, const struct euc_cmd_in *in, uint8_t byte,
                          struct euc_cmd_out *out)
{
    out_clear(out);

    if (EUC_QUIRK(CMD_LAST_BYTE_WINS)) {
        state->slot_byte = byte;
    } else {
        store_without_slot(state, byte);
    }

    const bool w_used = w_parse(state, in, byte, out);
    if (!w_used && !EUC_QUIRK(W_ARM_STICKY)) {
        w_reset_all(state); /* clean: a byte that does not continue a 'W' sequence ends it */
    }

    sequences_parse(state, in, byte, out);

    if (EUC_QUIRK(TLM_HOLDOFF)) {
        state->telemetry_holdoff_groups = TELEMETRY_HOLDOFF_GROUPS;
    }

    direct_action(in, byte, out);
}

void euc_cmd_tick(struct euc_cmd_state *state, const struct euc_cmd_in *in, struct euc_cmd_out *out)
{
    out_clear(out);

    if (!speed_at_most(in, SETTINGS_SPEED_LIMIT_CM_PER_S)) {
        state->rest_ticks = 0U;
    } else if (state->rest_ticks < UINT8_MAX) {
        state->rest_ticks++;
    } else {
        /* saturated */
    }

    for (size_t id = 0U; id < (size_t)EUC_CMD_SEQ_COUNT; id++) {
        if (state->sequence_gap_ticks[id] > 0) {
            state->sequence_gap_ticks[id]--;
        }
    }

    if (EUC_QUIRK(CMD_LAST_BYTE_WINS)) {
        if (tick_command(state, in, state->slot_byte, true, out) == TICK_DONE) {
            state->slot_byte = SLOT_EMPTY;
        }
    } else if (state->queue_count > 0U) {
        /* One queued command per tick; nothing waits at the head, so the queue cannot stall. */
        (void)tick_command(state, in, state->queue[state->queue_head], false, out);
        state->queue_head = (uint8_t)((state->queue_head + 1U) % EUC_CMD_QUEUE_BYTES);
        state->queue_count--;
    } else {
        /* nothing pending */
    }
}

void euc_cmd_telemetry_slot(struct euc_cmd_state *state, struct euc_cmd_slot_out *out)
{
    out->send_frames = false;
    out->reply = EUC_CMD_REPLY_NONE;

    if (state->telemetry_holdoff_groups > 0U) {
        state->telemetry_holdoff_groups--;
        if (state->telemetry_holdoff_groups > 0U) {
            return; /* paused after a received byte */
        }
    }

    bool text_request_served = false;
    if (EUC_QUIRK(CMD_LAST_BYTE_WINS)) {
        if (state->slot_byte == (uint8_t)'V') {
            out->reply = EUC_CMD_REPLY_VERSION;
        } else if (state->slot_byte == (uint8_t)'N') {
            out->reply = EUC_CMD_REPLY_NAME;
        } else if (state->slot_byte == (uint8_t)'k') {
            state->debug_dump_count++;
        } else {
            /* not a text request */
        }
        text_request_served = (state->slot_byte == (uint8_t)'V') || (state->slot_byte == (uint8_t)'N') ||
                              (state->slot_byte == (uint8_t)'k');
        if (text_request_served) {
            state->slot_byte = SLOT_EMPTY;
        }
    } else if (state->version_requested) {
        out->reply = EUC_CMD_REPLY_VERSION;
        state->version_requested = false;
    } else if (state->name_requested) {
        out->reply = EUC_CMD_REPLY_NAME;
        state->name_requested = false;
    } else {
        /* no text request */
    }

    if ((out->reply == EUC_CMD_REPLY_NONE) && (state->debug_dump_count >= DEBUG_DUMP_K_COUNT)) {
        out->reply = EUC_CMD_REPLY_DEBUG_DUMP;
        state->debug_dump_count = 0U;
    }
    text_request_served = text_request_served || (out->reply != EUC_CMD_REPLY_NONE);

    /* Stock: a text reply (or a consumed 'k') takes the place of the frame group. */
    out->send_frames = !(text_request_served && EUC_QUIRK(TLM_HOLDOFF));
}
