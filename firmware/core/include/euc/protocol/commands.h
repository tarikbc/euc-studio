/*
 * Begode BLE command parser (protocol description: docs/protocol/begode.md).
 *
 * The phone writes single ASCII bytes to the BLE module, which passes them to the controller UART. This
 * module turns that byte stream into typed actions and text replies. It does not touch hardware and holds all
 * of its state in struct euc_cmd_state, so a test or a lock-step harness can drive it byte by byte and tick
 * by tick.
 *
 * Three entry points, one per stock execution context:
 *   euc_cmd_receive_byte()    every received byte (stock: USART1 RX interrupt)
 *   euc_cmd_tick()            every RTC tick, 2.016 ms (stock: RTC interrupt)
 *   euc_cmd_telemetry_slot()  every telemetry group time, 151 ticks (stock: main loop)
 *
 * Speed guards use the signed wheel speed in 0.01 m/s: settings need |v| <= 49 (below 0.5 m/s), the
 * two-byte service sequences |v| <= 9.
 *
 * Compat behaviours (config/quirks.def): CMD_LAST_BYTE_WINS, TLM_HOLDOFF, W_ARM_STICKY,
 * DEFERRED_TILTBACK_OFF, SETTINGS_WHILE_MOVING.
 */
#ifndef EUC_PROTOCOL_COMMANDS_H
#define EUC_PROTOCOL_COMMANDS_H

#include <stdbool.h>
#include <stdint.h>

/* What a command asks the rest of the firmware to do. `value` is in the unit named here. */
enum euc_cmd_action_kind {
    EUC_CMD_HORN,                     /* 'b': two beeps */
    EUC_CMD_LIGHTS,                   /* 0 off ('E'), 1 on ('Q'), 2 strobe ('T') */
    EUC_CMD_PEDAL_MODE,               /* 0 soft ('s'), 1 medium ('f'), 2 hard ('h') */
    EUC_CMD_SPEED_ALARM_MODE,         /* 0 all on ('o'), 1..3 levels 1..3 off ('u', 'i', 'I') */
    EUC_CMD_ROLL_CUTOFF_MODE,         /* 0 = 50 deg ('>'), 1 = 58 deg ('='), 2 = 66 deg ('<') */
    EUC_CMD_UNITS,                    /* 0 km ('g'), 1 miles ('m') */
    EUC_CMD_PWM_TILTBACK,             /* 1 on ('e'), 0 off ('x') */
    EUC_CMD_TILTBACK_SPEED_KMH,       /* 0..99 ('W' 'Y' d d), 200 = off ('"') */
    EUC_CMD_PWM_ALARM_PERCENT,        /* 50..90 ('W' 'P' d d) */
    EUC_CMD_BEEPER_VOLUME,            /* 0..9 ('W' 'B' d) */
    EUC_CMD_WM_VALUE,                 /* 0..1 ('W' 'M' d), meaning unknown */
    EUC_CMD_TURN_COMPENSATION,        /* 0..9 ('W' 'R' d), centripetal compensation gain */
    EUC_CMD_PEDAL_TRIM,               /* 0..9 ('W' 'U' d), pedal angle trim (inferred) */
    EUC_CMD_WC_VALUE,                 /* 0..9 ('W' 'C' d), stored and echoed, no effect in stock */
    EUC_CMD_ROLL_LIMIT_STEP,          /* 0..9 ('W' 'X' d), angle limit 40 + 5 * d deg (inferred) */
    EUC_CMD_CALIBRATION_REQUEST,      /* 'c' then 'y': motor off, IMU calibration at next power-on */
    EUC_CMD_FIRMWARE_UPDATE_REQUEST,  /* '!' then '@': motor off, hand over to the bootloader */
    EUC_CMD_DRIVE_MODE_TOGGLE_REQUEST /* '+' then '-': motor off, toggle drive mode, save, power off */
};

struct euc_cmd_action {
    enum euc_cmd_action_kind kind;
    uint8_t value;
};

/* Text replies. The firmware build maps each id to its bytes (board constants and message tables). */
enum euc_cmd_reply {
    EUC_CMD_REPLY_NONE,
    EUC_CMD_REPLY_VERSION,          /* 'V': firmware banner, e.g. "GW1825020" */
    EUC_CMD_REPLY_NAME,             /* 'N': "NAME:" + model + CR LF */
    EUC_CMD_REPLY_SPEED_ALARM_ON,   /* 'o' accepted */
    EUC_CMD_REPLY_LEVEL1_ALARM_OFF, /* 'u' accepted */
    EUC_CMD_REPLY_LEVEL2_ALARM_OFF, /* 'i' accepted */
    EUC_CMD_REPLY_LEVEL3_ALARM_OFF, /* 'I' accepted */
    EUC_CMD_REPLY_STOP_FIRST,       /* a setting refused while moving */
    EUC_CMD_REPLY_DEBUG_DUMP        /* second 'k': IMU and angle debug text */
};

/*
 * Most actions one call can produce: a digit can complete every armed 'W' sub-command at once (two-digit WY
 * and WP plus six single-digit ones) when W_ARM_STICKY leaves several armed.
 */
#define EUC_CMD_MAX_ACTIONS 8U

struct euc_cmd_out {
    uint8_t action_count;
    struct euc_cmd_action actions[EUC_CMD_MAX_ACTIONS];
    enum euc_cmd_reply reply;
};

struct euc_cmd_in {
    int16_t speed_cm_per_s; /* signed wheel speed, 0.01 m/s */
};

/* Decision for one telemetry group time. */
struct euc_cmd_slot_out {
    bool send_frames;         /* send the normal frame group */
    enum euc_cmd_reply reply; /* text to send at this group time (before the frames if both) */
};

/* 'W' sub-commands, in the order the stock firmware runs their parsers. */
enum euc_cmd_w_sub {
    EUC_CMD_W_Y,
    EUC_CMD_W_M,
    EUC_CMD_W_B,
    EUC_CMD_W_R,
    EUC_CMD_W_P,
    EUC_CMD_W_U,
    EUC_CMD_W_C,
    EUC_CMD_W_X,
    EUC_CMD_W_SUB_COUNT
};

/* Two-byte service sequences: '!' '@', 'c' 'y', '+' '-' (indexes into sequence_gap_ticks). */
enum {
    EUC_CMD_SEQ_FIRMWARE_UPDATE,
    EUC_CMD_SEQ_CALIBRATION,
    EUC_CMD_SEQ_DRIVE_MODE_TOGGLE,
    EUC_CMD_SEQ_COUNT
};

/* Commands waiting for the tick when CMD_LAST_BYTE_WINS is off (clean profile). */
#define EUC_CMD_QUEUE_BYTES 8U

struct euc_cmd_state {
    /* Pending command byte (compat: one slot, every received byte overwrites it). */
    uint8_t slot_byte;
    /* Clean: tick commands in arrival order, and text requests waiting for the next telemetry slot. */
    uint8_t queue[EUC_CMD_QUEUE_BYTES];
    uint8_t queue_head;
    uint8_t queue_count;
    bool version_requested;
    bool name_requested;
    /* 'k' bytes counted; the second one asks for the debug dump. */
    uint8_t debug_dump_count;
    /* Telemetry group times left to skip after a received byte (TLM_HOLDOFF). */
    uint8_t telemetry_holdoff_groups;
    /* Consecutive ticks with |v| <= 49, saturating; speed-alarm commands need 101. */
    uint8_t rest_ticks;
    /* 'W' parser: one "W seen" and one "letter seen" flag per sub-command, digits for WY and WP. */
    bool w_seen[EUC_CMD_W_SUB_COUNT];
    bool w_armed[EUC_CMD_W_SUB_COUNT];
    uint8_t wy_digit_count;
    uint8_t wy_tens;
    uint8_t wp_digit_count;
    uint8_t wp_tens;
    /* Service sequences: ticks left before the second byte is allowed; -1 = not started or cancelled. */
    int16_t sequence_gap_ticks[EUC_CMD_SEQ_COUNT];
};

/* Power-on state: empty slot, no hold-off, every service sequence cancelled. */
void euc_cmd_init(struct euc_cmd_state *state);

/* One received byte. Clears `out` first. */
void euc_cmd_receive_byte(struct euc_cmd_state *state, const struct euc_cmd_in *in, uint8_t byte,
                          struct euc_cmd_out *out);

/* One RTC tick (2.016 ms). Clears `out` first. */
void euc_cmd_tick(struct euc_cmd_state *state, const struct euc_cmd_in *in, struct euc_cmd_out *out);

/* One telemetry group time: frames, a text reply, both, or nothing (TLM_HOLDOFF). */
void euc_cmd_telemetry_slot(struct euc_cmd_state *state, struct euc_cmd_slot_out *out);

#endif
