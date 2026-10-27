/*
 * Begode BLE telemetry frames (protocol description: docs/protocol/begode.md).
 *
 * Every frame is 24 bytes: 55 AA, eight big-endian 16-bit fields (bytes 2..17), frame type (byte 18),
 * sub-index (byte 19), 5A 5A 5A 5A. No checksum. Every 151 RTC ticks (304 ms) the firmware sends one group
 * of five frames: 00, 01(n), 02 or 03 (n), 04, 07, where n counts 0..3 and the cell pack alternates each
 * time n wraps.
 *
 * struct euc_tlm_in holds the values exactly as the frames carry them (units in the names; distances and
 * speeds already converted when the rider chose miles). The encoder only packs: it applies the clamps,
 * bit fields and sign extensions that belong to the frame format, so decode -> encode gives back the same
 * bytes. Turning measurements into these values (voltage filter, NTC curve, miles factor, PWM percent) is
 * the job of the modules that own the measurements.
 */
#ifndef EUC_PROTOCOL_TELEMETRY_H
#define EUC_PROTOCOL_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#define EUC_TLM_FRAME_BYTES        24U
#define EUC_TLM_GROUP_FRAMES       5U
#define EUC_TLM_GROUP_PERIOD_TICKS 151U /* RTC ticks of 2.016 ms: 304 ms between groups */
#define EUC_TLM_PAGES              4U   /* frame 01 BMS blocks, and pages of 8 cells per pack */
#define EUC_TLM_BMS_BLOCKS         4U
#define EUC_TLM_CELLS              64U /* 2 packs x 32 cells */
#define EUC_TLM_CELLS_PER_PACK     32U

enum euc_tlm_frame_type {
    EUC_TLM_FRAME_LIVE = 0x00,         /* voltage, speed, range, trip, phase current, temperature */
    EUC_TLM_FRAME_BMS = 0x01,          /* one BMS block per frame, sub-index = block */
    EUC_TLM_FRAME_CELLS_PACK_A = 0x02, /* 8 cell voltages, sub-index = page */
    EUC_TLM_FRAME_CELLS_PACK_B = 0x03,
    EUC_TLM_FRAME_STATUS = 0x04, /* odometer, modes, power-off timer, tiltback, alerts, lights */
    EUC_TLM_FRAME_POWER = 0x07   /* battery current, settings echo, PWM */
};

/* Sub-index byte of the frame types that have no page. */
#define EUC_TLM_FIXED_SUB_INDEX 0x18U

/* Alert bits, frame 04 byte 14. */
#define EUC_TLM_ALERT_PWM_HIGH         0x01U
#define EUC_TLM_ALERT_CURRENT_SENSOR   0x02U
#define EUC_TLM_ALERT_IMU_FAULT        0x04U
#define EUC_TLM_ALERT_UNDER_VOLTAGE    0x08U
#define EUC_TLM_ALERT_OVER_VOLTAGE     0x10U
#define EUC_TLM_ALERT_OVER_TEMPERATURE 0x20U
#define EUC_TLM_ALERT_HALL_SENSOR      0x40U
#define EUC_TLM_ALERT_TRANSPORT_MODE   0x80U

struct euc_tlm_bms_block {
    int16_t current_da;   /* 0.1 A */
    uint16_t status_word; /* 0x7D01 = healthy */
    uint16_t word42_raw;  /* BMS reply bytes 42-43, meaning unknown */
};

struct euc_tlm_in {
    /* Frame 00 */
    uint16_t battery_voltage_dv;       /* 0.1 V, controller ADC; frame 00 clamps it at 134.4 V */
    int16_t speed_cm_per_s;            /* 0.01 m/s, signed */
    uint16_t range_estimate_km;        /* km (miles when units_miles) */
    uint16_t trip_distance_m;          /* m (scaled to miles when units_miles), wraps at 65535 */
    int16_t phase_current_ca;          /* 0.01 A */
    int16_t board_temperature_mpu6050; /* board NTC in MPU6050 format: degC = raw / 340 + 36.53 */
    uint8_t turn_compensation;         /* 'W' 'R' d, 0..9 */
    uint8_t pedal_trim;                /* 'W' 'U' d, 0..9 */
    bool pwm_tiltback_enabled;         /* 'e' / 'x' */
    uint8_t drive_mode;                /* 0 voltage FOC 10 kHz, 1 current FOC 12 kHz */
    uint8_t beeper_volume;             /* 'W' 'B' d, 0..9 */
    /* Frame 01 */
    uint8_t pwm_alarm_percent;           /* 'W' 'P' d d, 50..90 */
    int16_t frame01_speed_copy_dm_per_s; /* 0.1 m/s, a speed copy whose meaning is unknown */
    int16_t bms_temperature_max_c;       /* hottest of the 12 BMS sensors */
    int16_t bms_temperature_min_c;       /* coldest of the 12 BMS sensors */
    struct euc_tlm_bms_block bms_blocks[EUC_TLM_BMS_BLOCKS];
    /* Frames 02 and 03 */
    uint16_t cell_voltage_mv[EUC_TLM_CELLS]; /* pack A = 0..31, pack B = 32..63 */
    /* Frame 04 */
    uint32_t odometer_m;      /* m (scaled to miles when units_miles) */
    uint8_t pedal_mode;       /* 0 soft, 1 medium, 2 hard */
    uint8_t speed_alarm_mode; /* 0..3 */
    uint8_t roll_cutoff_mode; /* 0..2 */
    bool units_miles;
    uint16_t auto_power_off_remaining_s; /* 7200 s minus idle time */
    uint16_t tiltback_speed_kmh;         /* 200 = off */
    uint8_t wm_value;                    /* 'W' 'M' d, 0..1 */
    uint8_t alert_flags;                 /* EUC_TLM_ALERT_* */
    uint8_t light_state;                 /* 0 off, 1 on, 2 strobe */
    /* Frame 07 */
    int16_t battery_current_ca; /* 0.01 A (scale inferred) */
    uint8_t wc_value;           /* 'W' 'C' d, 0..9 */
    uint8_t wx_value;           /* 'W' 'X' d, 0..9 */
    int8_t pwm_percent;         /* signed, whole percent */
};

struct euc_tlm_group {
    uint8_t frames[EUC_TLM_GROUP_FRAMES][EUC_TLM_FRAME_BYTES];
};

struct euc_tlm_state {
    uint16_t ticks_since_group; /* stock compares only the low byte with 150 */
    uint8_t page;               /* 0..3: frame 01 block and cell page of the next group */
    bool cell_pack_b;           /* cell frame of the next group: false = 02 (pack A), true = 03 (pack B) */
};

/* Starts the group cycle at `page` and pack (`cell_pack_b`). Page 0 / pack A is the natural power-on state;
 * the stock power-on values are not confirmed yet. */
void euc_tlm_init(struct euc_tlm_state *state, uint8_t page, bool cell_pack_b);

/* One RTC tick (stock: RTC interrupt). */
void euc_tlm_count_tick(struct euc_tlm_state *state);

/* Main-loop check: true once every 151 ticks, and restarts the count. */
bool euc_tlm_group_due(struct euc_tlm_state *state);

/* Builds the next group (00, 01, 02|03, 04, 07) and advances the page and pack. */
void euc_tlm_build_group(struct euc_tlm_state *state, const struct euc_tlm_in *in,
                         struct euc_tlm_group *group);

/* Encodes one frame. `page` (0..3) selects the BMS block or cell page; other types ignore it. */
void euc_tlm_encode_frame(const struct euc_tlm_in *in, enum euc_tlm_frame_type type, uint8_t page,
                          uint8_t frame[EUC_TLM_FRAME_BYTES]);

/* The optional IMU frame (docs/protocol/begode.md, "IMU frame"): sent only by images built with
 * EUC_FEATURE_IMU_TLM, outside the stock group, one per control-tick snapshot. The encoder below is always
 * built (the unit tests use it); the stock images never call it. */
#define EUC_TLM_FRAME_IMU        0x20U
#define EUC_TLM_IMU_PERIOD_TICKS 5U   /* a snapshot every 5th RTC tick: 99.2 Hz */
#define EUC_TLM_IMU_QUIET_TICKS  496U /* no IMU frame for 1 s after a V or N reply */
#define EUC_TLM_IMU_REPLY_BYTES  32U  /* room for the longest text reply */
/* Free transmit bytes an IMU frame must leave behind: the next group and a text reply go first. */
#define EUC_TLM_IMU_RESERVE_BYTES ((EUC_TLM_GROUP_FRAMES * EUC_TLM_FRAME_BYTES) + EUC_TLM_IMU_REPLY_BYTES)
#define EUC_TLM_GYRO_500DPS       0x08U /* MPU-6500 GYRO_CONFIG: +-500 deg/s, 65.5 counts per deg/s */

struct euc_tlm_imu {
    float pitch_deg;     /* attitude filter, positive nose down */
    float roll_deg;      /* attitude filter, positive right side down */
    int16_t gyro[3];     /* calibrated counts, x y z */
    int16_t accel[3];    /* calibrated counts, x y z; 8192 per g (+-4 g) */
    uint8_t gyro_config; /* GYRO_CONFIG in use: 0x08 (+-500 deg/s) or 0x18 (+-2000 deg/s, 16.4 per deg/s) */
    uint8_t sequence;    /* snapshot number, wraps */
};

/* Encodes the IMU frame: angles in 0.01 deg, gyro in 0.1 deg/s, accel in 1 mg, each rounded and clamped to
 * s16; then no 55 AA in bytes 2-17 (adds 1 to the field that holds the 55, until none is left). */
void euc_tlm_encode_imu(const struct euc_tlm_imu *imu, uint8_t frame[EUC_TLM_FRAME_BYTES]);

/* Whether the main loop may queue an IMU frame now: the stock telemetry is not paused, no V or N reply went
 * out in the last second, and the transmit buffer keeps EUC_TLM_IMU_RESERVE_BYTES after the frame. */
bool euc_tlm_imu_may_send(uint32_t tx_free_bytes, bool telemetry_paused, uint32_t ticks_since_text);

#endif
