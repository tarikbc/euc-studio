/*
 * Begode BLE telemetry frames: encoder and group scheduler. Field map: docs/protocol/begode.md, "Frames".
 */
#include "euc/protocol/telemetry.h"

#include <stddef.h>

#define FRAME_HEADER_0 0x55U
#define FRAME_HEADER_1 0xAAU
#define FRAME_FOOTER   0x5AU

#define FIELD_BYTES_OFFSET 2U
#define TYPE_BYTE          18U
#define SUB_INDEX_BYTE     19U
#define FOOTER_OFFSET      20U
#define FOOTER_BYTES       4U

/* Frame 00 carries the voltage as 0.02 V counts and stops at 134.40 V (1344 x 0.1 V). */
#define LIVE_VOLTAGE_MAX_DV        1344U
#define LIVE_VOLTAGE_COUNTS_PER_DV 5U

/* Frame 04 bytes 16-17: constant in every stock build seen so far (meaning unknown). */
#define STATUS_TRAILER_WORD 0x0012U

#define CELLS_PER_FRAME 8U

/* Fields ----------------------------------------------------------------------------------------------- */

static void put_u16be(uint8_t frame[EUC_TLM_FRAME_BYTES], size_t field_index, uint16_t value)
{
    const size_t offset = FIELD_BYTES_OFFSET + (2U * field_index);
    frame[offset] = (uint8_t)(value >> 8U);
    frame[offset + 1U] = (uint8_t)(value & 0xFFU);
}

/* Signed fields travel as two's complement. */
static uint16_t from_signed(int16_t value)
{
    return (uint16_t)value;
}

static void encode_live(const struct euc_tlm_in *in, uint8_t frame[EUC_TLM_FRAME_BYTES])
{
    const uint16_t voltage_dv =
        (in->battery_voltage_dv > LIVE_VOLTAGE_MAX_DV) ? LIVE_VOLTAGE_MAX_DV : in->battery_voltage_dv;
    /* Settings echo: turn compensation bits 10-13, pedal trim 5-8, PWM tiltback 3, drive mode 0. */
    const uint16_t settings_echo =
        (uint16_t)(((in->turn_compensation & 0x0FU) << 10U) | ((in->pedal_trim & 0x0FU) << 5U) |
                   ((in->pwm_tiltback_enabled ? 1U : 0U) << 3U) | (in->drive_mode & 0x01U));
    put_u16be(frame, 0U, (uint16_t)(voltage_dv * LIVE_VOLTAGE_COUNTS_PER_DV));
    put_u16be(frame, 1U, from_signed(in->speed_cm_per_s));
    put_u16be(frame, 2U, in->range_estimate_km);
    put_u16be(frame, 3U, in->trip_distance_m);
    put_u16be(frame, 4U, from_signed(in->phase_current_ca));
    put_u16be(frame, 5U, from_signed(in->board_temperature_mpu6050));
    put_u16be(frame, 6U, settings_echo);
    put_u16be(frame, 7U, in->beeper_volume);
}

static void encode_bms(const struct euc_tlm_in *in, uint8_t block, uint8_t frame[EUC_TLM_FRAME_BYTES])
{
    const struct euc_tlm_bms_block *bms = &in->bms_blocks[block];
    put_u16be(frame, 0U, in->pwm_alarm_percent);
    put_u16be(frame, 1U, from_signed(in->frame01_speed_copy_dm_per_s));
    put_u16be(frame, 2U, in->battery_voltage_dv);
    put_u16be(frame, 3U, from_signed(bms->current_da));
    put_u16be(frame, 4U, from_signed(in->bms_temperature_max_c));
    put_u16be(frame, 5U, from_signed(in->bms_temperature_min_c));
    put_u16be(frame, 6U, bms->word42_raw);
    put_u16be(frame, 7U, bms->status_word);
}

static void encode_cells(const struct euc_tlm_in *in, bool pack_b, uint8_t page,
                         uint8_t frame[EUC_TLM_FRAME_BYTES])
{
    const size_t pack_offset = pack_b ? EUC_TLM_CELLS_PER_PACK : 0U;
    const size_t first_cell = pack_offset + ((size_t)page * CELLS_PER_FRAME);
    for (size_t field = 0U; field < CELLS_PER_FRAME; field++) {
        put_u16be(frame, field, in->cell_voltage_mv[first_cell + field]);
    }
}

static void encode_status(const struct euc_tlm_in *in, uint8_t frame[EUC_TLM_FRAME_BYTES])
{
    /* Modes: pedal bits 13-14, speed alarm 10-11, roll cut-off 7-8, miles 0. */
    const uint16_t modes =
        (uint16_t)(((in->pedal_mode & 0x03U) << 13U) | ((in->speed_alarm_mode & 0x03U) << 10U) |
                   ((in->roll_cutoff_mode & 0x03U) << 7U) | (in->units_miles ? 1U : 0U));
    put_u16be(frame, 0U, (uint16_t)(in->odometer_m >> 16U));
    put_u16be(frame, 1U, (uint16_t)(in->odometer_m & 0xFFFFU));
    put_u16be(frame, 2U, modes);
    put_u16be(frame, 3U, in->auto_power_off_remaining_s);
    put_u16be(frame, 4U, in->tiltback_speed_kmh);
    put_u16be(frame, 5U, in->wm_value);
    put_u16be(frame, 6U, (uint16_t)(((uint16_t)in->alert_flags << 8U) | in->light_state));
    put_u16be(frame, 7U, STATUS_TRAILER_WORD);
}

static void encode_power(const struct euc_tlm_in *in, uint8_t frame[EUC_TLM_FRAME_BYTES])
{
    const uint16_t settings_echo = (uint16_t)(((in->wc_value & 0x0FU) << 4U) | (in->wx_value & 0x0FU));
    put_u16be(frame, 0U, from_signed(in->battery_current_ca));
    put_u16be(frame, 1U, settings_echo);
    put_u16be(frame, 2U, 0U); /* motor temperature: never sent by this controller */
    put_u16be(frame, 3U, from_signed((int16_t)in->pwm_percent)); /* sign-extended to 16 bits */
    for (size_t field = 4U; field < 8U; field++) {
        put_u16be(frame, field, 0U);
    }
}

/* Encoder and scheduler -------------------------------------------------------------------------------- */

void euc_tlm_encode_frame(const struct euc_tlm_in *in, enum euc_tlm_frame_type type, uint8_t page,
                          uint8_t frame[EUC_TLM_FRAME_BYTES])
{
    const uint8_t page_in_range = (uint8_t)(page % EUC_TLM_PAGES);
    uint8_t sub_index = EUC_TLM_FIXED_SUB_INDEX;

    switch (type) {
    case EUC_TLM_FRAME_LIVE:
        encode_live(in, frame);
        break;
    case EUC_TLM_FRAME_BMS:
        encode_bms(in, page_in_range, frame);
        sub_index = page_in_range;
        break;
    case EUC_TLM_FRAME_CELLS_PACK_A:
    case EUC_TLM_FRAME_CELLS_PACK_B:
        encode_cells(in, type == EUC_TLM_FRAME_CELLS_PACK_B, page_in_range, frame);
        sub_index = page_in_range;
        break;
    case EUC_TLM_FRAME_STATUS:
        encode_status(in, frame);
        break;
    default:
        encode_power(in, frame);
        break;
    }

    frame[0] = FRAME_HEADER_0;
    frame[1] = FRAME_HEADER_1;
    frame[TYPE_BYTE] = (uint8_t)type;
    frame[SUB_INDEX_BYTE] = sub_index;
    for (size_t index = 0U; index < FOOTER_BYTES; index++) {
        frame[FOOTER_OFFSET + index] = FRAME_FOOTER;
    }
}

void euc_tlm_init(struct euc_tlm_state *state, uint8_t page, bool cell_pack_b)
{
    state->ticks_since_group = 0U;
    state->page = (uint8_t)(page % EUC_TLM_PAGES);
    state->cell_pack_b = cell_pack_b;
}

void euc_tlm_count_tick(struct euc_tlm_state *state)
{
    state->ticks_since_group++;
}

bool euc_tlm_group_due(struct euc_tlm_state *state)
{
    /* Stock test: the low byte of the tick count above 150, i.e. 151 ticks after the last group. */
    if ((state->ticks_since_group & 0xFFU) < EUC_TLM_GROUP_PERIOD_TICKS) {
        return false;
    }
    state->ticks_since_group = 0U;
    return true;
}

void euc_tlm_build_group(struct euc_tlm_state *state, const struct euc_tlm_in *in,
                         struct euc_tlm_group *group)
{
    const enum euc_tlm_frame_type cell_frame =
        state->cell_pack_b ? EUC_TLM_FRAME_CELLS_PACK_B : EUC_TLM_FRAME_CELLS_PACK_A;

    euc_tlm_encode_frame(in, EUC_TLM_FRAME_LIVE, 0U, group->frames[0]);
    euc_tlm_encode_frame(in, EUC_TLM_FRAME_BMS, state->page, group->frames[1]);
    euc_tlm_encode_frame(in, cell_frame, state->page, group->frames[2]);
    euc_tlm_encode_frame(in, EUC_TLM_FRAME_STATUS, 0U, group->frames[3]);
    euc_tlm_encode_frame(in, EUC_TLM_FRAME_POWER, 0U, group->frames[4]);

    state->page++;
    if (state->page >= EUC_TLM_PAGES) {
        state->page = 0U;
        state->cell_pack_b = !state->cell_pack_b;
    }
}
