/*
 * BMS link (see euc/bms/bms.h).
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "euc/bms/bms.h"
#include "euc/quirks.h"

#define REQUEST_PERIOD_TICKS 1501U /* request when the count reaches this */
#define REPLY_TIMEOUT_TICKS  500U  /* parse when the count passes this */
#define BLOCK_BYTES          44U
#define STATUS_OFFSET        38U
#define CURRENT_OFFSET       40U
#define WORD42_OFFSET        42U
#define CELLS_OFFSET         6U
#define FAILURES_TO_FAULT    3U

#define TEMPERATURE_MIN_AT_POWER_ON 5000 /* frame 01 shows it until the first reply is parsed */

/* Current scale: raw x 90 / 32.768 / 100 = 0.1 A, in double precision, truncated, as the stock code does. */
#define CURRENT_GAIN    90.0
#define CURRENT_DIVISOR 32.768
#define CURRENT_PERCENT 100.0

const uint8_t euc_bms_request[EUC_BMS_REQUEST_BYTES] = {0x1CU, 0x03U, 0x40U, 0x31U, 0xFEU};

void euc_bms_init(struct euc_bms_state *state)
{
    (void)memset(state, 0, sizeof(*state));
    state->temperature_min_c = TEMPERATURE_MIN_AT_POWER_ON;
}

void euc_bms_tick(struct euc_bms_state *state)
{
    if (state->ticks_since_request < UINT16_MAX) {
        state->ticks_since_request++;
    }
}

static uint16_t be16(const uint8_t *bytes)
{
    return (uint16_t)(((uint16_t)bytes[0] << 8U) | (uint16_t)bytes[1]);
}

static int16_t be16_signed(const uint8_t *bytes)
{
    const uint16_t raw = be16(bytes);
    return (raw > (uint16_t)INT16_MAX) ? (int16_t)((int32_t)raw - 65536) : (int16_t)raw;
}

/* Bits 13:12 not 0, bits 11:10 both set, bits 6:4 clear. */
static bool status_is_healthy(uint16_t status)
{
    const uint32_t word = status;
    return (((word >> 12U) & 3U) != 0U) && (((word >> 10U) & 3U) == 3U) && (((word >> 4U) & 7U) == 0U);
}

static void count_check(struct euc_bms_state *state, uint32_t block, bool healthy)
{
    if (healthy) {
        state->failures[block] = 0U;
        return;
    }
    if (state->failures[block] < UINT8_MAX) {
        state->failures[block]++;
    }
    if (state->failures[block] >= FAILURES_TO_FAULT) {
        state->fault = true;
    }
}

static int16_t scaled_current(int16_t raw)
{
    const double scaled = (((double)raw * CURRENT_GAIN) / CURRENT_DIVISOR) / CURRENT_PERCENT;
    return (int16_t)scaled;
}

static void temperature_range(struct euc_bms_state *state)
{
    int32_t min_c = state->temperature_c[0];
    int32_t max_c = state->temperature_c[0];
    if (EUC_QUIRK(BMS_MAX_TEMP_UNSIGNED) && (max_c < 0)) {
        /* Stock starts from the first sensor read as unsigned: below 0 degC nothing beats it as the maximum.
         */
        max_c += 65536;
    }
    for (uint32_t i = 0U; i < EUC_BMS_TEMPERATURES; i++) {
        const int32_t value = state->temperature_c[i];
        if (value > max_c) {
            max_c = value;
        }
        if (value < min_c) {
            min_c = value;
        }
    }
    state->temperature_min_c = (int16_t)min_c;
    state->temperature_max_c = (max_c > INT16_MAX) ? (int16_t)(max_c - 65536) : (int16_t)max_c;
}

void euc_bms_parse(struct euc_bms_state *state, const uint8_t reply[EUC_BMS_REPLY_BYTES])
{
    for (size_t block = 0U; block < EUC_BMS_BLOCKS; block++) {
        const uint8_t *data = &reply[block * BLOCK_BYTES];
        for (size_t i = 0U; i < EUC_BMS_TEMPS_PER_BLOCK; i++) {
            state->temperature_c[(block * EUC_BMS_TEMPS_PER_BLOCK) + i] = be16_signed(&data[2U * i]);
        }
        for (size_t i = 0U; i < EUC_BMS_CELLS_PER_BLOCK; i++) {
            state->cell_mv[(block * EUC_BMS_CELLS_PER_BLOCK) + i] = be16(&data[CELLS_OFFSET + (2U * i)]);
        }
        struct euc_bms_block *info = &state->blocks[block];
        info->status_word = be16(&data[STATUS_OFFSET]);
        info->current_raw = be16_signed(&data[CURRENT_OFFSET]);
        info->current_da = scaled_current(info->current_raw);
        info->word42_raw = be16(&data[WORD42_OFFSET]);
        count_check(state, (uint32_t)block, status_is_healthy(info->status_word));
    }
    temperature_range(state);
}

void euc_bms_poll(struct euc_bms_state *state, const struct euc_bms_in *in, struct euc_bms_out *out)
{
    (void)memset(out, 0, sizeof(*out));
    if (state->ticks_since_request >= REQUEST_PERIOD_TICKS) {
        out->send_request = true;
        state->reply_pending = true;
        state->ticks_since_request = 0U;
        return;
    }
    const bool complete = in->bytes_received >= EUC_BMS_REPLY_BYTES;
    const bool timed_out = state->ticks_since_request > REPLY_TIMEOUT_TICKS;
    if (((!complete) && (!timed_out)) || (!state->reply_pending)) {
        return;
    }
    state->reply_pending = false;
    if (complete || EUC_QUIRK(BMS_PARTIAL_REPLY)) {
        euc_bms_parse(state, in->reply);
        out->parsed = true;
        return;
    }
    /* Clean: no complete reply within 1 s counts as a failed check of every block. */
    for (uint32_t block = 0U; block < EUC_BMS_BLOCKS; block++) {
        count_check(state, block, false);
    }
}
