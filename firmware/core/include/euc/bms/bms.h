/*
 * BMS link on USART3 (9600 8N1): request, reply, health check.
 *
 * Every 1501 RTC ticks (3.03 s) the controller sends the 5-byte request and receives a 176-byte reply by DMA:
 * 4 blocks of 44 bytes, big-endian:
 *
 *   offset  0-5    3 temperatures, s16, degC
 *           6-37   16 cell voltages, u16, mV
 *           38-39  status word (0x7D01 = healthy)
 *           40-41  current, s16 raw; x 90 / 32.768 / 100 = 0.1 A
 *           42-43  unknown word
 *
 * The reply is parsed once, when all 176 bytes are in or 500 ticks after the request. Stock parses whatever
 * the buffer holds at that time, so a short or missing reply repeats old data (quirk BMS_PARTIAL_REPLY);
 * clean parses only complete replies and counts a missing one as a failed check of every block.
 *
 * Until the first reply is parsed, the lowest temperature reads 5000 and the highest 0 (stock power-on
 * values, telemetry frame 01).
 *
 * Health: a block fails when status bits 13:12 are 0, bits 11:10 are not both set, or bits 6:4 are not 0.
 * Three failures in a row on any block latch the BMS fault (alarm beep). Nothing in the drive reads the BMS.
 */
#ifndef EUC_BMS_BMS_H
#define EUC_BMS_BMS_H

#include <stdbool.h>
#include <stdint.h>

#define EUC_BMS_REQUEST_BYTES   5U
#define EUC_BMS_REPLY_BYTES     176U
#define EUC_BMS_BLOCKS          4U
#define EUC_BMS_CELLS_PER_BLOCK 16U
#define EUC_BMS_TEMPS_PER_BLOCK 3U
#define EUC_BMS_CELLS           (EUC_BMS_BLOCKS * EUC_BMS_CELLS_PER_BLOCK)
#define EUC_BMS_TEMPERATURES    (EUC_BMS_BLOCKS * EUC_BMS_TEMPS_PER_BLOCK)

/* The request the BMS answers. */
extern const uint8_t euc_bms_request[EUC_BMS_REQUEST_BYTES];

struct euc_bms_block {
    uint16_t status_word;
    int16_t current_raw;
    int16_t current_da; /* 0.1 A */
    uint16_t word42_raw;
};

struct euc_bms_state {
    uint16_t ticks_since_request;
    bool reply_pending;
    uint8_t failures[EUC_BMS_BLOCKS];
    bool fault; /* latched */
    struct euc_bms_block blocks[EUC_BMS_BLOCKS];
    int16_t temperature_c[EUC_BMS_TEMPERATURES];
    uint16_t cell_mv[EUC_BMS_CELLS]; /* block k: cells 16k..16k+15 (pack A = blocks 0, 1) */
    int16_t temperature_min_c;
    int16_t temperature_max_c;
};

struct euc_bms_in {
    uint16_t bytes_received; /* reply bytes in the buffer since the last request */
    const uint8_t *reply;    /* the receive buffer, EUC_BMS_REPLY_BYTES long */
};

struct euc_bms_out {
    bool send_request; /* restart reception of 176 bytes, then send euc_bms_request */
    bool parsed;       /* new values this call */
};

void euc_bms_init(struct euc_bms_state *state);

/* Every RTC tick. */
void euc_bms_tick(struct euc_bms_state *state);

/* Every main-loop pass. Clears `out` first. */
void euc_bms_poll(struct euc_bms_state *state, const struct euc_bms_in *in, struct euc_bms_out *out);

/* Decodes one reply into the state and runs the health check (part of euc_bms_poll, public for tests). */
void euc_bms_parse(struct euc_bms_state *state, const uint8_t reply[EUC_BMS_REPLY_BYTES]);

#endif
