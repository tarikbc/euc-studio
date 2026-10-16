/*
 * Service sequences (see euc/service/service.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/math/float_ops.h"
#include "euc/quirks.h"
#include "euc/service/service.h"

#define STAGE_NONE       0U
#define STAGE_WAITING    1U
#define STAGE_DONE       2U
#define STAGE_CONFIRMING 3U

#define FLOAT_EXPONENT_MASK 0x7F800000U

#define CONFIRM_WINDOW_TICKS 4960U /* 10 s */
#define CONFIRM_PRESS_MIN    11U   /* a short press, as for the light mode */
#define CONFIRM_PRESS_MAX    219U
#define PRESS_TICKS_LIMIT    255U
#define PROMPT_PERIOD_TICKS  1000U
#define PROMPT_GAP_TICKS     50U
#define PROMPT_BEEPS         3U
#define PROMPT_BEEP_TICKS    15U
#define ACCEPT_BEEP_TICKS    100U

void euc_service_init(struct euc_service_state *state)
{
    (void)memset(state, 0, sizeof(*state));
}

void euc_service_request(struct euc_service_state *state, enum euc_service_request request)
{
    if ((uint32_t)request >= (uint32_t)EUC_SERVICE_COUNT) {
        return;
    }
    if (EUC_QUIRK(SERVICE_WITHOUT_CONFIRM)) {
        state->stage[request] = STAGE_WAITING;
        return;
    }
    for (uint32_t i = 0U; i < (uint32_t)EUC_SERVICE_COUNT; i++) {
        if (state->stage[i] == STAGE_CONFIRMING) {
            state->stage[i] = STAGE_NONE; /* a new sequence replaces a waiting one */
        }
    }
    state->stage[request] = STAGE_CONFIRMING;
    state->confirm_ticks = 0U;
    state->press_after_request = false; /* a press already under way does not count */
}

bool euc_service_confirming(const struct euc_service_state *state)
{
    bool confirming = false;
    for (uint32_t i = 0U; i < (uint32_t)EUC_SERVICE_COUNT; i++) {
        confirming = confirming || (state->stage[i] == STAGE_CONFIRMING);
    }
    return confirming;
}

/* True on the release that ends a short press which started after the sequence. */
static bool take_short_press(struct euc_service_state *state, bool pressed)
{
    if (pressed) {
        if (state->press_ticks == 0U) {
            state->press_after_request = true;
        }
        if (state->press_ticks < PRESS_TICKS_LIMIT) {
            state->press_ticks++;
        }
        return false;
    }
    const bool short_press = state->press_after_request && (state->press_ticks >= CONFIRM_PRESS_MIN) &&
                             (state->press_ticks <= CONFIRM_PRESS_MAX);
    state->press_ticks = 0U;
    state->press_after_request = false;
    return short_press;
}

static void prompt_beeps(uint32_t ticks, struct euc_service_out *out)
{
    const uint32_t phase = ticks % PROMPT_PERIOD_TICKS;
    if (((phase % PROMPT_GAP_TICKS) == 0U) && (phase < (PROMPT_BEEPS * PROMPT_GAP_TICKS))) {
        out->beep_ticks = PROMPT_BEEP_TICKS;
    }
}

/* Clean: a waiting sequence needs no rider and a short power-button press within 10 s. */
static void confirm(struct euc_service_state *state, const struct euc_service_in *in,
                    struct euc_service_out *out)
{
    const bool short_press = take_short_press(state, in->power_button_pressed);
    for (uint32_t i = 0U; i < (uint32_t)EUC_SERVICE_COUNT; i++) {
        if (state->stage[i] != STAGE_CONFIRMING) {
            continue;
        }
        const bool expired = state->confirm_ticks >= CONFIRM_WINDOW_TICKS;
        if (in->no_rider && short_press) {
            state->stage[i] = STAGE_WAITING;
            out->reply = EUC_SERVICE_REPLY_OK;
            out->beep_ticks = ACCEPT_BEEP_TICKS;
        } else if ((!in->no_rider) || expired) {
            state->stage[i] = STAGE_NONE;
            out->reply = EUC_SERVICE_REPLY_DENIED;
        } else {
            if (state->confirm_ticks == 0U) {
                out->reply = EUC_SERVICE_REPLY_CONFIRM;
            }
            prompt_beeps(state->confirm_ticks, out);
            state->confirm_ticks++;
        }
    }
}

void euc_service_tick(struct euc_service_state *state, const struct euc_service_in *in,
                      struct euc_service_out *out)
{
    (void)memset(out, 0, sizeof(*out));
    if (!EUC_QUIRK(SERVICE_WITHOUT_CONFIRM)) {
        confirm(state, in, out);
    }
    const bool command_zero = (euc_f32_bits(in->balance_command) & FLOAT_EXPONENT_MASK) == 0U;

    if (state->stage[EUC_SERVICE_FIRMWARE_UPDATE] == STAGE_WAITING) {
        out->motor_off = true;
        /* The reset ends everything; the stage stays "waiting". */
        out->enter_bootloader = command_zero;
    }
    if (state->stage[EUC_SERVICE_CALIBRATION] == STAGE_WAITING) {
        out->motor_off = true;
        if (command_zero) {
            out->erase_calibration = true;
            state->stage[EUC_SERVICE_CALIBRATION] = STAGE_DONE;
        }
    }
    if (state->stage[EUC_SERVICE_DRIVE_MODE_TOGGLE] == STAGE_WAITING) {
        out->motor_off = true;
        if (command_zero) {
            out->toggle_drive_mode = true;
            state->stage[EUC_SERVICE_DRIVE_MODE_TOGGLE] = STAGE_DONE;
        }
    }
}
