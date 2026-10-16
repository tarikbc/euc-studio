/*
 * Service sequences (see euc/service/service.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/math/float_ops.h"
#include "euc/service/service.h"

#define STAGE_WAITING 1U
#define STAGE_DONE    2U

#define FLOAT_EXPONENT_MASK 0x7F800000U

void euc_service_init(struct euc_service_state *state)
{
    (void)memset(state, 0, sizeof(*state));
}

void euc_service_request(struct euc_service_state *state, enum euc_service_request request)
{
    if ((uint32_t)request < (uint32_t)EUC_SERVICE_COUNT) {
        state->stage[request] = STAGE_WAITING;
    }
}

void euc_service_tick(struct euc_service_state *state, float balance_command, struct euc_service_out *out)
{
    (void)memset(out, 0, sizeof(*out));
    const bool command_zero = (euc_f32_bits(balance_command) & FLOAT_EXPONENT_MASK) == 0U;

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
