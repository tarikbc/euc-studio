/*
 * PID regulator (see euc/motor/pid.h).
 *
 * With 16-bit inputs the error fits in 17 bits, so the P and I products fit in 32 bits for any 16-bit gain
 * and every division is a 32-bit one (one instruction on the Cortex-M3). The D product can exceed 32 bits
 * for a gain above 16383; it saturates there, where the stock 32-bit product would wrap. The gains of this
 * drive never reach that, so the results equal the stock ones.
 */
#include <stdbool.h>
#include <stdint.h>

#include "euc/math/fixed.h"
#include "euc/motor/pid.h"

static int64_t clamp_i64(int64_t value, int64_t min_value, int64_t max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

int16_t euc_pid_step(struct euc_pid *pid, int16_t reference, int16_t measured, bool clear_integral)
{
    const int32_t error = (int32_t)reference - (int32_t)measured;

    if (pid->ki == 0) {
        pid->integral = 0;
    } else {
        const int64_t sum = (int64_t)pid->integral + ((int64_t)pid->ki * (int64_t)error);
        pid->integral = (int32_t)clamp_i64(sum, pid->integral_min, pid->integral_max);
    }
    if (clear_integral) {
        pid->integral = 0;
    }

    const int32_t proportional = ((int32_t)pid->kp * error) / (int32_t)pid->kp_div;
    const int32_t integral = pid->integral / (int32_t)pid->ki_div;
    const int32_t change = sat_i64_to_i32((int64_t)pid->kd * ((int64_t)error - (int64_t)pid->previous_error));
    const int32_t derivative = change / (int32_t)pid->kd_div;
    pid->previous_error = error;

    const int64_t output = (int64_t)proportional + (int64_t)integral + (int64_t)derivative;
    return (int16_t)clamp_i64(output, pid->out_min, pid->out_max);
}
