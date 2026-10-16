/*
 * PID regulator of the current loop, integer only:
 *
 *   e        = reference - measured
 *   integral = clamp(integral + Ki e, integral_min, integral_max)     (0 while Ki = 0)
 *   output   = clamp(Kp e / kp_div + integral / ki_div + Kd (e - previous e) / kd_div, out_min, out_max)
 *
 * Each term is narrowed by C division (truncation toward zero), as in the stock drive. The caller can hold
 * the integral at zero for one step (the stop brake does that).
 */
#ifndef EUC_MOTOR_PID_H
#define EUC_MOTOR_PID_H

#include <stdbool.h>
#include <stdint.h>

struct euc_pid {
    int16_t kp;
    uint16_t kp_div; /* divisors are never 0 */
    int16_t ki;
    uint16_t ki_div;
    int16_t kd;
    uint16_t kd_div;
    int16_t out_min;
    int16_t out_max;
    int32_t integral_min;
    int32_t integral_max;
    int32_t integral;
    int32_t previous_error;
};

/* One step; returns the clamped output. */
int16_t euc_pid_step(struct euc_pid *pid, int16_t reference, int16_t measured, bool clear_integral);

#endif
