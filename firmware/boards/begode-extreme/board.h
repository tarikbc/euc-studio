/*
 * Begode Extreme main board: STM32F103RC-class MCU (ST or a GD32/APM32/AT32 clone, vendor not confirmed).
 *
 * Board constants only: facts about this PCB, with units in the names. No code, no tuning (stock tuning goes
 * in tuning_stock.c). The simulator's view of the same board is profiles/begode-extreme/board.toml.
 *
 * The memory map lives in link_app.ld, the one place it is written down. Code that needs an address of the
 * image takes it from the linker symbols (ld_*).
 */
#ifndef BOARD_BEGODE_EXTREME_H
#define BOARD_BEGODE_EXTREME_H

/* Identity --------------------------------------------------------------------------------------------- */

/* Board id in the .euc_info block and directory name under boards/ and profiles/ (at most 15 characters). */
#define BOARD_ID "begode-extreme"

/* Clocks ----------------------------------------------------------------------------------------------- */
/* HSE crystal and PLL settings: added with the clock driver (F5). */

/* Pins ------------------------------------------------------------------------------------------------- */
/* Fixed peripheral pins (TIM8, ADC, USART1/3, SPI3, TM1640, WS2812, buzzer) are set by their port drivers. */

/* GPIOB: power latch (high = stay on), power / light button (low = pressed; input with pull-down), light. */
#define BOARD_PIN_LATCH        14U
#define BOARD_PIN_POWER_BUTTON 15U
#define BOARD_PIN_LIGHT        12U
/* GPIOB: halls; the hall state is A | B << 1 | C << 2. */
#define BOARD_PIN_HALL_A 7U
#define BOARD_PIN_HALL_B 6U
#define BOARD_PIN_HALL_C 8U
/* GPIOA: handle button (low = pressed; floating input). */
#define BOARD_PIN_HANDLE 12U

/* Identity replies --------------------------------------------------------------------------------------- */
/* 'V' and 'N' answers. Apps identify the wheel by them, so they stay the stock strings. */
#define BOARD_FIRMWARE_BANNER "GW1825020"
#define BOARD_MODEL_REPLY     "NAME:EXTREME\r\n"

/* Analog scaling --------------------------------------------------------------------------------------- */
/* Battery divider, current-sense gain, NTC: added with the ADC path. The core works on the stock current
 * scale (Q1.15 32768 = 437.5 A); the true sensor gain is not known yet. */

/* Motor drive ------------------------------------------------------------------------------------------ */

/* TIM8 compare channel (1..3) that drives each motor phase. Phases B and C are crossed on this board: the
 * core's phase B duty goes to CCR3 and phase C to CCR2. */
#define BOARD_MOTOR_PHASE_A_TIM8_CHANNEL 1U
#define BOARD_MOTOR_PHASE_B_TIM8_CHANNEL 3U
#define BOARD_MOTOR_PHASE_C_TIM8_CHANNEL 2U

/*
 * Phase over-current trip (core/motor/current.h), OFF in both profiles until the level is measured on the
 * bench (quirk NO_OVERCURRENT_TRIP is on in clean too; docs/watchdog-and-overcurrent.md). PLACEHOLDER: 31500
 * counts is about 420 A on the stock scale, the level the stock code only counts. Set it from the measured
 * limits of the power stage (MOSFETs, current sensors) before the trip is switched on.
 */
#define BOARD_PHASE_OVERCURRENT_TRIP_Q15     31500
#define BOARD_PHASE_OVERCURRENT_TRIP_SAMPLES 2U /* consecutive FOC readings: 167 us at 12 kHz */

#endif
