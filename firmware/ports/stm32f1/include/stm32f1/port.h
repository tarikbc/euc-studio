/*
 * STM32F1 port: register-level drivers for what the core needs. No ST library code: each driver is written
 * from RM0008 and follows the stock firmware's observable set-up (timer and ADC modes, baud rates,
 * priorities).
 *
 * Clock tree after stm32f1_clock_init(): HSE 8 MHz x 9 = SYSCLK 72 MHz, HCLK 72 MHz, PCLK2 72 MHz (TIM8, ADC,
 * USART1, GPIO), PCLK1 36 MHz (timers x2 = 72 MHz), ADC 12 MHz, flash 2 wait states with prefetch.
 */
#ifndef STM32F1_PORT_H
#define STM32F1_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f1/regs.h"

/* Clock, delays, reset ------------------------------------------------------------------------------- */

/* Starts the HSE and the PLL. Returns false when the HSE does not start; stock loops forever there instead
 * (quirk HSE_START_HANG), so in the compat profile this call does not return in that case. */
bool stm32f1_clock_init(void);

/* Busy waits on SysTick (HCLK / 8), as the stock delay does. */
void stm32f1_delay_ms(uint32_t ms);
void stm32f1_delay_us(uint32_t us);

/* System reset through the core (AIRCR SYSRESETREQ). */
void stm32f1_system_reset(void) __attribute__((noreturn));

/* NVIC with the stock priority grouping (2 bits pre-emption, 2 bits sub-priority). */
void stm32f1_nvic_init(void);
void stm32f1_nvic_enable(int32_t irq, uint32_t preempt, uint32_t sub);

/* GPIO -------------------------------------------------------------------------------------------------- */

enum stm32f1_pin_mode {
    STM32F1_PIN_ANALOG = 0x0,
    STM32F1_PIN_INPUT_FLOATING = 0x4,
    STM32F1_PIN_INPUT_PULL = 0x8,      /* pull-up or pull-down, chosen by the ODR bit */
    STM32F1_PIN_OUTPUT_10MHZ = 0x1,    /* push-pull */
    STM32F1_PIN_OUTPUT_50MHZ = 0x3,    /* push-pull */
    STM32F1_PIN_ALTERNATE_10MHZ = 0x9, /* alternate function push-pull */
    STM32F1_PIN_ALTERNATE_50MHZ = 0xB
};

void stm32f1_gpio_clocks_on(void);
void stm32f1_gpio_mode(struct stm32f1_gpio *port, uint32_t pin, enum stm32f1_pin_mode mode);
void stm32f1_gpio_pull(struct stm32f1_gpio *port, uint32_t pin, bool up); /* for STM32F1_PIN_INPUT_PULL */
void stm32f1_gpio_write(struct stm32f1_gpio *port, uint32_t pin, bool high);
bool stm32f1_gpio_read(const struct stm32f1_gpio *port, uint32_t pin);
/* Frees PB3, PB4 and PA15 from JTAG (SWD stays on). */
void stm32f1_gpio_release_jtag(void);

/* Motor timer and ADC ---------------------------------------------------------------------------------- */

struct stm32f1_current_sums {
    uint32_t adc1_rank1; /* phase A */
    uint32_t adc1_rank2; /* phase B on ADC1 */
    uint32_t adc2_rank1; /* phase B on ADC2 */
};

/* TIM8 centre-aligned PWM on CH1..3 + complementary outputs, outputs off (MOE 0), update interrupt on. */
void stm32f1_pwm_init(uint16_t period_counts, uint8_t dead_time_counts);
void stm32f1_pwm_set(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3);
void stm32f1_pwm_outputs(bool enabled);
bool stm32f1_pwm_counting_down(void);
void stm32f1_pwm_clear_update(void);

/* ADC1 + ADC2 in dual injected simultaneous mode, software trigger, left aligned, 71.5-cycle samples.
 * ADC1: PA4 (phase A), PA0 (battery), PC1 (NTC); ADC2: PA5 (phase B), PC0 (bus current). */
void stm32f1_adc_init(void);
/* The stock offset measurement: 16 conversions of the two phase channels with the bridge off. */
void stm32f1_adc_measure_offsets(struct stm32f1_current_sums *sums);
void stm32f1_adc_enable_interrupt(void);
void stm32f1_adc_start(void);
bool stm32f1_adc_take_end_of_conversion(void);
uint16_t stm32f1_adc1_injected(uint32_t rank); /* data register, rank 1..3 */
uint16_t stm32f1_adc2_injected(uint32_t rank); /* rank 1..2 */

/* RTC tick, BKP, flash, watchdog ------------------------------------------------------------------------ */

/* RTC from HSE / 128 with prescaler 126: second interrupt every 2.016 ms (496 Hz). */
void stm32f1_rtc_tick_init(void);
void stm32f1_rtc_tick_acknowledge(void);

void stm32f1_backup_init(void);
uint16_t stm32f1_backup_read(uint32_t index); /* DR1..DR42 */
void stm32f1_backup_write(uint32_t index, uint16_t value);

/* Flash writes name their place by a pointer into a storage page (the ld_*_page linker symbols), not by an
 * address: the host SIL port keeps the pages in host memory, where an address does not fit 32 bits. */
void stm32f1_flash_erase_page(const uint16_t *page);
void stm32f1_flash_program(const uint16_t *destination, const uint16_t *halfwords, uint32_t count);
/* The stock read-erase-rewrite of a 2 KiB page, for writing over halfwords that are not erased. */
void stm32f1_flash_rewrite(const uint16_t *destination, const uint16_t *halfwords, uint32_t count);

void stm32f1_watchdog_start(uint32_t timeout_ms);
void stm32f1_watchdog_reload(void);
/* True when the last reset came from the independent watchdog; clears the reset flags. */
bool stm32f1_reset_was_watchdog(void);

/* Serial ports ------------------------------------------------------------------------------------------- */

/* USART1 115200 8N1 (PA9 TX, PA10 RX) with receive interrupt and a transmit ring. */
void stm32f1_ble_init(void);
bool stm32f1_ble_send(const uint8_t *bytes, uint32_t count); /* false when the ring has no room */
uint32_t stm32f1_ble_free(void);                             /* free bytes in the transmit ring */
/* Interrupt body: returns true with a received byte; drains the transmit ring. */
bool stm32f1_ble_service(uint8_t *received);

/* USART3 9600 8N1 (PB10 TX, PB11 RX), reception by DMA1 channel 3. */
void stm32f1_bms_init(uint8_t *buffer, uint16_t length);
void stm32f1_bms_request(const uint8_t *request, uint32_t count, uint16_t reply_length);
uint16_t stm32f1_bms_bytes_received(uint16_t reply_length);

/* IMU, display, LED strip, buzzer ----------------------------------------------------------------------- */

/* SPI3 (PB3..5, chip select PA15) and the MPU-6500 set-up with this GYRO_CONFIG value (0x08: +-500 deg/s,
 * 0x18: +-2000 deg/s); false when WHO_AM_I is wrong. */
void stm32f1_imu_bus_init(void);
bool stm32f1_imu_init(uint8_t gyro_config);
/* accel x, y, z, temperature, gyro x, y, z (big-endian words swapped to host order) */
void stm32f1_imu_read(int16_t words[7]);

/* TM1640 on PC4 (DIN) / PC5 (CLK): 16 grid bytes at addresses 0xC0.. and display on, brightness 14/16. */
void stm32f1_display_init(void);
void stm32f1_display_send(const uint8_t grids[16]);

/* WS2812 strip on PA6: TIM3 CH1 800 kHz, DMA1 channel 2 (TIM3 CC3 request), circular. */
#define STM32F1_LED_COUNT 32U
void stm32f1_led_strip_init(void);
void stm32f1_led_strip_set(uint32_t index, uint8_t red, uint8_t green, uint8_t blue);

/* Buzzer on PB9: TIM4 CH4 at 2.70 kHz; compare 0 = silent. */
void stm32f1_buzzer_init(void);
void stm32f1_buzzer_set(uint16_t compare);

#endif
