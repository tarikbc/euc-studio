/*
 * IMU on SPI3 (RM0008 25, MPU-6500 register map), TM1640 display (bit-banged), WS2812 LED strip (TIM3 + DMA)
 * and the buzzer (TIM4).
 */
#include <stdbool.h>
#include <stdint.h>

#include "stm32f1/device.h"
#include "stm32f1/port.h"
#include "stm32f1/regs.h"

/* IMU --------------------------------------------------------------------------------------------------- */

#define IMU_CS_PIN     15U /* PA15 */
#define SPI_WAIT_LIMIT 200U
#define IMU_READ       0x80U
#define IMU_WHO_AM_I   0x75U
#define IMU_ID_MPU6500 0x70U
#define IMU_DATA_FIRST 0x3BU /* ACCEL_XOUT_H */
#define IMU_DATA_BYTES 14U

struct imu_write {
    uint8_t reg;
    uint8_t value;
    uint8_t wait_ms;
};

static uint8_t spi_transfer(uint8_t out)
{
    uint32_t waits = 0U;
    while ((SPI3->SR & SPI_SR_TXE) == 0U) {
        waits++;
        if (waits > SPI_WAIT_LIMIT) {
            return 0U;
        }
    }
    SPI3->DR = out;
    waits = 0U;
    while ((SPI3->SR & SPI_SR_RXNE) == 0U) {
        waits++;
        if (waits > SPI_WAIT_LIMIT) {
            return 0U;
        }
    }
    return (uint8_t)(SPI3->DR & 0xFFU);
}

static void imu_select(bool selected)
{
    stm32f1_gpio_write(GPIOA, IMU_CS_PIN, !selected);
}

static void imu_write(uint8_t reg, uint8_t value)
{
    imu_select(true);
    (void)spi_transfer(reg);
    (void)spi_transfer(value);
    imu_select(false);
}

static uint8_t imu_read_register(uint8_t reg)
{
    imu_select(true);
    (void)spi_transfer((uint8_t)(reg | IMU_READ));
    const uint8_t value = spi_transfer(0xFFU);
    imu_select(false);
    return value;
}

void stm32f1_imu_bus_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_SPI3;
    stm32f1_gpio_mode(GPIOB, 3U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOB, 4U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOB, 5U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_release_jtag();
    SPI3->CR1 = SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_MSTR | SPI_CR1_BR_DIV64 | SPI_CR1_SSI | SPI_CR1_SSM;
    SPI3->CR1 |= SPI_CR1_SPE;
    (void)spi_transfer(0xFFU);
    stm32f1_gpio_mode(GPIOA, IMU_CS_PIN, STM32F1_PIN_OUTPUT_50MHZ);
    imu_select(false);
}

bool stm32f1_imu_init(void)
{
    /* MPU-6500 start-up, as the stock firmware writes it: register, value, wait in ms. */
    static const struct imu_write imu_setup[] = {
        {0x6BU, 0x80U, 200U}, /* PWR_MGMT_1: reset */
        {0x68U, 0x07U, 100U}, /* SIGNAL_PATH_RESET */
        {0x6BU, 0x01U, 20U},  /* PWR_MGMT_1: PLL clock */
        {0x6AU, 0x10U, 20U},  /* USER_CTRL: I2C interface off */
        {0x19U, 0x01U, 20U},  /* SMPLRT_DIV */
        {0x1AU, 0x20U, 20U},  /* CONFIG */
        {0x1BU, 0x08U, 20U},  /* GYRO_CONFIG: +-500 deg/s */
        {0x1CU, 0x08U, 20U},  /* ACCEL_CONFIG: +-4 g */
        {0x1DU, 0x00U, 20U},  /* ACCEL_CONFIG2 */
    };
    for (uint32_t i = 0U; i < (sizeof(imu_setup) / sizeof(imu_setup[0])); i++) {
        imu_write(imu_setup[i].reg, imu_setup[i].value);
        stm32f1_delay_ms(imu_setup[i].wait_ms);
    }
    while ((SPI3->SR & SPI_SR_TXE) == 0U) {
        /* idle before the speed change */
    }
    SPI3->CR1 &= ~(SPI_CR1_BR_MASK | SPI_CR1_SPE); /* 18 MHz for the data reads */
    SPI3->CR1 |= SPI_CR1_SPE;
    stm32f1_delay_ms(100U);
    return imu_read_register(IMU_WHO_AM_I) == IMU_ID_MPU6500;
}

void stm32f1_imu_read(int16_t words[7])
{
    uint8_t bytes[IMU_DATA_BYTES];
    imu_select(true);
    (void)spi_transfer((uint8_t)(IMU_DATA_FIRST | IMU_READ));
    for (uint32_t i = 0U; i < IMU_DATA_BYTES; i++) {
        bytes[i] = spi_transfer(0xFFU);
    }
    imu_select(false);
    for (uint32_t i = 0U; i < 7U; i++) {
        const uint16_t word = (uint16_t)(((uint16_t)bytes[2U * i] << 8U) | (uint16_t)bytes[(2U * i) + 1U]);
        words[i] = (word > (uint16_t)INT16_MAX) ? (int16_t)((int32_t)word - 65536) : (int16_t)word;
    }
}

/* TM1640 ------------------------------------------------------------------------------------------------- */

#define DISPLAY_DIN            4U  /* PC4 */
#define DISPLAY_CLK            5U  /* PC5 */
#define DISPLAY_PAUSE_LOOPS    20U /* about 1 us; no SysTick, which the main loop may be using */
#define DISPLAY_AUTO_INCREMENT 0x40U
#define DISPLAY_ON_14_16       0x8FU
#define DISPLAY_FIRST_ADDRESS  0xC0U
#define DISPLAY_GRIDS          16U

static void display_line(uint32_t pin, bool high)
{
    stm32f1_gpio_write(GPIOC, pin, high);
    for (volatile uint32_t i = 0U; i < DISPLAY_PAUSE_LOOPS; i++) {
        /* pause */
    }
}

static void display_start(void)
{
    display_line(DISPLAY_DIN, true);
    display_line(DISPLAY_CLK, true);
    display_line(DISPLAY_DIN, false); /* start: DIN falls while CLK is high */
    display_line(DISPLAY_CLK, false);
}

static void display_stop(void)
{
    display_line(DISPLAY_DIN, false);
    display_line(DISPLAY_CLK, true);
    display_line(DISPLAY_DIN, true); /* stop: DIN rises while CLK is high */
    display_line(DISPLAY_CLK, false);
}

static void display_byte(uint8_t value)
{
    for (uint32_t bit = 0U; bit < 8U; bit++) { /* LSB first, data valid on the rising edge */
        display_line(DISPLAY_CLK, false);
        display_line(DISPLAY_DIN, ((uint32_t)value & ((uint32_t)1U << bit)) != 0U);
        display_line(DISPLAY_CLK, true);
    }
    display_line(DISPLAY_CLK, false);
}

void stm32f1_display_init(void)
{
    stm32f1_gpio_mode(GPIOC, DISPLAY_DIN, STM32F1_PIN_OUTPUT_50MHZ);
    stm32f1_gpio_mode(GPIOC, DISPLAY_CLK, STM32F1_PIN_OUTPUT_50MHZ);
    stm32f1_gpio_write(GPIOC, DISPLAY_DIN, true);
    stm32f1_gpio_write(GPIOC, DISPLAY_CLK, true);
}

void stm32f1_display_send(const uint8_t grids[16])
{
    display_start();
    display_byte(DISPLAY_AUTO_INCREMENT);
    display_stop();
    display_start();
    display_byte(DISPLAY_ON_14_16);
    display_stop();
    display_start();
    display_byte(DISPLAY_FIRST_ADDRESS);
    for (uint32_t i = 0U; i < DISPLAY_GRIDS; i++) {
        display_byte(grids[i]);
    }
    display_stop();
}

/* WS2812 ---------------------------------------------------------------------------------------------------
 */

#define LED_BITS_PER_LED 24U
#define LED_RESET_SLOTS  1000U /* low time between frames */
#define LED_SLOTS        ((STM32F1_LED_COUNT * LED_BITS_PER_LED) + LED_RESET_SLOTS)
#define LED_ONE          20U /* compare counts of a 1 bit (of 30 per 1.25 us) */
#define LED_ZERO         8U
#define LED_TIMER_PSC    2U             /* 72 MHz / 3 = 24 MHz */
#define LED_TIMER_ARR    29U            /* 24 MHz / 30 = 800 kHz */
#define LED_DMA          (&DMA1->CH[1]) /* channel 2: TIM3 CC3 request */

static uint8_t led_slots[LED_SLOTS];

void stm32f1_led_strip_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3;
    RCC->AHBENR |= RCC_AHBENR_DMA1;
    stm32f1_gpio_mode(GPIOA, 6U, STM32F1_PIN_ALTERNATE_50MHZ);
    for (uint32_t i = 0U; i < (STM32F1_LED_COUNT * LED_BITS_PER_LED); i++) {
        led_slots[i] = LED_ZERO;
    }

    TIM3->PSC = LED_TIMER_PSC;
    TIM3->ARR = LED_TIMER_ARR;
    TIM3->CCMR1 = 0x0068U; /* CH1: PWM mode 1, compare preload */
    TIM3->CCMR2 = 0U;
    TIM3->CCR1 = 0U;
    TIM3->CCR3 = 1U; /* CH3 compare: one DMA request per bit, early in the bit */
    TIM3->CCER = TIM_CCER_CC1E;
    TIM3->DIER = TIM_DIER_CC3DE;

    LED_DMA->CCR = 0U;
    LED_DMA->CPAR = (uint32_t)(uintptr_t)&TIM3->CCR1;
    LED_DMA->CMAR = (uint32_t)(uintptr_t)led_slots;
    LED_DMA->CNDTR = LED_SLOTS;
    LED_DMA->CCR =
        DMA_CCR_DIR_TO_PER | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_16 | DMA_CCR_PL_HIGH | DMA_CCR_EN;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void stm32f1_led_strip_set(uint32_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= STM32F1_LED_COUNT) {
        return;
    }
    const uint32_t grb = ((uint32_t)green << 16U) | ((uint32_t)red << 8U) | (uint32_t)blue;
    for (uint32_t bit = 0U; bit < LED_BITS_PER_LED; bit++) {
        const bool one = (grb & ((uint32_t)1U << (LED_BITS_PER_LED - 1U - bit))) != 0U;
        led_slots[(index * LED_BITS_PER_LED) + bit] = one ? (uint8_t)LED_ONE : (uint8_t)LED_ZERO;
    }
}

/* Buzzer ---------------------------------------------------------------------------------------------------
 */

#define BUZZER_PSC 39U  /* 72 MHz / 40 = 1.8 MHz */
#define BUZZER_ARR 666U /* 1.8 MHz / 667 = 2.70 kHz */

void stm32f1_buzzer_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4;
    stm32f1_gpio_mode(GPIOB, 9U, STM32F1_PIN_ALTERNATE_10MHZ);
    TIM4->PSC = BUZZER_PSC;
    TIM4->ARR = BUZZER_ARR;
    TIM4->CCMR2 = 0x6800U; /* CH4: PWM mode 1, compare preload */
    TIM4->CCR4 = 0U;
    TIM4->CCER = 0x1000U; /* CC4E */
    TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void stm32f1_buzzer_set(uint16_t compare)
{
    TIM4->CCR4 = compare;
}
