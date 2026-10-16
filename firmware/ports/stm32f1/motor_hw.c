/*
 * Motor hardware: TIM8 three-phase PWM (RM0008 14) and the ADC1/ADC2 injected pair (RM0008 11).
 *
 * TIM8 counts up and down (centre-aligned mode 1) at 72 MHz with no repetition, so the update interrupt comes
 * twice per PWM period. CH1..3 and their complements are PWM mode 1, active high, compare preload on; the
 * main output enable (MOE) stays off until the motor layer turns the bridge on. The break input is not used.
 *
 * The ADCs run in dual injected simultaneous mode, started by software from the TIM8 interrupt, left aligned:
 * a data register holds the 12-bit sample x 8.
 */
#include <stdbool.h>
#include <stdint.h>

#include "stm32f1/device.h"
#include "stm32f1/port.h"
#include "stm32f1/regs.h"

#define CCMR1_VALUE 0x6868U  /* CH1 and CH2: PWM mode 1 (OCxM 110), compare preload (OCxPE) */
#define CCMR2_VALUE 0x0068U  /* CH3: the same */
#define CCER_BRIDGE 0x0555UL /* CC1E, CC1NE, CC2E, CC2NE, CC3E, CC3NE */

#define ADC_JL_SHIFT     20U
#define ADC_JSQ_BITS     5U
#define OFFSET_SAMPLES   16U
#define DATA_TO_RAW      3U
#define OFFSET_SETTLE_MS 10U

/* ADC channels (RM0008 table "ADC1 channels"): PA0 = 0, PA4 = 4, PA5 = 5, PC0 = 10, PC1 = 11. */
#define CH_BATTERY 0U
#define CH_PHASE_A 4U
#define CH_PHASE_B 5U
#define CH_BUS_I   10U
#define CH_NTC     11U

void stm32f1_pwm_init(uint16_t period_counts, uint8_t dead_time_counts)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM8;
    stm32f1_gpio_mode(GPIOC, 6U, STM32F1_PIN_ALTERNATE_50MHZ); /* CH1 */
    stm32f1_gpio_mode(GPIOC, 7U, STM32F1_PIN_ALTERNATE_50MHZ); /* CH2 */
    stm32f1_gpio_mode(GPIOC, 8U, STM32F1_PIN_ALTERNATE_50MHZ); /* CH3 */
    stm32f1_gpio_mode(GPIOA, 7U, STM32F1_PIN_ALTERNATE_50MHZ); /* CH1N */
    stm32f1_gpio_mode(GPIOB, 0U, STM32F1_PIN_ALTERNATE_50MHZ); /* CH2N */
    stm32f1_gpio_mode(GPIOB, 1U, STM32F1_PIN_ALTERNATE_50MHZ); /* CH3N */

    TIM8->CR1 = TIM_CR1_CMS_CENTER1 | TIM_CR1_ARPE;
    TIM8->PSC = 0U;
    TIM8->ARR = period_counts;
    TIM8->RCR = 0U;
    TIM8->CCMR1 = CCMR1_VALUE;
    TIM8->CCMR2 = CCMR2_VALUE;
    TIM8->CCR1 = (uint32_t)period_counts / 2U;
    TIM8->CCR2 = (uint32_t)period_counts / 2U;
    TIM8->CCR3 = (uint32_t)period_counts / 2U;
    TIM8->CCER = CCER_BRIDGE;
    TIM8->BDTR = dead_time_counts; /* MOE, AOE, break: off */
    TIM8->EGR = TIM_EGR_UG;
    TIM8->SR = 0U;
    TIM8->DIER = TIM_DIER_UIE;
    stm32f1_nvic_enable(TIM8_UP_IRQn, 0U, 1U);
    TIM8->CR1 |= TIM_CR1_CEN;
}

void stm32f1_pwm_set(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3)
{
    TIM8->CCR1 = ccr1;
    TIM8->CCR2 = ccr2;
    TIM8->CCR3 = ccr3;
}

void stm32f1_pwm_outputs(bool enabled)
{
    if (enabled) {
        TIM8->BDTR |= TIM_BDTR_MOE;
    } else {
        TIM8->BDTR &= ~(TIM_BDTR_MOE | TIM_BDTR_AOE);
    }
}

bool stm32f1_pwm_counting_down(void)
{
    return (TIM8->CR1 & TIM_CR1_DIR) != 0U;
}

void stm32f1_pwm_clear_update(void)
{
    TIM8->SR = ~TIM_SR_UIF;
}

static uint32_t jsqr(uint32_t count, const uint32_t *channels)
{
    /* With n conversions the sequence occupies the last n of the four slots JSQ1..JSQ4. */
    uint32_t value = (count - 1U) << ADC_JL_SHIFT;
    for (uint32_t rank = 0U; rank < count; rank++) {
        const uint32_t slot = (4U - count) + rank;
        value |= channels[rank] << (slot * ADC_JSQ_BITS);
    }
    return value;
}

static void adc_calibrate(struct stm32f1_adc *adc)
{
    adc->CR2 |= ADC_CR2_RSTCAL;
    while ((adc->CR2 & ADC_CR2_RSTCAL) != 0U) {
        /* reset the calibration registers */
    }
    adc->CR2 |= ADC_CR2_CAL;
    while ((adc->CR2 & ADC_CR2_CAL) != 0U) {
        /* calibrate */
    }
}

static const uint32_t adc1_run_channels[3] = {CH_PHASE_A, CH_BATTERY, CH_NTC};

void stm32f1_adc_init(void)
{
    static const uint32_t adc2_run_channels[2] = {CH_PHASE_B, CH_BUS_I};
    RCC->APB2ENR |= RCC_APB2ENR_ADC1 | RCC_APB2ENR_ADC2;
    stm32f1_gpio_mode(GPIOA, 0U, STM32F1_PIN_ANALOG);
    stm32f1_gpio_mode(GPIOA, 4U, STM32F1_PIN_ANALOG);
    stm32f1_gpio_mode(GPIOA, 5U, STM32F1_PIN_ANALOG);
    stm32f1_gpio_mode(GPIOC, 0U, STM32F1_PIN_ANALOG);
    stm32f1_gpio_mode(GPIOC, 1U, STM32F1_PIN_ANALOG);

    const uint32_t control = ADC_CR2_ALIGN_LEFT | ADC_CR2_JEXTSEL_SW | ADC_CR2_JEXTTRIG | ADC_CR2_EXTSEL_SW;
    ADC1->CR1 = ADC_CR1_DUAL_INJ_SIM | ADC_CR1_SCAN;
    ADC1->CR2 = control;
    ADC2->CR1 = ADC_CR1_SCAN;
    ADC2->CR2 = control;
    const uint32_t sample = ADC_SAMPLE_71_5;
    ADC1->SMPR2 = (sample << (3U * CH_BATTERY)) | (sample << (3U * CH_PHASE_A));
    ADC1->SMPR1 = sample << (3U * (CH_NTC - 10U));
    ADC2->SMPR2 = sample << (3U * CH_PHASE_B);
    ADC2->SMPR1 = sample << (3U * (CH_BUS_I - 10U));
    ADC1->JSQR = jsqr(3U, adc1_run_channels);
    ADC2->JSQR = jsqr(2U, adc2_run_channels);

    ADC1->CR2 |= ADC_CR2_ADON;
    ADC2->CR2 |= ADC_CR2_ADON;
    adc_calibrate(ADC1);
    adc_calibrate(ADC2);
}

static uint32_t data_to_raw(volatile const uint32_t *data)
{
    return (*data & 0xFFFFU) >> DATA_TO_RAW;
}

void stm32f1_adc_measure_offsets(struct stm32f1_current_sums *sums)
{
    static const uint32_t phases[2] = {CH_PHASE_A, CH_PHASE_B};
    sums->adc1_rank1 = 0U;
    sums->adc1_rank2 = 0U;
    sums->adc2_rank1 = 0U;

    ADC1->CR1 &= ~ADC_CR1_JEOCIE;
    ADC1->JSQR = jsqr(2U, phases);
    ADC1->SR = ~ADC_SR_JEOC;
    stm32f1_adc_start();
    stm32f1_delay_ms(OFFSET_SETTLE_MS);
    for (uint32_t i = 0U; i < OFFSET_SAMPLES; i++) {
        while ((ADC1->SR & ADC_SR_JEOC) == 0U) {
            /* conversion */
        }
        sums->adc1_rank1 += data_to_raw(&ADC1->JDR[0]);
        sums->adc1_rank2 += data_to_raw(&ADC1->JDR[1]);
        sums->adc2_rank1 += data_to_raw(&ADC2->JDR[0]);
        ADC1->SR = ~ADC_SR_JEOC;
        stm32f1_adc_start();
    }
    while ((ADC1->SR & ADC_SR_JEOC) == 0U) {
        /* let the last conversion finish */
    }
    ADC1->SR = ~ADC_SR_JEOC;
    ADC1->JSQR = jsqr(3U, adc1_run_channels);
}

void stm32f1_adc_enable_interrupt(void)
{
    ADC1->SR = ~ADC_SR_JEOC;
    ADC1->CR1 |= ADC_CR1_JEOCIE;
    stm32f1_nvic_enable(ADC1_2_IRQn, 0U, 0U);
}

void stm32f1_adc_start(void)
{
    ADC1->CR2 |= ADC_CR2_JEXTTRIG | ADC_CR2_JSWSTART;
}

bool stm32f1_adc_take_end_of_conversion(void)
{
    if ((ADC1->SR & ADC_SR_JEOC) == 0U) {
        return false;
    }
    ADC1->SR = ~ADC_SR_JEOC;
    return true;
}

uint16_t stm32f1_adc1_injected(uint32_t rank)
{
    return (uint16_t)(ADC1->JDR[rank - 1U] & 0xFFFFU);
}

uint16_t stm32f1_adc2_injected(uint32_t rank)
{
    return (uint16_t)(ADC2->JDR[rank - 1U] & 0xFFFFU);
}
