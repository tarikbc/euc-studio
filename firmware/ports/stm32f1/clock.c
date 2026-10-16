/*
 * Clock tree, SysTick delays, NVIC grouping and reset (RM0008 7, PM0056 4.4 and 4.5).
 */
#include <stdbool.h>
#include <stdint.h>

#include "euc/quirks.h"
#include "stm32f1/device.h"
#include "stm32f1/port.h"
#include "stm32f1/regs.h"

#define HSE_START_POLLS      0x500U   /* the stock HSE start-up wait */
#define SYSTICK_HZ           9000000U /* HCLK / 8 */
#define SYSTICK_TICKS_PER_MS (SYSTICK_HZ / 1000U)
#define SYSTICK_TICKS_PER_US (SYSTICK_HZ / 1000000U)
#define SYSTICK_MAX_RELOAD   0xFFFFFFU
#define SYSTICK_MAX_MS       1800U
#define NVIC_GROUP_2_PREEMPT 5U /* PRIGROUP: 2 bits pre-emption, 2 bits sub-priority */

bool stm32f1_clock_init(void)
{
    RCC->CR |= RCC_CR_HSEON;
    uint32_t polls = 0U;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
        polls++;
        if (polls >= HSE_START_POLLS) {
            if (EUC_QUIRK(HSE_START_HANG)) {
                for (;;) {
                    /* stock: no clock, no firmware */
                }
            }
            return false;
        }
    }

    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    uint32_t cfgr = RCC->CFGR;
    cfgr &= ~(RCC_CFGR_HPRE_MASK | RCC_CFGR_PPRE1_MASK | RCC_CFGR_PPRE2_MASK | RCC_CFGR_ADC_MASK |
              RCC_CFGR_PLLMUL_MASK | RCC_CFGR_PLLXTPRE);
    cfgr |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_ADC_DIV6 | RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMUL9;
    RCC->CFGR = cfgr;

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {
        /* PLL lock */
    }
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL) {
        /* switch to the PLL */
    }
    return true;
}

static void systick_wait(uint32_t ticks)
{
    SYSTICK->LOAD = ticks - 1U;
    SYSTICK->VAL = 0U;
    SYSTICK->CTRL = SYSTICK_CTRL_ENABLE; /* clock source HCLK / 8 */
    while ((SYSTICK->CTRL & SYSTICK_CTRL_COUNTFLAG) == 0U) {
        /* wait */
    }
    SYSTICK->CTRL = 0U;
    SYSTICK->VAL = 0U;
}

void stm32f1_delay_ms(uint32_t ms)
{
    uint32_t left = ms;
    while (left > 0U) {
        const uint32_t chunk = (left > SYSTICK_MAX_MS) ? SYSTICK_MAX_MS : left;
        systick_wait(chunk * SYSTICK_TICKS_PER_MS);
        left -= chunk;
    }
}

void stm32f1_delay_us(uint32_t us)
{
    if (us > 0U) {
        const uint32_t ticks = us * SYSTICK_TICKS_PER_US;
        systick_wait((ticks > SYSTICK_MAX_RELOAD) ? SYSTICK_MAX_RELOAD : ticks);
    }
}

void stm32f1_system_reset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void stm32f1_nvic_init(void)
{
    NVIC_SetPriorityGrouping(NVIC_GROUP_2_PREEMPT);
}

void stm32f1_nvic_enable(int32_t irq, uint32_t preempt, uint32_t sub)
{
    const IRQn_Type line = (IRQn_Type)irq;
    NVIC_SetPriority(line, NVIC_EncodePriority(NVIC_GROUP_2_PREEMPT, preempt, sub));
    NVIC_EnableIRQ(line);
}
