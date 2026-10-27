/*
 * The STM32F1 port API (ports/stm32f1/include/stm32f1/port.h) for the host SIL library.
 *
 * Each function makes the register accesses its ports/stm32f1 counterpart makes, in the same order and with
 * the same values, through the host (sil_read and sil_write), so the engine's peripheral models see what they
 * see from the image. Keep the two in step: a change to a driver there needs the same change here.
 *
 * What differs is what cannot cross to the host: the firmware's own memory. Flash pages live in host memory
 * and are reloaded from the board after every write (host.c); the DMA buffers of the BMS link and the LED
 * strip live in emulated SRAM at EUC_SIL_DMA_RAM_ADDRESS, copied to and from the firmware's buffers; pause
 * loops become spins. Register addresses come from the layouts in stm32f1/regs.h.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "euc/bms/bms.h"
#include "euc/quirks.h"
#include "euc/sil.h"
#include "sil_port.h"
#include "stm32f1/irqn.h"
#include "stm32f1/port.h"
#include "stm32f1/regs.h"

/* Register addresses ----------------------------------------------------------------------------------- */

/* The bus address of a register, named as the target port names it (`REG(TIM8->CCR1)`): the address the
 * regs.h instance macros and layouts give it. Nothing is read or written through the pointer. */
#define REG(reg) ((uint32_t)(uintptr_t)&(reg))

/* System control block and NVIC (PM0056 4.3, 4.4), which CMSIS defines on the target. */
#define SCB_AIRCR           0xE000ED0CU
#define AIRCR_VECTKEY       0x05FA0000U
#define AIRCR_VECTKEY_MASK  0xFFFF0000U
#define AIRCR_PRIGROUP_MASK 0x00000700U
#define AIRCR_PRIGROUP_POS  8U
#define AIRCR_SYSRESETREQ   0x00000004U
#define NVIC_ISER           0xE000E100U
#define NVIC_IPR            0xE000E400U
#define NVIC_PRIO_BITS      4U

static uint32_t read32(uint32_t address)
{
    return sil_read(address, 4U);
}

static void write32(uint32_t address, uint32_t value)
{
    sil_write(address, 4U, value);
}

/* `register |= bits` and `register &= ~bits`: a read, then a write, as on the MCU. */
static void set_bits(uint32_t address, uint32_t bits)
{
    write32(address, read32(address) | bits);
}

static void clear_bits(uint32_t address, uint32_t bits)
{
    write32(address, read32(address) & ~bits);
}

/* Clock, delays, reset (ports/stm32f1/clock.c) ----------------------------------------------------------- */

#define HSE_START_POLLS      0x500U
#define SYSTICK_HZ           9000000U
#define SYSTICK_TICKS_PER_MS (SYSTICK_HZ / 1000U)
#define SYSTICK_TICKS_PER_US (SYSTICK_HZ / 1000000U)
#define SYSTICK_MAX_RELOAD   0xFFFFFFU
#define SYSTICK_MAX_MS       1800U
#define NVIC_GROUP_2_PREEMPT 5U
/* A parked core (a stock hang, a reset on its way) spins in steps this long until the host stops it. */
#define PARK_CYCLES 72000U

bool stm32f1_clock_init(void)
{
    set_bits(REG(RCC->CR), RCC_CR_HSEON);
    uint32_t polls = 0U;
    while ((read32(REG(RCC->CR)) & RCC_CR_HSERDY) == 0U) {
        polls++;
        if (polls >= HSE_START_POLLS) {
            while (EUC_QUIRK(HSE_START_HANG)) {
                sil_spin(PARK_CYCLES); /* stock: no clock, no firmware */
            }
            return false;
        }
    }

    write32(REG(FLASH->ACR), FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2);
    uint32_t cfgr = read32(REG(RCC->CFGR));
    cfgr &= ~(RCC_CFGR_HPRE_MASK | RCC_CFGR_PPRE1_MASK | RCC_CFGR_PPRE2_MASK | RCC_CFGR_ADC_MASK |
              RCC_CFGR_PLLMUL_MASK | RCC_CFGR_PLLXTPRE);
    cfgr |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_ADC_DIV6 | RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMUL9;
    write32(REG(RCC->CFGR), cfgr);

    set_bits(REG(RCC->CR), RCC_CR_PLLON);
    while ((read32(REG(RCC->CR)) & RCC_CR_PLLRDY) == 0U) {
        /* PLL lock */
    }
    write32(REG(RCC->CFGR), (read32(REG(RCC->CFGR)) & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_PLL);
    while ((read32(REG(RCC->CFGR)) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL) {
        /* switch to the PLL */
    }
    return true;
}

static void systick_wait(uint32_t ticks)
{
    write32(REG(SYSTICK->LOAD), ticks - 1U);
    write32(REG(SYSTICK->VAL), 0U);
    write32(REG(SYSTICK->CTRL), SYSTICK_CTRL_ENABLE);
    while ((read32(REG(SYSTICK->CTRL)) & SYSTICK_CTRL_COUNTFLAG) == 0U) {
        /* wait */
    }
    write32(REG(SYSTICK->CTRL), 0U);
    write32(REG(SYSTICK->VAL), 0U);
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
    /* CMSIS NVIC_SystemReset: keep the priority grouping, request the reset, wait for it. */
    const uint32_t prigroup = read32(SCB_AIRCR) & AIRCR_PRIGROUP_MASK;
    write32(SCB_AIRCR, AIRCR_VECTKEY | prigroup | AIRCR_SYSRESETREQ);
    for (;;) {
        sil_spin(PARK_CYCLES); /* the board resets the core, and the host stops the firmware */
    }
}

void stm32f1_nvic_init(void)
{
    /* CMSIS NVIC_SetPriorityGrouping */
    const uint32_t aircr = read32(SCB_AIRCR) & ~(AIRCR_VECTKEY_MASK | AIRCR_PRIGROUP_MASK);
    write32(SCB_AIRCR, aircr | AIRCR_VECTKEY | ((uint32_t)NVIC_GROUP_2_PREEMPT << AIRCR_PRIGROUP_POS));
}

void stm32f1_nvic_enable(int32_t irq, uint32_t preempt, uint32_t sub)
{
    /* CMSIS NVIC_EncodePriority with grouping 5 (2 pre-emption bits, 2 sub-priority bits of 4), then
     * NVIC_SetPriority (one IPR byte, priority in its top bits) and NVIC_EnableIRQ. */
    const uint32_t line = (uint32_t)irq;
    const uint32_t priority = ((preempt & 3U) << 2U) | (sub & 3U);
    sil_write(NVIC_IPR + line, 1U, (priority << (8U - NVIC_PRIO_BITS)) & 0xFFU);
    write32(NVIC_ISER + (4U * (line >> 5U)), (uint32_t)1U << (line & 0x1FU));
}

/* GPIO (ports/stm32f1/gpio.c) -------------------------------------------------------------------------- */

#define PIN_CONFIG_BITS   4U
#define PIN_CONFIG_MASK   0xFU
#define PINS_PER_REGISTER 8U

void stm32f1_gpio_clocks_on(void)
{
    set_bits(REG(RCC->APB2ENR), RCC_APB2ENR_AFIO | RCC_APB2ENR_GPIOA | RCC_APB2ENR_GPIOB | RCC_APB2ENR_GPIOC);
}

/* port.h's signature: the target writes the port's registers through the pointer; here it only names them. */
/* cppcheck-suppress constParameterPointer */
void stm32f1_gpio_mode(struct stm32f1_gpio *port, uint32_t pin, enum stm32f1_pin_mode mode)
{
    const uint32_t config = (pin < PINS_PER_REGISTER) ? REG(port->CRL) : REG(port->CRH);
    const uint32_t shift = (pin % PINS_PER_REGISTER) * PIN_CONFIG_BITS;
    write32(config, (read32(config) & ~(PIN_CONFIG_MASK << shift)) | ((uint32_t)mode << shift));
}

void stm32f1_gpio_pull(struct stm32f1_gpio *port, uint32_t pin, bool up)
{
    stm32f1_gpio_write(port, pin, up);
}

/* port.h's signature, as for stm32f1_gpio_mode. */
/* cppcheck-suppress constParameterPointer */
void stm32f1_gpio_write(struct stm32f1_gpio *port, uint32_t pin, bool high)
{
    if (high) {
        write32(REG(port->BSRR), (uint32_t)1U << pin);
    } else {
        write32(REG(port->BRR), (uint32_t)1U << pin);
    }
}

bool stm32f1_gpio_read(const struct stm32f1_gpio *port, uint32_t pin)
{
    return (read32(REG(port->IDR)) & ((uint32_t)1U << pin)) != 0U;
}

void stm32f1_gpio_release_jtag(void)
{
    write32(REG(AFIO->MAPR), (read32(REG(AFIO->MAPR)) & ~AFIO_MAPR_SWJ_MASK) | AFIO_MAPR_SWJ_SWONLY);
}

/* Motor timer and ADC (ports/stm32f1/motor_hw.c) ------------------------------------------------------- */

#define CCMR1_VALUE 0x6868U
#define CCMR2_VALUE 0x0068U
#define CCER_BRIDGE 0x0555UL

#define ADC_JL_SHIFT     20U
#define ADC_JSQ_BITS     5U
#define OFFSET_SAMPLES   16U
#define DATA_TO_RAW      3U
#define OFFSET_SETTLE_MS 10U

#define CH_BATTERY 0U
#define CH_PHASE_A 4U
#define CH_PHASE_B 5U
#define CH_BUS_I   10U
#define CH_NTC     11U

void stm32f1_pwm_init(uint16_t period_counts, uint8_t dead_time_counts)
{
    set_bits(REG(RCC->APB2ENR), RCC_APB2ENR_TIM8);
    stm32f1_gpio_mode(GPIOC, 6U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOC, 7U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOC, 8U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOA, 7U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOB, 0U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOB, 1U, STM32F1_PIN_ALTERNATE_50MHZ);

    write32(REG(TIM8->CR1), TIM_CR1_CMS_CENTER1 | TIM_CR1_ARPE);
    write32(REG(TIM8->PSC), 0U);
    write32(REG(TIM8->ARR), period_counts);
    write32(REG(TIM8->RCR), 0U);
    write32(REG(TIM8->CCMR1), CCMR1_VALUE);
    write32(REG(TIM8->CCMR2), CCMR2_VALUE);
    write32(REG(TIM8->CCR1), (uint32_t)period_counts / 2U);
    write32(REG(TIM8->CCR2), (uint32_t)period_counts / 2U);
    write32(REG(TIM8->CCR3), (uint32_t)period_counts / 2U);
    write32(REG(TIM8->CCER), CCER_BRIDGE);
    write32(REG(TIM8->BDTR), dead_time_counts);
    write32(REG(TIM8->EGR), TIM_EGR_UG);
    write32(REG(TIM8->SR), 0U);
    write32(REG(TIM8->DIER), TIM_DIER_UIE);
    stm32f1_nvic_enable(TIM8_UP_IRQn, 0U, 1U);
    set_bits(REG(TIM8->CR1), TIM_CR1_CEN);
}

void stm32f1_pwm_set(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3)
{
    write32(REG(TIM8->CCR1), ccr1);
    write32(REG(TIM8->CCR2), ccr2);
    write32(REG(TIM8->CCR3), ccr3);
}

void stm32f1_pwm_outputs(bool enabled)
{
    if (enabled) {
        set_bits(REG(TIM8->BDTR), TIM_BDTR_MOE);
    } else {
        clear_bits(REG(TIM8->BDTR), TIM_BDTR_MOE | TIM_BDTR_AOE);
    }
}

bool stm32f1_pwm_counting_down(void)
{
    return (read32(REG(TIM8->CR1)) & TIM_CR1_DIR) != 0U;
}

void stm32f1_pwm_clear_update(void)
{
    write32(REG(TIM8->SR), ~TIM_SR_UIF);
}

static uint32_t jsqr(uint32_t count, const uint32_t *channels)
{
    uint32_t value = (count - 1U) << ADC_JL_SHIFT;
    for (uint32_t rank = 0U; rank < count; rank++) {
        const uint32_t slot = (4U - count) + rank;
        value |= channels[rank] << (slot * ADC_JSQ_BITS);
    }
    return value;
}

static void adc_calibrate(const struct stm32f1_adc *adc)
{
    set_bits(REG(adc->CR2), ADC_CR2_RSTCAL);
    while ((read32(REG(adc->CR2)) & ADC_CR2_RSTCAL) != 0U) {
        /* reset the calibration registers */
    }
    set_bits(REG(adc->CR2), ADC_CR2_CAL);
    while ((read32(REG(adc->CR2)) & ADC_CR2_CAL) != 0U) {
        /* calibrate */
    }
}

static const uint32_t adc1_run_channels[3] = {CH_PHASE_A, CH_BATTERY, CH_NTC};

void stm32f1_adc_init(void)
{
    static const uint32_t adc2_run_channels[2] = {CH_PHASE_B, CH_BUS_I};
    set_bits(REG(RCC->APB2ENR), RCC_APB2ENR_ADC1 | RCC_APB2ENR_ADC2);
    stm32f1_gpio_mode(GPIOA, 0U, STM32F1_PIN_ANALOG);
    stm32f1_gpio_mode(GPIOA, 4U, STM32F1_PIN_ANALOG);
    stm32f1_gpio_mode(GPIOA, 5U, STM32F1_PIN_ANALOG);
    stm32f1_gpio_mode(GPIOC, 0U, STM32F1_PIN_ANALOG);
    stm32f1_gpio_mode(GPIOC, 1U, STM32F1_PIN_ANALOG);

    const uint32_t control = ADC_CR2_ALIGN_LEFT | ADC_CR2_JEXTSEL_SW | ADC_CR2_JEXTTRIG | ADC_CR2_EXTSEL_SW;
    write32(REG(ADC1->CR1), ADC_CR1_DUAL_INJ_SIM | ADC_CR1_SCAN);
    write32(REG(ADC1->CR2), control);
    write32(REG(ADC2->CR1), ADC_CR1_SCAN);
    write32(REG(ADC2->CR2), control);
    const uint32_t sample = ADC_SAMPLE_71_5;
    write32(REG(ADC1->SMPR2), (sample << (3U * CH_BATTERY)) | (sample << (3U * CH_PHASE_A)));
    write32(REG(ADC1->SMPR1), sample << (3U * (CH_NTC - 10U)));
    write32(REG(ADC2->SMPR2), sample << (3U * CH_PHASE_B));
    write32(REG(ADC2->SMPR1), sample << (3U * (CH_BUS_I - 10U)));
    write32(REG(ADC1->JSQR), jsqr(3U, adc1_run_channels));
    write32(REG(ADC2->JSQR), jsqr(2U, adc2_run_channels));

    set_bits(REG(ADC1->CR2), ADC_CR2_ADON);
    set_bits(REG(ADC2->CR2), ADC_CR2_ADON);
    adc_calibrate(ADC1);
    adc_calibrate(ADC2);
}

static uint32_t data_to_raw(uint32_t address)
{
    return (read32(address) & 0xFFFFU) >> DATA_TO_RAW;
}

void stm32f1_adc_measure_offsets(struct stm32f1_current_sums *sums)
{
    static const uint32_t phases[2] = {CH_PHASE_A, CH_PHASE_B};
    sums->adc1_rank1 = 0U;
    sums->adc1_rank2 = 0U;
    sums->adc2_rank1 = 0U;

    clear_bits(REG(ADC1->CR1), ADC_CR1_JEOCIE);
    write32(REG(ADC1->JSQR), jsqr(2U, phases));
    write32(REG(ADC1->SR), ~ADC_SR_JEOC);
    stm32f1_adc_start();
    stm32f1_delay_ms(OFFSET_SETTLE_MS);
    for (uint32_t i = 0U; i < OFFSET_SAMPLES; i++) {
        while ((read32(REG(ADC1->SR)) & ADC_SR_JEOC) == 0U) {
            /* conversion */
        }
        sums->adc1_rank1 += data_to_raw(REG(ADC1->JDR[0]));
        sums->adc1_rank2 += data_to_raw(REG(ADC1->JDR[1]));
        sums->adc2_rank1 += data_to_raw(REG(ADC2->JDR[0]));
        write32(REG(ADC1->SR), ~ADC_SR_JEOC);
        stm32f1_adc_start();
    }
    while ((read32(REG(ADC1->SR)) & ADC_SR_JEOC) == 0U) {
        /* let the last conversion finish */
    }
    write32(REG(ADC1->SR), ~ADC_SR_JEOC);
    write32(REG(ADC1->JSQR), jsqr(3U, adc1_run_channels));
}

void stm32f1_adc_enable_interrupt(void)
{
    write32(REG(ADC1->SR), ~ADC_SR_JEOC);
    set_bits(REG(ADC1->CR1), ADC_CR1_JEOCIE);
    stm32f1_nvic_enable(ADC1_2_IRQn, 0U, 0U);
}

void stm32f1_adc_start(void)
{
    set_bits(REG(ADC1->CR2), ADC_CR2_JEXTTRIG | ADC_CR2_JSWSTART);
}

bool stm32f1_adc_take_end_of_conversion(void)
{
    if ((read32(REG(ADC1->SR)) & ADC_SR_JEOC) == 0U) {
        return false;
    }
    write32(REG(ADC1->SR), ~ADC_SR_JEOC);
    return true;
}

uint16_t stm32f1_adc1_injected(uint32_t rank)
{
    return (uint16_t)(read32(REG(ADC1->JDR[rank - 1U])) & 0xFFFFU);
}

uint16_t stm32f1_adc2_injected(uint32_t rank)
{
    return (uint16_t)(read32(REG(ADC2->JDR[rank - 1U])) & 0xFFFFU);
}

/* RTC, backup registers, flash, watchdog (ports/stm32f1/backup.c) --------------------------------------- */

#define RTC_PRESCALER_RELOAD 125U

#define BKP_DR1_OFFSET  0x04U
#define BKP_DR11_OFFSET 0x40U
#define BKP_LOW_COUNT   10U

#define IWDG_KEY_UNLOCK   0x5555U
#define IWDG_KEY_RELOAD   0xAAAAU
#define IWDG_KEY_START    0xCCCCU
#define IWDG_PRESCALER_64 4U
#define IWDG_TICKS_PER_S  625U
#define IWDG_MAX_RELOAD   0xFFFU

static void rtc_wait_write_done(void)
{
    while ((read32(REG(RTC->CRL)) & RTC_CRL_RTOFF) == 0U) {
        /* last write still going */
    }
}

void stm32f1_rtc_tick_init(void)
{
    stm32f1_backup_init();
    set_bits(REG(RCC->BDCR), RCC_BDCR_LSEON);
    write32(REG(RCC->BDCR),
            (read32(REG(RCC->BDCR)) & ~RCC_BDCR_RTCSEL_MASK) | RCC_BDCR_RTCSEL_HSE | RCC_BDCR_RTCEN);
    clear_bits(REG(RTC->CRL), RTC_CRL_RSF);
    while ((read32(REG(RTC->CRL)) & RTC_CRL_RSF) == 0U) {
        /* registers synchronised */
    }
    rtc_wait_write_done();
    set_bits(REG(RTC->CRH), RTC_CRH_SECIE);
    rtc_wait_write_done();
    set_bits(REG(RTC->CRL), RTC_CRL_CNF);
    write32(REG(RTC->PRLH), 0U);
    write32(REG(RTC->PRLL), RTC_PRESCALER_RELOAD);
    clear_bits(REG(RTC->CRL), RTC_CRL_CNF);
    rtc_wait_write_done();
    stm32f1_nvic_enable(RTC_IRQn, 2U, 2U);
}

void stm32f1_rtc_tick_acknowledge(void)
{
    clear_bits(REG(RTC->CRL), RTC_CRL_SECF);
    rtc_wait_write_done();
}

void stm32f1_backup_init(void)
{
    set_bits(REG(RCC->APB1ENR), RCC_APB1ENR_PWR | RCC_APB1ENR_BKP);
    set_bits(REG(PWR->CR), PWR_CR_DBP);
}

static uint32_t backup_register(uint32_t index)
{
    const uint32_t offset = (index <= BKP_LOW_COUNT) ? (BKP_DR1_OFFSET + (4U * (index - 1U)))
                                                     : (BKP_DR11_OFFSET + (4U * (index - 11U)));
    return BKP_BASE_ADDRESS + offset;
}

uint16_t stm32f1_backup_read(uint32_t index)
{
    return (uint16_t)(read32(backup_register(index)) & 0xFFFFU);
}

void stm32f1_backup_write(uint32_t index, uint16_t value)
{
    write32(backup_register(index), value);
}

static void flash_unlock(void)
{
    if ((read32(REG(FLASH->CR)) & FLASH_CR_LOCK) != 0U) {
        write32(REG(FLASH->KEYR), FLASH_KEY1);
        write32(REG(FLASH->KEYR), FLASH_KEY2);
    }
}

static void flash_wait(void)
{
    while ((read32(REG(FLASH->SR)) & FLASH_SR_BSY) != 0U) {
        /* busy */
    }
    write32(REG(FLASH->SR), FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);
}

static void flash_erase_unlocked(uint32_t address)
{
    flash_wait();
    set_bits(REG(FLASH->CR), FLASH_CR_PER);
    write32(REG(FLASH->AR), address);
    set_bits(REG(FLASH->CR), FLASH_CR_STRT);
    flash_wait();
    clear_bits(REG(FLASH->CR), FLASH_CR_PER);
}

static void flash_program_unlocked(uint32_t address, const uint16_t *halfwords, uint32_t count)
{
    set_bits(REG(FLASH->CR), FLASH_CR_PG);
    for (uint32_t i = 0U; i < count; i++) {
        sil_write(address + (2U * i), 2U, halfwords[i]);
        flash_wait();
    }
    clear_bits(REG(FLASH->CR), FLASH_CR_PG);
}

void stm32f1_flash_erase_page(const uint16_t *page)
{
    const uint32_t address = sil_flash_address(page);
    flash_unlock();
    flash_erase_unlocked(address);
    set_bits(REG(FLASH->CR), FLASH_CR_LOCK);
    sil_flash_reload(address);
}

void stm32f1_flash_program(const uint16_t *destination, const uint16_t *halfwords, uint32_t count)
{
    const uint32_t address = sil_flash_address(destination);
    flash_unlock();
    flash_program_unlocked(address, halfwords, count);
    set_bits(REG(FLASH->CR), FLASH_CR_LOCK);
    sil_flash_reload(address);
}

void stm32f1_flash_rewrite(const uint16_t *destination, const uint16_t *halfwords, uint32_t count)
{
    static uint16_t page[SIL_FLASH_PAGE_HALFWORDS];
    const uint32_t address = sil_flash_address(destination);
    const uint32_t page_address = address & ~(SIL_FLASH_PAGE_BYTES - 1U);
    /* The host copy of the page holds what the board's flash holds. */
    sil_load(page_address, (uint8_t *)page, SIL_FLASH_PAGE_BYTES);
    const uint32_t first = (address - page_address) / 2U;
    for (uint32_t i = 0U; (i < count) && ((first + i) < SIL_FLASH_PAGE_HALFWORDS); i++) {
        page[first + i] = halfwords[i];
    }
    flash_unlock();
    flash_erase_unlocked(page_address);
    flash_program_unlocked(page_address, page, SIL_FLASH_PAGE_HALFWORDS);
    set_bits(REG(FLASH->CR), FLASH_CR_LOCK);
    sil_flash_reload(page_address);
}

void stm32f1_watchdog_start(uint32_t timeout_ms)
{
    uint32_t reload = (timeout_ms * IWDG_TICKS_PER_S) / 1000U;
    if (reload > IWDG_MAX_RELOAD) {
        reload = IWDG_MAX_RELOAD;
    }
    write32(REG(IWDG->KR), IWDG_KEY_UNLOCK);
    write32(REG(IWDG->PR), IWDG_PRESCALER_64);
    write32(REG(IWDG->RLR), reload);
    write32(REG(IWDG->KR), IWDG_KEY_RELOAD);
    write32(REG(IWDG->KR), IWDG_KEY_START);
}

void stm32f1_watchdog_reload(void)
{
    write32(REG(IWDG->KR), IWDG_KEY_RELOAD);
}

bool stm32f1_reset_was_watchdog(void)
{
    const bool watchdog = (read32(REG(RCC->CSR)) & RCC_CSR_IWDGRSTF) != 0U;
    set_bits(REG(RCC->CSR), RCC_CSR_RMVF);
    return watchdog;
}

/* Serial ports (ports/stm32f1/serial.c) ----------------------------------------------------------------- */

#define BLE_BRR_115200 625U
#define BMS_BRR_9600   3750U
#if defined(EUC_FEATURE_IMU_TLM)
#define TX_RING_BYTES 512U
#else
#define TX_RING_BYTES 256U
#endif
#define BMS_DMA (&DMA1->CH[2]) /* channel 3 */
/* The BMS reply buffer the DMA fills, in emulated SRAM. */
#define BMS_DMA_ADDRESS EUC_SIL_DMA_RAM_ADDRESS

static uint8_t tx_ring[TX_RING_BYTES];
static uint32_t tx_head;
static uint32_t tx_tail;

/* The firmware's BMS reply buffer, which the port fills from the DMA buffer. */
static uint8_t *bms_buffer;
static uint16_t bms_buffer_bytes;

void stm32f1_ble_init(void)
{
    set_bits(REG(RCC->APB2ENR), RCC_APB2ENR_USART1);
    stm32f1_gpio_mode(GPIOA, 9U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOA, 10U, STM32F1_PIN_INPUT_PULL);
    stm32f1_gpio_pull(GPIOA, 10U, true);
    write32(REG(USART1->BRR), BLE_BRR_115200);
    write32(REG(USART1->CR2), 0U);
    write32(REG(USART1->CR3), 0U);
    write32(REG(USART1->CR1), USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);
}

bool stm32f1_ble_send(const uint8_t *bytes, uint32_t count)
{
    const uint32_t used = tx_head - tx_tail;
    if (count > (TX_RING_BYTES - used)) {
        return false;
    }
    uint32_t head = tx_head;
    for (uint32_t i = 0U; i < count; i++) {
        tx_ring[head % TX_RING_BYTES] = bytes[i];
        head++;
    }
    tx_head = head;
    set_bits(REG(USART1->CR1), USART_CR1_TXEIE);
    return true;
}

uint32_t stm32f1_ble_free(void)
{
    return TX_RING_BYTES - (tx_head - tx_tail);
}

bool stm32f1_ble_service(uint8_t *received)
{
    const uint32_t status = read32(REG(USART1->SR));
    bool got = false;
    if ((status & (USART_SR_RXNE | USART_SR_ORE)) != 0U) {
        *received = (uint8_t)(read32(REG(USART1->DR)) & 0xFFU);
        got = (status & USART_SR_RXNE) != 0U;
    }
    if (((read32(REG(USART1->CR1)) & USART_CR1_TXEIE) != 0U) && ((status & USART_SR_TXE) != 0U)) {
        if (tx_tail != tx_head) {
            write32(REG(USART1->DR), tx_ring[tx_tail % TX_RING_BYTES]);
            tx_tail++;
        } else {
            clear_bits(REG(USART1->CR1), USART_CR1_TXEIE);
        }
    }
    return got;
}

void stm32f1_bms_init(uint8_t *buffer, uint16_t length)
{
    bms_buffer = buffer;
    bms_buffer_bytes = length;
    set_bits(REG(RCC->APB1ENR), RCC_APB1ENR_USART3);
    set_bits(REG(RCC->AHBENR), RCC_AHBENR_DMA1);
    stm32f1_gpio_mode(GPIOB, 10U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOB, 11U, STM32F1_PIN_INPUT_FLOATING);
    write32(REG(USART3->BRR), BMS_BRR_9600);
    write32(REG(USART3->CR2), 0U);
    write32(REG(USART3->CR3), USART_CR3_DMAR);
    write32(REG(USART3->CR1), USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);

    write32(REG(BMS_DMA->CCR), 0U);
    write32(REG(BMS_DMA->CPAR), REG(USART3->DR));
    write32(REG(BMS_DMA->CMAR), BMS_DMA_ADDRESS);
    write32(REG(BMS_DMA->CNDTR), length);
    write32(REG(BMS_DMA->CCR), DMA_CCR_MINC | DMA_CCR_PL_VHIGH | DMA_CCR_EN);
}

static void bms_put(uint8_t byte)
{
    while ((read32(REG(USART3->SR)) & USART_SR_TXE) == 0U) {
        /* previous byte still going */
    }
    write32(REG(USART3->DR), byte);
}

void stm32f1_bms_request(const uint8_t *request, uint32_t count, uint16_t reply_length)
{
    clear_bits(REG(BMS_DMA->CCR), DMA_CCR_EN);
    write32(REG(BMS_DMA->CNDTR), reply_length);
    set_bits(REG(BMS_DMA->CCR), DMA_CCR_EN);
    for (uint32_t i = 0U; i < count; i++) {
        bms_put(request[i]);
    }
    while ((read32(REG(USART3->SR)) & USART_SR_TC) == 0U) {
        /* last byte out */
    }
}

uint16_t stm32f1_bms_bytes_received(uint16_t reply_length)
{
    const uint32_t left = read32(REG(BMS_DMA->CNDTR)) & 0xFFFFU;
    const uint16_t received = (left >= reply_length) ? 0U : (uint16_t)(reply_length - left);
    /* The firmware reads its buffer once this count says the reply is in: bring what the DMA wrote. */
    if ((bms_buffer != NULL) && (received > 0U)) {
        sil_load(BMS_DMA_ADDRESS, bms_buffer, (received < bms_buffer_bytes) ? received : bms_buffer_bytes);
    }
    return received;
}

/* IMU, display, LED strip, buzzer (ports/stm32f1/peripherals.c) ---------------------------------------- */

#define IMU_CS_PIN     15U
#define SPI_WAIT_LIMIT 200U
#define IMU_READ       0x80U
#define IMU_WHO_AM_I   0x75U
#define IMU_ID_MPU6500 0x70U
#define IMU_DATA_FIRST 0x3BU
#define IMU_DATA_BYTES 14U

struct imu_write {
    uint8_t reg;
    uint8_t value;
    uint8_t wait_ms;
};

static uint8_t spi_transfer(uint8_t out)
{
    uint32_t waits = 0U;
    while ((read32(REG(SPI3->SR)) & SPI_SR_TXE) == 0U) {
        waits++;
        if (waits > SPI_WAIT_LIMIT) {
            return 0U;
        }
    }
    write32(REG(SPI3->DR), out);
    waits = 0U;
    while ((read32(REG(SPI3->SR)) & SPI_SR_RXNE) == 0U) {
        waits++;
        if (waits > SPI_WAIT_LIMIT) {
            return 0U;
        }
    }
    return (uint8_t)(read32(REG(SPI3->DR)) & 0xFFU);
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
    set_bits(REG(RCC->APB1ENR), RCC_APB1ENR_SPI3);
    stm32f1_gpio_mode(GPIOB, 3U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOB, 4U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOB, 5U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_release_jtag();
    write32(REG(SPI3->CR1),
            SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_MSTR | SPI_CR1_BR_DIV64 | SPI_CR1_SSI | SPI_CR1_SSM);
    set_bits(REG(SPI3->CR1), SPI_CR1_SPE);
    (void)spi_transfer(0xFFU);
    stm32f1_gpio_mode(GPIOA, IMU_CS_PIN, STM32F1_PIN_OUTPUT_50MHZ);
    imu_select(false);
}

bool stm32f1_imu_init(uint8_t gyro_config)
{
    const struct imu_write imu_setup[] = {
        {0x6BU, 0x80U, 200U},      {0x68U, 0x07U, 100U}, {0x6BU, 0x01U, 20U},
        {0x6AU, 0x10U, 20U},       {0x19U, 0x01U, 20U},  {0x1AU, 0x20U, 20U},
        {0x1BU, gyro_config, 20U}, {0x1CU, 0x08U, 20U},  {0x1DU, 0x00U, 20U},
    };
    for (uint32_t i = 0U; i < (sizeof(imu_setup) / sizeof(imu_setup[0])); i++) {
        imu_write(imu_setup[i].reg, imu_setup[i].value);
        stm32f1_delay_ms(imu_setup[i].wait_ms);
    }
    while ((read32(REG(SPI3->SR)) & SPI_SR_TXE) == 0U) {
        /* idle before the speed change */
    }
    clear_bits(REG(SPI3->CR1), SPI_CR1_BR_MASK | SPI_CR1_SPE);
    set_bits(REG(SPI3->CR1), SPI_CR1_SPE);
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
        const uint32_t high = 2U * i;
        const uint16_t word = (uint16_t)(((uint16_t)bytes[high] << 8U) | (uint16_t)bytes[high + 1U]);
        words[i] = (word > (uint16_t)INT16_MAX) ? (int16_t)((int32_t)word - 65536) : (int16_t)word;
    }
}

#define DISPLAY_DIN 4U
#define DISPLAY_CLK 5U
/* The target's pause: 20 turns of a volatile counter loop, about 6 instructions each at 2 cycles. */
#define DISPLAY_PAUSE_CYCLES   240U
#define DISPLAY_AUTO_INCREMENT 0x40U
#define DISPLAY_ON_14_16       0x8FU
#define DISPLAY_FIRST_ADDRESS  0xC0U
#define DISPLAY_GRIDS          16U

static void display_line(uint32_t pin, bool high)
{
    stm32f1_gpio_write(GPIOC, pin, high);
    sil_spin(DISPLAY_PAUSE_CYCLES);
}

static void display_start(void)
{
    display_line(DISPLAY_DIN, true);
    display_line(DISPLAY_CLK, true);
    display_line(DISPLAY_DIN, false);
    display_line(DISPLAY_CLK, false);
}

static void display_stop(void)
{
    display_line(DISPLAY_DIN, false);
    display_line(DISPLAY_CLK, true);
    display_line(DISPLAY_DIN, true);
    display_line(DISPLAY_CLK, false);
}

static void display_byte(uint8_t value)
{
    for (uint32_t bit = 0U; bit < 8U; bit++) {
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

#define LED_BITS_PER_LED 24U
#define LED_RESET_SLOTS  1000U
#define LED_SLOTS        ((STM32F1_LED_COUNT * LED_BITS_PER_LED) + LED_RESET_SLOTS)
#define LED_ONE          20U
#define LED_ZERO         8U
#define LED_TIMER_PSC    2U
#define LED_TIMER_ARR    29U
#define LED_DMA          (&DMA1->CH[1]) /* channel 2: TIM3 CC3 request */
#define LED_SETTLE_MS    10U
/* The bit slots the DMA sends, in emulated SRAM after the BMS buffer. */
#define LED_DMA_ADDRESS (EUC_SIL_DMA_RAM_ADDRESS + 0x100U)

_Static_assert(EUC_BMS_REPLY_BYTES <= 0x100U, "the BMS reply buffer must end before the LED slots");

void stm32f1_led_strip_init(void)
{
    static uint8_t slots[LED_SLOTS];
    set_bits(REG(RCC->APB1ENR), RCC_APB1ENR_TIM3);
    set_bits(REG(RCC->AHBENR), RCC_AHBENR_DMA1);
    stm32f1_gpio_mode(GPIOA, 6U, STM32F1_PIN_ALTERNATE_50MHZ);
    for (uint32_t i = 0U; i < LED_SLOTS; i++) {
        slots[i] = (i < (STM32F1_LED_COUNT * LED_BITS_PER_LED)) ? (uint8_t)LED_ZERO : 0U;
    }
    sil_store(LED_DMA_ADDRESS, slots, LED_SLOTS);

    write32(REG(TIM3->PSC), LED_TIMER_PSC);
    write32(REG(TIM3->ARR), LED_TIMER_ARR);
    write32(REG(TIM3->CCMR1), 0x0068U);
    write32(REG(TIM3->CCMR2), 0U);
    write32(REG(TIM3->CCR1), 0U);
    write32(REG(TIM3->CCR3), 1U);
    write32(REG(TIM3->CCER), TIM_CCER_CC1E);
    write32(REG(TIM3->DIER), TIM_DIER_CC3DE);
    stm32f1_delay_ms(LED_SETTLE_MS);

    write32(REG(LED_DMA->CCR), 0U);
    write32(REG(LED_DMA->CPAR), REG(TIM3->CCR1));
    write32(REG(LED_DMA->CMAR), LED_DMA_ADDRESS);
    write32(REG(LED_DMA->CNDTR), LED_SLOTS);
    write32(REG(LED_DMA->CCR), DMA_CCR_DIR_TO_PER | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_16 |
                                   DMA_CCR_PL_HIGH | DMA_CCR_EN);
    write32(REG(TIM3->CR1), TIM_CR1_ARPE | TIM_CR1_CEN);
}

void stm32f1_led_strip_set(uint32_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= STM32F1_LED_COUNT) {
        return;
    }
    uint8_t slots[LED_BITS_PER_LED];
    const uint32_t grb = ((uint32_t)green << 16U) | ((uint32_t)red << 8U) | (uint32_t)blue;
    for (uint32_t bit = 0U; bit < LED_BITS_PER_LED; bit++) {
        const bool one = (grb & ((uint32_t)1U << (LED_BITS_PER_LED - 1U - bit))) != 0U;
        slots[bit] = one ? (uint8_t)LED_ONE : (uint8_t)LED_ZERO;
    }
    sil_store(LED_DMA_ADDRESS + (index * LED_BITS_PER_LED), slots, LED_BITS_PER_LED);
}

#define BUZZER_PSC 39U
#define BUZZER_ARR 666U

void stm32f1_buzzer_init(void)
{
    set_bits(REG(RCC->APB1ENR), RCC_APB1ENR_TIM4);
    stm32f1_gpio_mode(GPIOB, 9U, STM32F1_PIN_ALTERNATE_10MHZ);
    write32(REG(TIM4->PSC), BUZZER_PSC);
    write32(REG(TIM4->ARR), BUZZER_ARR);
    write32(REG(TIM4->CCMR2), 0x6800U);
    write32(REG(TIM4->CCR4), 0U);
    write32(REG(TIM4->CCER), 0x1000U);
    write32(REG(TIM4->CR1), TIM_CR1_ARPE | TIM_CR1_CEN);
}

void stm32f1_buzzer_set(uint16_t compare)
{
    write32(REG(TIM4->CCR4), compare);
}
