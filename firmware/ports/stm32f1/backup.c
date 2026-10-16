/*
 * RTC tick (RM0008 18), backup registers (RM0008 6), flash programming (PM0075) and the independent watchdog
 * (RM0008 19).
 */
#include <stdbool.h>
#include <stdint.h>

#include "stm32f1/device.h"
#include "stm32f1/port.h"
#include "stm32f1/regs.h"

#define RTC_PRESCALER_RELOAD 125U /* 62.5 kHz / 126 = 496 Hz */

#define BKP_DR1_OFFSET  0x04U
#define BKP_DR11_OFFSET 0x40U
#define BKP_LOW_COUNT   10U

#define FLASH_PAGE_BYTES     2048U
#define FLASH_PAGE_HALFWORDS (FLASH_PAGE_BYTES / 2U)

#define IWDG_KEY_UNLOCK   0x5555U
#define IWDG_KEY_RELOAD   0xAAAAU
#define IWDG_KEY_START    0xCCCCU
#define IWDG_PRESCALER_64 4U /* 40 kHz LSI / 64 = 625 Hz */
#define IWDG_TICKS_PER_S  625U
#define IWDG_MAX_RELOAD   0xFFFU

/* RTC -------------------------------------------------------------------------------------------------------
 */

static void rtc_wait_write_done(void)
{
    while ((RTC->CRL & RTC_CRL_RTOFF) == 0U) {
        /* last write still going */
    }
}

void stm32f1_rtc_tick_init(void)
{
    stm32f1_backup_init();
    RCC->BDCR |= RCC_BDCR_LSEON; /* the stock firmware turns it on, but clocks the RTC from the HSE */
    RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL_MASK) | RCC_BDCR_RTCSEL_HSE | RCC_BDCR_RTCEN;
    RTC->CRL &= ~RTC_CRL_RSF;
    while ((RTC->CRL & RTC_CRL_RSF) == 0U) {
        /* registers synchronised */
    }
    rtc_wait_write_done();
    RTC->CRH |= RTC_CRH_SECIE;
    rtc_wait_write_done();
    RTC->CRL |= RTC_CRL_CNF;
    RTC->PRLH = 0U;
    RTC->PRLL = RTC_PRESCALER_RELOAD;
    RTC->CRL &= ~RTC_CRL_CNF;
    rtc_wait_write_done();
    stm32f1_nvic_enable(RTC_IRQn, 2U, 2U);
}

void stm32f1_rtc_tick_acknowledge(void)
{
    RTC->CRL &= ~RTC_CRL_SECF;
    rtc_wait_write_done();
}

/* Backup registers
 * -------------------------------------------------------------------------------------------- */

void stm32f1_backup_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWR | RCC_APB1ENR_BKP;
    PWR->CR |= PWR_CR_DBP;
}

static volatile uint32_t *backup_register(uint32_t index)
{
    const uint32_t offset = (index <= BKP_LOW_COUNT) ? (BKP_DR1_OFFSET + (4U * (index - 1U)))
                                                     : (BKP_DR11_OFFSET + (4U * (index - 11U)));
    return (volatile uint32_t *)(BKP_BASE_ADDRESS + offset);
}

uint16_t stm32f1_backup_read(uint32_t index)
{
    return (uint16_t)(*backup_register(index) & 0xFFFFU);
}

void stm32f1_backup_write(uint32_t index, uint16_t value)
{
    *backup_register(index) = value;
}

/* Flash ---------------------------------------------------------------------------------------------------
 */

static void flash_unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0U) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

static void flash_wait(void)
{
    while ((FLASH->SR & FLASH_SR_BSY) != 0U) {
        /* busy */
    }
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
}

static void flash_erase_unlocked(uint32_t address)
{
    flash_wait();
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = address;
    FLASH->CR |= FLASH_CR_STRT;
    flash_wait();
    FLASH->CR &= ~FLASH_CR_PER;
}

static void flash_program_unlocked(uint32_t address, const uint16_t *halfwords, uint32_t count)
{
    FLASH->CR |= FLASH_CR_PG;
    for (uint32_t i = 0U; i < count; i++) {
        *(volatile uint16_t *)(address + (2U * i)) = halfwords[i];
        flash_wait();
    }
    FLASH->CR &= ~FLASH_CR_PG;
}

void stm32f1_flash_erase_page(uint32_t address)
{
    flash_unlock();
    flash_erase_unlocked(address);
    FLASH->CR |= FLASH_CR_LOCK;
}

void stm32f1_flash_program(uint32_t address, const uint16_t *halfwords, uint32_t count)
{
    flash_unlock();
    flash_program_unlocked(address, halfwords, count);
    FLASH->CR |= FLASH_CR_LOCK;
}

void stm32f1_flash_rewrite(uint32_t address, const uint16_t *halfwords, uint32_t count)
{
    static uint16_t page[FLASH_PAGE_HALFWORDS];
    const uint32_t page_address = address & ~(FLASH_PAGE_BYTES - 1U);
    const volatile uint16_t *flash_page = (const volatile uint16_t *)page_address;
    for (uint32_t i = 0U; i < FLASH_PAGE_HALFWORDS; i++) {
        page[i] = flash_page[i];
    }
    const uint32_t first = (address - page_address) / 2U;
    for (uint32_t i = 0U; (i < count) && ((first + i) < FLASH_PAGE_HALFWORDS); i++) {
        page[first + i] = halfwords[i];
    }
    flash_unlock();
    flash_erase_unlocked(page_address);
    flash_program_unlocked(page_address, page, FLASH_PAGE_HALFWORDS);
    FLASH->CR |= FLASH_CR_LOCK;
}

/* Watchdog --------------------------------------------------------------------------------------------------
 */

void stm32f1_watchdog_start(uint32_t timeout_ms)
{
    uint32_t reload = (timeout_ms * IWDG_TICKS_PER_S) / 1000U;
    if (reload > IWDG_MAX_RELOAD) {
        reload = IWDG_MAX_RELOAD;
    }
    IWDG->KR = IWDG_KEY_UNLOCK;
    IWDG->PR = IWDG_PRESCALER_64;
    IWDG->RLR = reload;
    IWDG->KR = IWDG_KEY_RELOAD;
    IWDG->KR = IWDG_KEY_START;
}

void stm32f1_watchdog_reload(void)
{
    IWDG->KR = IWDG_KEY_RELOAD;
}

bool stm32f1_reset_was_watchdog(void)
{
    const bool watchdog = (RCC->CSR & RCC_CSR_IWDGRSTF) != 0U;
    RCC->CSR |= RCC_CSR_RMVF;
    return watchdog;
}
