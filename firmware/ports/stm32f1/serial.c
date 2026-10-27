/*
 * Serial ports (RM0008 27, DMA 13): USART1 to the BLE module, USART3 to the BMS.
 */
#include <stdbool.h>
#include <stdint.h>

#include "stm32f1/device.h"
#include "stm32f1/port.h"
#include "stm32f1/regs.h"

#define BLE_BRR_115200 625U  /* 72 MHz / 115200 */
#define BMS_BRR_9600   3750U /* 36 MHz / 9600 */
#if defined(EUC_FEATURE_IMU_TLM)
#define TX_RING_BYTES 512U /* power of two; the IMU frames ride next to the stock groups */
#else
#define TX_RING_BYTES 256U /* power of two */
#endif

/* USART1 transmit ring: the main loop writes the head, the interrupt reads the tail. */
static uint8_t tx_ring[TX_RING_BYTES];
static volatile uint32_t tx_head;
static volatile uint32_t tx_tail;

void stm32f1_ble_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1;
    stm32f1_gpio_mode(GPIOA, 9U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOA, 10U, STM32F1_PIN_INPUT_PULL);
    stm32f1_gpio_pull(GPIOA, 10U, true);
    USART1->BRR = BLE_BRR_115200;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
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
    USART1->CR1 |= USART_CR1_TXEIE;
    return true;
}

uint32_t stm32f1_ble_free(void)
{
    return TX_RING_BYTES - (tx_head - tx_tail);
}

bool stm32f1_ble_service(uint8_t *received)
{
    const uint32_t status = USART1->SR;
    bool got = false;
    if ((status & (USART_SR_RXNE | USART_SR_ORE)) != 0U) {
        *received = (uint8_t)(USART1->DR & 0xFFU); /* reading DR also clears an overrun */
        got = (status & USART_SR_RXNE) != 0U;
    }
    if (((USART1->CR1 & USART_CR1_TXEIE) != 0U) && ((status & USART_SR_TXE) != 0U)) {
        if (tx_tail != tx_head) {
            USART1->DR = tx_ring[tx_tail % TX_RING_BYTES];
            tx_tail++;
        } else {
            USART1->CR1 &= ~USART_CR1_TXEIE;
        }
    }
    return got;
}

/* USART3 + DMA1 channel 3 ------------------------------------------------------------------------------ */

#define BMS_DMA (&DMA1->CH[2])

/* The DMA writes the buffer, so it is not const. */
/* cppcheck-suppress constParameterPointer */
void stm32f1_bms_init(uint8_t *buffer, uint16_t length) /* NOLINT(readability-non-const-parameter) */
{
    RCC->APB1ENR |= RCC_APB1ENR_USART3;
    RCC->AHBENR |= RCC_AHBENR_DMA1;
    stm32f1_gpio_mode(GPIOB, 10U, STM32F1_PIN_ALTERNATE_50MHZ);
    stm32f1_gpio_mode(GPIOB, 11U, STM32F1_PIN_INPUT_FLOATING);
    USART3->BRR = BMS_BRR_9600;
    USART3->CR2 = 0U;
    USART3->CR3 = USART_CR3_DMAR;
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    BMS_DMA->CCR = 0U;
    BMS_DMA->CPAR = (uint32_t)(uintptr_t)&USART3->DR;
    BMS_DMA->CMAR = (uint32_t)(uintptr_t)buffer;
    BMS_DMA->CNDTR = length;
    BMS_DMA->CCR = DMA_CCR_MINC | DMA_CCR_PL_VHIGH | DMA_CCR_EN;
}

static void bms_put(uint8_t byte)
{
    while ((USART3->SR & USART_SR_TXE) == 0U) {
        /* previous byte still going */
    }
    USART3->DR = byte;
}

void stm32f1_bms_request(const uint8_t *request, uint32_t count, uint16_t reply_length)
{
    BMS_DMA->CCR &= ~DMA_CCR_EN;
    BMS_DMA->CNDTR = reply_length;
    BMS_DMA->CCR |= DMA_CCR_EN;
    for (uint32_t i = 0U; i < count; i++) {
        bms_put(request[i]);
    }
    while ((USART3->SR & USART_SR_TC) == 0U) {
        /* last byte out */
    }
}

uint16_t stm32f1_bms_bytes_received(uint16_t reply_length)
{
    const uint32_t left = BMS_DMA->CNDTR & 0xFFFFU;
    return (left >= reply_length) ? 0U : (uint16_t)(reply_length - left);
}
