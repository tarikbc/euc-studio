/*
 * GPIO (RM0008 9.2): four configuration bits per pin, CRL for pins 0..7 and CRH for 8..15.
 */
#include <stdbool.h>
#include <stdint.h>

#include "stm32f1/port.h"
#include "stm32f1/regs.h"

#define PIN_CONFIG_BITS   4U
#define PIN_CONFIG_MASK   0xFU
#define PINS_PER_REGISTER 8U

void stm32f1_gpio_clocks_on(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_AFIO | RCC_APB2ENR_GPIOA | RCC_APB2ENR_GPIOB | RCC_APB2ENR_GPIOC;
}

void stm32f1_gpio_mode(struct stm32f1_gpio *port, uint32_t pin, enum stm32f1_pin_mode mode)
{
    volatile uint32_t *config = (pin < PINS_PER_REGISTER) ? &port->CRL : &port->CRH;
    const uint32_t shift = (pin % PINS_PER_REGISTER) * PIN_CONFIG_BITS;
    *config = (*config & ~(PIN_CONFIG_MASK << shift)) | ((uint32_t)mode << shift);
}

void stm32f1_gpio_pull(struct stm32f1_gpio *port, uint32_t pin, bool up)
{
    stm32f1_gpio_write(port, pin, up);
}

void stm32f1_gpio_write(struct stm32f1_gpio *port, uint32_t pin, bool high)
{
    if (high) {
        port->BSRR = (uint32_t)1U << pin;
    } else {
        port->BRR = (uint32_t)1U << pin;
    }
}

bool stm32f1_gpio_read(const struct stm32f1_gpio *port, uint32_t pin)
{
    return (port->IDR & ((uint32_t)1U << pin)) != 0U;
}

void stm32f1_gpio_release_jtag(void)
{
    AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_SWJ_MASK) | AFIO_MAPR_SWJ_SWONLY;
}
