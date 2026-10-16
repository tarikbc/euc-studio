/*
 * Reset path and vector table for STM32F1 high-density parts.
 *
 * The vendor bootloader at 0x08000000 jumps to the image through its first two words (initial SP, reset
 * vector), so the vector table must be the first thing in the image. Every handler is a weak alias of
 * Default_Handler; a driver takes over an interrupt by defining the handler with the same CMSIS name.
 */
#include <stdint.h>
#include <string.h>

#include "stm32f1/regs.h"
#include "stm32f1/system.h"

/* Symbols defined by the board linker script (boards/<board>/link_app.ld). */
extern const uint8_t ld_data_load[]; /* initial values of .data, in flash */
extern uint8_t ld_data_start[];
extern uint8_t ld_data_end[];
extern uint8_t ld_bss_start[];
extern uint8_t ld_bss_end[];
extern const uint8_t ld_stack_top[]; /* first byte above the main stack */

int main(void);

void Reset_Handler(void);
static void Default_Handler(void);

#define STM32F1_WEAK_HANDLER(name) void name(void) __attribute__((weak, alias("Default_Handler")))

STM32F1_WEAK_HANDLER(NMI_Handler);
STM32F1_WEAK_HANDLER(HardFault_Handler);
STM32F1_WEAK_HANDLER(MemManage_Handler);
STM32F1_WEAK_HANDLER(BusFault_Handler);
STM32F1_WEAK_HANDLER(UsageFault_Handler);
STM32F1_WEAK_HANDLER(SVC_Handler);
STM32F1_WEAK_HANDLER(DebugMon_Handler);
STM32F1_WEAK_HANDLER(PendSV_Handler);
STM32F1_WEAK_HANDLER(SysTick_Handler);

STM32F1_WEAK_HANDLER(WWDG_IRQHandler);
STM32F1_WEAK_HANDLER(PVD_IRQHandler);
STM32F1_WEAK_HANDLER(TAMPER_IRQHandler);
STM32F1_WEAK_HANDLER(RTC_IRQHandler);
STM32F1_WEAK_HANDLER(FLASH_IRQHandler);
STM32F1_WEAK_HANDLER(RCC_IRQHandler);
STM32F1_WEAK_HANDLER(EXTI0_IRQHandler);
STM32F1_WEAK_HANDLER(EXTI1_IRQHandler);
STM32F1_WEAK_HANDLER(EXTI2_IRQHandler);
STM32F1_WEAK_HANDLER(EXTI3_IRQHandler);
STM32F1_WEAK_HANDLER(EXTI4_IRQHandler);
STM32F1_WEAK_HANDLER(DMA1_Channel1_IRQHandler);
STM32F1_WEAK_HANDLER(DMA1_Channel2_IRQHandler);
STM32F1_WEAK_HANDLER(DMA1_Channel3_IRQHandler);
STM32F1_WEAK_HANDLER(DMA1_Channel4_IRQHandler);
STM32F1_WEAK_HANDLER(DMA1_Channel5_IRQHandler);
STM32F1_WEAK_HANDLER(DMA1_Channel6_IRQHandler);
STM32F1_WEAK_HANDLER(DMA1_Channel7_IRQHandler);
STM32F1_WEAK_HANDLER(ADC1_2_IRQHandler);
STM32F1_WEAK_HANDLER(USB_HP_CAN1_TX_IRQHandler);
STM32F1_WEAK_HANDLER(USB_LP_CAN1_RX0_IRQHandler);
STM32F1_WEAK_HANDLER(CAN1_RX1_IRQHandler);
STM32F1_WEAK_HANDLER(CAN1_SCE_IRQHandler);
STM32F1_WEAK_HANDLER(EXTI9_5_IRQHandler);
STM32F1_WEAK_HANDLER(TIM1_BRK_IRQHandler);
STM32F1_WEAK_HANDLER(TIM1_UP_IRQHandler);
STM32F1_WEAK_HANDLER(TIM1_TRG_COM_IRQHandler);
STM32F1_WEAK_HANDLER(TIM1_CC_IRQHandler);
STM32F1_WEAK_HANDLER(TIM2_IRQHandler);
STM32F1_WEAK_HANDLER(TIM3_IRQHandler);
STM32F1_WEAK_HANDLER(TIM4_IRQHandler);
STM32F1_WEAK_HANDLER(I2C1_EV_IRQHandler);
STM32F1_WEAK_HANDLER(I2C1_ER_IRQHandler);
STM32F1_WEAK_HANDLER(I2C2_EV_IRQHandler);
STM32F1_WEAK_HANDLER(I2C2_ER_IRQHandler);
STM32F1_WEAK_HANDLER(SPI1_IRQHandler);
STM32F1_WEAK_HANDLER(SPI2_IRQHandler);
STM32F1_WEAK_HANDLER(USART1_IRQHandler);
STM32F1_WEAK_HANDLER(USART2_IRQHandler);
STM32F1_WEAK_HANDLER(USART3_IRQHandler);
STM32F1_WEAK_HANDLER(EXTI15_10_IRQHandler);
STM32F1_WEAK_HANDLER(RTC_Alarm_IRQHandler);
STM32F1_WEAK_HANDLER(USBWakeUp_IRQHandler);
STM32F1_WEAK_HANDLER(TIM8_BRK_IRQHandler);
STM32F1_WEAK_HANDLER(TIM8_UP_IRQHandler);
STM32F1_WEAK_HANDLER(TIM8_TRG_COM_IRQHandler);
STM32F1_WEAK_HANDLER(TIM8_CC_IRQHandler);
STM32F1_WEAK_HANDLER(ADC3_IRQHandler);
STM32F1_WEAK_HANDLER(FSMC_IRQHandler);
STM32F1_WEAK_HANDLER(SDIO_IRQHandler);
STM32F1_WEAK_HANDLER(TIM5_IRQHandler);
STM32F1_WEAK_HANDLER(SPI3_IRQHandler);
STM32F1_WEAK_HANDLER(UART4_IRQHandler);
STM32F1_WEAK_HANDLER(UART5_IRQHandler);
STM32F1_WEAK_HANDLER(TIM6_IRQHandler);
STM32F1_WEAK_HANDLER(TIM7_IRQHandler);
STM32F1_WEAK_HANDLER(DMA2_Channel1_IRQHandler);
STM32F1_WEAK_HANDLER(DMA2_Channel2_IRQHandler);
STM32F1_WEAK_HANDLER(DMA2_Channel3_IRQHandler);
STM32F1_WEAK_HANDLER(DMA2_Channel4_5_IRQHandler);

__attribute__((section(".isr_vector"), used)) const struct stm32f1_vector_table stm32f1_vector_table = {
    .initial_stack_pointer = ld_stack_top,
    .reset = Reset_Handler,
    .nmi = NMI_Handler,
    .hard_fault = HardFault_Handler,
    .mem_manage = MemManage_Handler,
    .bus_fault = BusFault_Handler,
    .usage_fault = UsageFault_Handler,
    .sv_call = SVC_Handler,
    .debug_monitor = DebugMon_Handler,
    .pend_sv = PendSV_Handler,
    .sys_tick = SysTick_Handler,
    .irq =
        {
            [WWDG_IRQn] = WWDG_IRQHandler,
            [PVD_IRQn] = PVD_IRQHandler,
            [TAMPER_IRQn] = TAMPER_IRQHandler,
            [RTC_IRQn] = RTC_IRQHandler,
            [FLASH_IRQn] = FLASH_IRQHandler,
            [RCC_IRQn] = RCC_IRQHandler,
            [EXTI0_IRQn] = EXTI0_IRQHandler,
            [EXTI1_IRQn] = EXTI1_IRQHandler,
            [EXTI2_IRQn] = EXTI2_IRQHandler,
            [EXTI3_IRQn] = EXTI3_IRQHandler,
            [EXTI4_IRQn] = EXTI4_IRQHandler,
            [DMA1_Channel1_IRQn] = DMA1_Channel1_IRQHandler,
            [DMA1_Channel2_IRQn] = DMA1_Channel2_IRQHandler,
            [DMA1_Channel3_IRQn] = DMA1_Channel3_IRQHandler,
            [DMA1_Channel4_IRQn] = DMA1_Channel4_IRQHandler,
            [DMA1_Channel5_IRQn] = DMA1_Channel5_IRQHandler,
            [DMA1_Channel6_IRQn] = DMA1_Channel6_IRQHandler,
            [DMA1_Channel7_IRQn] = DMA1_Channel7_IRQHandler,
            [ADC1_2_IRQn] = ADC1_2_IRQHandler,
            [USB_HP_CAN1_TX_IRQn] = USB_HP_CAN1_TX_IRQHandler,
            [USB_LP_CAN1_RX0_IRQn] = USB_LP_CAN1_RX0_IRQHandler,
            [CAN1_RX1_IRQn] = CAN1_RX1_IRQHandler,
            [CAN1_SCE_IRQn] = CAN1_SCE_IRQHandler,
            [EXTI9_5_IRQn] = EXTI9_5_IRQHandler,
            [TIM1_BRK_IRQn] = TIM1_BRK_IRQHandler,
            [TIM1_UP_IRQn] = TIM1_UP_IRQHandler,
            [TIM1_TRG_COM_IRQn] = TIM1_TRG_COM_IRQHandler,
            [TIM1_CC_IRQn] = TIM1_CC_IRQHandler,
            [TIM2_IRQn] = TIM2_IRQHandler,
            [TIM3_IRQn] = TIM3_IRQHandler,
            [TIM4_IRQn] = TIM4_IRQHandler,
            [I2C1_EV_IRQn] = I2C1_EV_IRQHandler,
            [I2C1_ER_IRQn] = I2C1_ER_IRQHandler,
            [I2C2_EV_IRQn] = I2C2_EV_IRQHandler,
            [I2C2_ER_IRQn] = I2C2_ER_IRQHandler,
            [SPI1_IRQn] = SPI1_IRQHandler,
            [SPI2_IRQn] = SPI2_IRQHandler,
            [USART1_IRQn] = USART1_IRQHandler,
            [USART2_IRQn] = USART2_IRQHandler,
            [USART3_IRQn] = USART3_IRQHandler,
            [EXTI15_10_IRQn] = EXTI15_10_IRQHandler,
            [RTC_Alarm_IRQn] = RTC_Alarm_IRQHandler,
            [USBWakeUp_IRQn] = USBWakeUp_IRQHandler,
            [TIM8_BRK_IRQn] = TIM8_BRK_IRQHandler,
            [TIM8_UP_IRQn] = TIM8_UP_IRQHandler,
            [TIM8_TRG_COM_IRQn] = TIM8_TRG_COM_IRQHandler,
            [TIM8_CC_IRQn] = TIM8_CC_IRQHandler,
            [ADC3_IRQn] = ADC3_IRQHandler,
            [FSMC_IRQn] = FSMC_IRQHandler,
            [SDIO_IRQn] = SDIO_IRQHandler,
            [TIM5_IRQn] = TIM5_IRQHandler,
            [SPI3_IRQn] = SPI3_IRQHandler,
            [UART4_IRQn] = UART4_IRQHandler,
            [UART5_IRQn] = UART5_IRQHandler,
            [TIM6_IRQn] = TIM6_IRQHandler,
            [TIM7_IRQn] = TIM7_IRQHandler,
            [DMA2_Channel1_IRQn] = DMA2_Channel1_IRQHandler,
            [DMA2_Channel2_IRQn] = DMA2_Channel2_IRQHandler,
            [DMA2_Channel3_IRQn] = DMA2_Channel3_IRQHandler,
            [DMA2_Channel4_5_IRQn] = DMA2_Channel4_5_IRQHandler,
        },
};

/*
 * C runtime set-up, then SystemInit and main. Runs on the stack the core loaded from vector word 0, with no
 * assumption about what the bootloader left behind except that the clock runs.
 */
void Reset_Handler(void)
{
    /* The bounds are separate linker symbols, so subtract addresses, not pointers to different objects. */
    const size_t data_bytes = (size_t)((uintptr_t)ld_data_end - (uintptr_t)ld_data_start);
    const size_t bss_bytes = (size_t)((uintptr_t)ld_bss_end - (uintptr_t)ld_bss_start);

    (void)memcpy(ld_data_start, ld_data_load, data_bytes);
    (void)memset(ld_bss_start, 0, bss_bytes);

    SystemInit();
    (void)main();

    /* main never returns; if it does, stop here rather than run off the end of flash. */
    for (;;) {
    }
}

/*
 * Any exception or interrupt without its own handler, faults included. It turns the motor bridge off first
 * (TIM8 main output enable), in both profiles: the stock handler leaves the last PWM running. Then it parks
 * the core so a debugger can see where it came from (IPSR holds the exception number); the clean profile's
 * watchdog resets it.
 */
static void Default_Handler(void)
{
    TIM8->BDTR &= ~(TIM_BDTR_MOE | TIM_BDTR_AOE);
    for (;;) {
    }
}
