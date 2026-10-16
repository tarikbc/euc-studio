/*
 * STM32F1 peripheral registers the port drives, written from the reference manual RM0008 (register maps of
 * each chapter). Every register is 32 bits wide in the map; 16-bit registers use the low half.
 */
#ifndef STM32F1_REGS_H
#define STM32F1_REGS_H

#include <stdint.h>

/* RCC (RM0008 7.3) ------------------------------------------------------------------------------------- */
struct stm32f1_rcc {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
};

#define RCC_CR_HSEON  0x00010000U
#define RCC_CR_HSERDY 0x00020000U
#define RCC_CR_PLLON  0x01000000U
#define RCC_CR_PLLRDY 0x02000000U

#define RCC_CFGR_SW_MASK     0x00000003U
#define RCC_CFGR_SW_PLL      0x00000002U
#define RCC_CFGR_SWS_MASK    0x0000000CU
#define RCC_CFGR_SWS_PLL     0x00000008U
#define RCC_CFGR_HPRE_MASK   0x000000F0U
#define RCC_CFGR_PPRE1_MASK  0x00000700U
#define RCC_CFGR_PPRE1_DIV2  0x00000400U
#define RCC_CFGR_PPRE2_MASK  0x00003800U
#define RCC_CFGR_ADC_MASK    0x0000C000U
#define RCC_CFGR_ADC_DIV6    0x00008000U
#define RCC_CFGR_PLLSRC_HSE  0x00010000U
#define RCC_CFGR_PLLXTPRE    0x00020000U
#define RCC_CFGR_PLLMUL_MASK 0x003C0000U
#define RCC_CFGR_PLLMUL9     0x001C0000U

#define RCC_AHBENR_DMA1 0x00000001U

#define RCC_APB2ENR_AFIO   0x00000001U
#define RCC_APB2ENR_GPIOA  0x00000004U
#define RCC_APB2ENR_GPIOB  0x00000008U
#define RCC_APB2ENR_GPIOC  0x00000010U
#define RCC_APB2ENR_ADC1   0x00000200U
#define RCC_APB2ENR_ADC2   0x00000400U
#define RCC_APB2ENR_TIM8   0x00002000U
#define RCC_APB2ENR_USART1 0x00004000U

#define RCC_APB1ENR_TIM3   0x00000002U
#define RCC_APB1ENR_TIM4   0x00000004U
#define RCC_APB1ENR_SPI3   0x00008000U
#define RCC_APB1ENR_USART3 0x00040000U
#define RCC_APB1ENR_BKP    0x08000000U
#define RCC_APB1ENR_PWR    0x10000000U

#define RCC_BDCR_LSEON       0x00000001U
#define RCC_BDCR_RTCSEL_MASK 0x00000300U
#define RCC_BDCR_RTCSEL_HSE  0x00000300U /* HSE / 128 */
#define RCC_BDCR_RTCEN       0x00008000U

/* FLASH interface (RM0008 3.3.3, PM0075) -------------------------------------------------------------- */
struct stm32f1_flash {
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t AR;
    volatile uint32_t RESERVED;
    volatile uint32_t OBR;
    volatile uint32_t WRPR;
};

#define FLASH_ACR_LATENCY_2 0x00000002U
#define FLASH_ACR_PRFTBE    0x00000010U
#define FLASH_SR_BSY        0x00000001U
#define FLASH_SR_PGERR      0x00000004U
#define FLASH_SR_WRPRTERR   0x00000010U
#define FLASH_SR_EOP        0x00000020U
#define FLASH_CR_PG         0x00000001U
#define FLASH_CR_PER        0x00000002U
#define FLASH_CR_STRT       0x00000040U
#define FLASH_CR_LOCK       0x00000080U
#define FLASH_KEY1          0x45670123U
#define FLASH_KEY2          0xCDEF89ABU

/* GPIO and AFIO (RM0008 9.2, 9.4) ----------------------------------------------------------------------- */
struct stm32f1_gpio {
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
};

struct stm32f1_afio {
    volatile uint32_t EVCR;
    volatile uint32_t MAPR;
    volatile uint32_t EXTICR[4];
    volatile uint32_t RESERVED;
    volatile uint32_t MAPR2;
};

#define AFIO_MAPR_SWJ_MASK   0x07000000U
#define AFIO_MAPR_SWJ_SWONLY 0x02000000U /* JTAG off, SWD on: frees PB3, PB4, PA15 */

/* Timers (RM0008 14, 15) -------------------------------------------------------------------------------- */
struct stm32f1_tim {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
};

#define TIM_CR1_CEN         0x00000001U
#define TIM_CR1_DIR         0x00000010U
#define TIM_CR1_CMS_CENTER1 0x00000020U
#define TIM_CR1_ARPE        0x00000080U
#define TIM_DIER_UIE        0x00000001U
#define TIM_DIER_CC3DE      0x00000800U
#define TIM_SR_UIF          0x00000001U
#define TIM_EGR_UG          0x00000001U
#define TIM_CCER_CC1E       0x00000001U
#define TIM_BDTR_AOE        0x00004000U
#define TIM_BDTR_MOE        0x00008000U

/* ADC (RM0008 11.12) ------------------------------------------------------------------------------------- */
struct stm32f1_adc {
    volatile uint32_t SR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t JOFR[4];
    volatile uint32_t HTR;
    volatile uint32_t LTR;
    volatile uint32_t SQR1;
    volatile uint32_t SQR2;
    volatile uint32_t SQR3;
    volatile uint32_t JSQR;
    volatile uint32_t JDR[4];
    volatile uint32_t DR;
};

#define ADC_SR_JEOC          0x00000004U
#define ADC_CR1_JEOCIE       0x00000080U
#define ADC_CR1_SCAN         0x00000100U
#define ADC_CR1_DUAL_INJ_SIM 0x00050000U
#define ADC_CR2_ADON         0x00000001U
#define ADC_CR2_CAL          0x00000004U
#define ADC_CR2_RSTCAL       0x00000008U
#define ADC_CR2_ALIGN_LEFT   0x00000800U
#define ADC_CR2_JEXTSEL_SW   0x00007000U
#define ADC_CR2_JEXTTRIG     0x00008000U
#define ADC_CR2_EXTSEL_SW    0x000E0000U
#define ADC_CR2_JSWSTART     0x00200000U
#define ADC_SAMPLE_71_5      6U

/* USART (RM0008 27.6) ------------------------------------------------------------------------------------ */
struct stm32f1_usart {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
};

#define USART_SR_ORE     0x00000008U
#define USART_SR_RXNE    0x00000020U
#define USART_SR_TC      0x00000040U
#define USART_SR_TXE     0x00000080U
#define USART_CR1_RE     0x00000004U
#define USART_CR1_TE     0x00000008U
#define USART_CR1_RXNEIE 0x00000020U
#define USART_CR1_TXEIE  0x00000080U
#define USART_CR1_UE     0x00002000U
#define USART_CR3_DMAR   0x00000040U

/* DMA (RM0008 13.4) -------------------------------------------------------------------------------------- */
struct stm32f1_dma_channel {
    volatile uint32_t CCR;
    volatile uint32_t CNDTR;
    volatile uint32_t CPAR;
    volatile uint32_t CMAR;
    volatile uint32_t RESERVED;
};

struct stm32f1_dma {
    volatile uint32_t ISR;
    volatile uint32_t IFCR;
    struct stm32f1_dma_channel CH[7]; /* CH[0] = channel 1 */
};

#define DMA_CCR_EN         0x00000001U
#define DMA_CCR_DIR_TO_PER 0x00000010U
#define DMA_CCR_CIRC       0x00000020U
#define DMA_CCR_MINC       0x00000080U
#define DMA_CCR_PSIZE_16   0x00000100U
#define DMA_CCR_PL_HIGH    0x00002000U
#define DMA_CCR_PL_VHIGH   0x00003000U

/* SPI (RM0008 25.5) -------------------------------------------------------------------------------------- */
struct stm32f1_spi {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
};

#define SPI_CR1_CPHA     0x00000001U
#define SPI_CR1_CPOL     0x00000002U
#define SPI_CR1_MSTR     0x00000004U
#define SPI_CR1_BR_MASK  0x00000038U
#define SPI_CR1_BR_DIV64 0x00000028U
#define SPI_CR1_SPE      0x00000040U
#define SPI_CR1_SSI      0x00000100U
#define SPI_CR1_SSM      0x00000200U
#define SPI_SR_RXNE      0x00000001U
#define SPI_SR_TXE       0x00000002U

/* RTC, BKP, PWR (RM0008 18.4, 6.4, 5.4) ------------------------------------------------------------------ */
struct stm32f1_rtc {
    volatile uint32_t CRH;
    volatile uint32_t CRL;
    volatile uint32_t PRLH;
    volatile uint32_t PRLL;
    volatile uint32_t DIVH;
    volatile uint32_t DIVL;
    volatile uint32_t CNTH;
    volatile uint32_t CNTL;
    volatile uint32_t ALRH;
    volatile uint32_t ALRL;
};

#define RTC_CRH_SECIE 0x00000001U
#define RTC_CRL_SECF  0x00000001U
#define RTC_CRL_RSF   0x00000008U
#define RTC_CRL_CNF   0x00000010U
#define RTC_CRL_RTOFF 0x00000020U

#define BKP_BASE_ADDRESS 0x40006C00U
#define PWR_CR_DBP       0x00000100U

struct stm32f1_pwr {
    volatile uint32_t CR;
    volatile uint32_t CSR;
};

/* IWDG (RM0008 19.4) ------------------------------------------------------------------------------------- */
struct stm32f1_iwdg {
    volatile uint32_t KR;
    volatile uint32_t PR;
    volatile uint32_t RLR;
    volatile uint32_t SR;
};

/* SysTick (PM0056 4.5); the port polls it for delays. */
struct stm32f1_systick {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
};

#define SYSTICK_CTRL_ENABLE    0x00000001U
#define SYSTICK_CTRL_COUNTFLAG 0x00010000U

/* Instances ------------------------------------------------------------------------------------------- */
#define RCC     ((struct stm32f1_rcc *)0x40021000UL)
#define FLASH   ((struct stm32f1_flash *)0x40022000UL)
#define AFIO    ((struct stm32f1_afio *)0x40010000UL)
#define GPIOA   ((struct stm32f1_gpio *)0x40010800UL)
#define GPIOB   ((struct stm32f1_gpio *)0x40010C00UL)
#define GPIOC   ((struct stm32f1_gpio *)0x40011000UL)
#define TIM3    ((struct stm32f1_tim *)0x40000400UL)
#define TIM4    ((struct stm32f1_tim *)0x40000800UL)
#define TIM8    ((struct stm32f1_tim *)0x40013400UL)
#define ADC1    ((struct stm32f1_adc *)0x40012400UL)
#define ADC2    ((struct stm32f1_adc *)0x40012800UL)
#define USART1  ((struct stm32f1_usart *)0x40013800UL)
#define USART3  ((struct stm32f1_usart *)0x40004800UL)
#define DMA1    ((struct stm32f1_dma *)0x40020000UL)
#define SPI3    ((struct stm32f1_spi *)0x40003C00UL)
#define RTC     ((struct stm32f1_rtc *)0x40002800UL)
#define PWR     ((struct stm32f1_pwr *)0x40007000UL)
#define IWDG    ((struct stm32f1_iwdg *)0x40003000UL)
#define SYSTICK ((struct stm32f1_systick *)0xE000E010UL)

#endif
