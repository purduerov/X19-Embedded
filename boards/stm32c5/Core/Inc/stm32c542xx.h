/**
 * @file stm32c542xx.h
 * @brief CMSIS Device Peripheral Access Layer for STM32C542xx MCU.
 * @organization Purdue ROV
 */

#ifndef STM32C542XX_H
#define STM32C542XX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define __CM33_REV             0x0000U
#define __NVIC_PRIO_BITS       3U
#define __Vendor_SysTickConfig 0U
#define __FPU_PRESENT          1U
#define __FPU_USED             1U
#define __DSP_PRESENT          1U

typedef enum {
    Reset_IRQn = -15,
    NonMaskableInt_IRQn = -14,
    HardFault_IRQn = -13,
    MemoryManagement_IRQn = -12,
    BusFault_IRQn = -11,
    UsageFault_IRQn = -10,
    SecureFault_IRQn = -9,
    SVCall_IRQn = -5,
    DebugMonitor_IRQn = -4,
    PendSV_IRQn = -2,
    SysTick_IRQn = -1,
    WWDG_IRQn = 0,
    PVD_IRQn = 1,
    RTC_IRQn = 2,
    TAMP_IRQn = 3,
    FLASH_IRQn = 4,
    RCC_IRQn = 5,
    EXTI0_IRQn = 6,
    EXTI1_IRQn = 7,
    EXTI2_IRQn = 8,
    EXTI3_IRQn = 9,
    EXTI4_IRQn = 10,
    DMA1_Channel1_IRQn = 11,
    DMA1_Channel2_IRQn = 12,
    DMA1_Channel3_IRQn = 13,
    DMA1_Channel4_IRQn = 14,
    DMA1_Channel5_IRQn = 15,
    ADC1_IRQn = 16,
    FDCAN1_IT0_IRQn = 17,
    FDCAN1_IT1_IRQn = 18,
    I2C1_EV_IRQn = 19,
    I2C1_ER_IRQn = 20,
    SPI1_IRQn = 21,
    USART1_IRQn = 22,
    TIM1_BRK_UP_TRG_COM_IRQn = 23,
    TIM1_CC_IRQn = 24,
    TIM2_IRQn = 25,
    TIM3_IRQn = 26,
} IRQn_Type;

/* Core Cortex-M33 register definitions */
typedef struct {
    volatile uint32_t CPUID;
    volatile uint32_t ICSR;
    volatile uint32_t VTOR;
    volatile uint32_t AIRCR;
    volatile uint32_t SCR;
    volatile uint32_t CCR;
    volatile uint32_t SHPR[3];
    volatile uint32_t SHCSR;
    volatile uint32_t CFSR;
    volatile uint32_t HFSR;
    volatile uint32_t DFSR;
    volatile uint32_t MMFAR;
    volatile uint32_t BFAR;
    volatile uint32_t AFSR;
    uint32_t RESERVED0[18];
    volatile uint32_t CPACR;
} SCB_Type;

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
} SysTick_Type;

#define SCS_BASE     (0xE000E000UL)
#define SysTick_BASE (SCS_BASE + 0x0010UL)
#define SCB_BASE     (SCS_BASE + 0x0D00UL)

#define SCB     ((SCB_Type *)SCB_BASE)
#define SysTick ((SysTick_Type *)SysTick_BASE)

/* Peripheral Memory Map */
#define PERIPH_BASE     (0x40000000UL)
#define APB1PERIPH_BASE (PERIPH_BASE)
#define APB2PERIPH_BASE (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE (0x42020000UL)

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
    volatile uint32_t BRR;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t TIMINGR;
    volatile uint32_t TIMEOUTR;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t PECR;
    volatile uint32_t RXDR;
    volatile uint32_t TXDR;
} I2C_TypeDef;

typedef struct {
    volatile uint32_t CREL;
    volatile uint32_t ENDN;
    volatile uint32_t DBTP;
    volatile uint32_t TEST;
    volatile uint32_t RWD;
    volatile uint32_t CCCR;
    volatile uint32_t NBTP;
    volatile uint32_t TSCC;
    volatile uint32_t TSCV;
    volatile uint32_t TOCC;
    volatile uint32_t TOCV;
} FDCAN_TypeDef;

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t ICSCR;
    volatile uint32_t CFGR;
    volatile uint32_t PLLCFGR;
} RCC_TypeDef;

#define GPIOA_BASE (AHB2PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE (AHB2PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE (AHB2PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE (AHB2PERIPH_BASE + 0x0C00UL)

#define I2C1_BASE   (APB1PERIPH_BASE + 0x5400UL)
#define FDCAN1_BASE (APB1PERIPH_BASE + 0xA400UL)
#define RCC_BASE    (AHB1PERIPH_BASE + 0x1000UL)

#define GPIOA  ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB  ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC  ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD  ((GPIO_TypeDef *)GPIOD_BASE)
#define I2C1   ((I2C_TypeDef *)I2C1_BASE)
#define FDCAN1 ((FDCAN_TypeDef *)FDCAN1_BASE)
#define RCC    ((RCC_TypeDef *)RCC_BASE)

#ifdef __cplusplus
}
#endif

#endif /* STM32C542XX_H */
