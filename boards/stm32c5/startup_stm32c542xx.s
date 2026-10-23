/**
 * @file startup_stm32c542xx.s
 * @brief STM32C542xx Cortex-M33 vector table and reset handler (GCC toolchain).
 * @organization Purdue ROV
 */

  .syntax unified
  .cpu cortex-m33
  .fpu fpv5-sp-d16
  .thumb

.global g_pfnVectors
.global Default_Handler
.global Reset_Handler

/* Symbols defined in linker script */
.word _sidata
.word _sdata
.word _edata
.word _sbss
.word _ebss

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0

  /* Enable CP10 and CP11 Full Access for hardware FPU */
  ldr   r0, =0xE000ED88    /* CPACR */
  ldr   r1, [r0]
  orr   r1, r1, #(0xF << 20)
  str   r1, [r0]
  dsb
  isb

  /* System and clock setup */
  bl  SystemInit

  /* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

  /* Zero fill the bss segment */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

  /* Call static constructors */
  bl __libc_init_array

  /* Call main */
  bl main

LoopForever:
  b LoopForever

.size Reset_Handler, .-Reset_Handler

  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler

  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word SecureFault_Handler
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler

  /* External Interrupts */
  .word WWDG_IRQHandler
  .word PVD_IRQHandler
  .word RTC_IRQHandler
  .word TAMP_IRQHandler
  .word FLASH_IRQHandler
  .word RCC_IRQHandler
  .word EXTI0_IRQHandler
  .word EXTI1_IRQHandler
  .word EXTI2_IRQHandler
  .word EXTI3_IRQHandler
  .word EXTI4_IRQHandler
  .word DMA1_Channel1_IRQHandler
  .word DMA1_Channel2_IRQHandler
  .word DMA1_Channel3_IRQHandler
  .word DMA1_Channel4_IRQHandler
  .word DMA1_Channel5_IRQHandler
  .word ADC1_IRQHandler
  .word FDCAN1_IT0_IRQHandler
  .word FDCAN1_IT1_IRQHandler
  .word I2C1_EV_IRQHandler
  .word I2C1_ER_IRQHandler
  .word SPI1_IRQHandler
  .word USART1_IRQHandler
  .word TIM1_BRK_UP_TRG_COM_IRQHandler
  .word TIM1_CC_IRQHandler
  .word TIM2_IRQHandler
  .word TIM3_IRQHandler

  .size g_pfnVectors, .-g_pfnVectors

  .macro def_irq_handler handler_name
  .weak \handler_name
  .set \handler_name, Default_Handler
  .endm

  def_irq_handler NMI_Handler
  def_irq_handler HardFault_Handler
  def_irq_handler MemManage_Handler
  def_irq_handler BusFault_Handler
  def_irq_handler UsageFault_Handler
  def_irq_handler SecureFault_Handler
  def_irq_handler SVC_Handler
  def_irq_handler DebugMon_Handler
  def_irq_handler PendSV_Handler
  def_irq_handler SysTick_Handler
  def_irq_handler WWDG_IRQHandler
  def_irq_handler PVD_IRQHandler
  def_irq_handler RTC_IRQHandler
  def_irq_handler TAMP_IRQHandler
  def_irq_handler FLASH_IRQHandler
  def_irq_handler RCC_IRQHandler
  def_irq_handler EXTI0_IRQHandler
  def_irq_handler EXTI1_IRQHandler
  def_irq_handler EXTI2_IRQHandler
  def_irq_handler EXTI3_IRQHandler
  def_irq_handler EXTI4_IRQHandler
  def_irq_handler DMA1_Channel1_IRQHandler
  def_irq_handler DMA1_Channel2_IRQHandler
  def_irq_handler DMA1_Channel3_IRQHandler
  def_irq_handler DMA1_Channel4_IRQHandler
  def_irq_handler DMA1_Channel5_IRQHandler
  def_irq_handler ADC1_IRQHandler
  def_irq_handler FDCAN1_IT0_IRQHandler
  def_irq_handler FDCAN1_IT1_IRQHandler
  def_irq_handler I2C1_EV_IRQHandler
  def_irq_handler I2C1_ER_IRQHandler
  def_irq_handler SPI1_IRQHandler
  def_irq_handler USART1_IRQHandler
  def_irq_handler TIM1_BRK_UP_TRG_COM_IRQHandler
  def_irq_handler TIM1_CC_IRQHandler
  def_irq_handler TIM2_IRQHandler
  def_irq_handler TIM3_IRQHandler
