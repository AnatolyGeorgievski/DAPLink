/*!
    \file    startup_gd32e10x.s
    \brief   start up file

    \version 2022-04-17, firmware for GD32E10x
*/

.syntax unified
	.cpu cortex-m4
	.fpu softvfp
	.thumb

.global	g_pfnVectors
.global	Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word	_sidata
/* start address for the .data section. defined in linker script */
.word	_sdata
/* end address for the .data section. defined in linker script */
.word	_edata
/* start address for the .bss section. defined in linker script */
.word	_sbss
/* end address for the .bss section. defined in linker script */
.word	_ebss


    .section	.text.Reset_Handler
	.weak	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
/*  ldr   sp, =_estack    */
/* Atollic update: set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  movs	r1, #0
  b	LoopCopyDataInit

CopyDataInit:
	ldr	r3, =_sidata
	ldr	r3, [r3, r1]
	str	r3, [r0, r1]
	adds	r1, r1, #4

LoopCopyDataInit:
	ldr	r0, =_sdata
	ldr	r3, =_edata
	adds	r2, r0, r1
	cmp	r2, r3
	bcc	CopyDataInit
	ldr	r2, =_sbss
	b	LoopFillZerobss
/* Zero fill the bss segment. */
FillZerobss:
	movs	r3, #0
	str	r3, [r2], #4

LoopFillZerobss:
	ldr	r3, = _ebss
	cmp	r2, r3
	bcc	FillZerobss

/* Call the clock system intitialization function.*/
    bl  SystemInit
/* Call static constructors */
/*    bl __libc_init_array */
/* переключить стеки Thread Mode PSP Привилегии не трогали */
#if 1
	mov r0, sp
	msr psp, r0;
	mrs r0, control;
	orr r0, #2
	msr control, r0
	sub sp, #256
#endif
/* Call the application`s entry point.*/
	b	__main

LoopForever:
    b LoopForever
    
.size	Reset_Handler, .-Reset_Handler
/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section	.text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
	b	Infinite_Loop
	.size	Default_Handler, .-Default_Handler


/******************************************************************************
*
* The minimal vector table for a Cortex-M4.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
 	.section	.isr_vector,"a",%progbits
	.type	g_pfnVectors, %object
g_pfnVectors:
	.word     _estack                      		// Top of Stack
	.word     Reset_Handler                     //; Reset Handler
	.word     NMI_Handler                       //; NMI Handler
	.word     HardFault_Handler                 // Hard Fault Handler
	.word     MemManage_Handler                 //; MPU Fault Handler
	.word     BusFault_Handler                  //; Bus Fault Handler
	.word     UsageFault_Handler                //; Usage Fault Handler
	.word     0                                 //; Reserved
	.word     0                                 //; Reserved
	.word     0                                 //; Reserved
	.word     0                                 //; Reserved
	.word     SVC_Handler                       //; SVCall Handler
	.word     DebugMon_Handler                  //; Debug Monitor Handler
	.word     0                                 //; Reserved
	.word     PendSV_Handler                    //; PendSV Handler
	.word     SysTick_Handler                   //; SysTick Handler
               /* external interrupts handler */
	.word     WWDGT_IRQHandler                  //; 16:Window Watchdog Timer
	.word     LVD_IRQHandler                    //; 17:LVD through EXTI Line detect
	.word     TAMPER_IRQHandler                 //; 18:Tamper through EXTI Line detect
	.word     RTC_IRQHandler                    //; 19:RTC through EXTI Line
	.word     FMC_IRQHandler                    //; 20:FMC
	.word     RCU_CTC_IRQHandler                //; 21:RCU and CTC
	.word     EXTI0_IRQHandler                  //; 22:EXTI Line 0
	.word     EXTI1_IRQHandler                  //; 23:EXTI Line 1
	.word     EXTI2_IRQHandler                  //; 24:EXTI Line 2
	.word     EXTI3_IRQHandler                  //; 25:EXTI Line 3
	.word     EXTI4_IRQHandler                  //; 26:EXTI Line 4
	.word     DMA0_Channel0_IRQHandler          //; 27:DMA0 Channel0
	.word     DMA0_Channel1_IRQHandler          //; 28:DMA0 Channel1
	.word     DMA0_Channel2_IRQHandler          //; 29:DMA0 Channel2
	.word     DMA0_Channel3_IRQHandler          //; 30:DMA0 Channel3
	.word     DMA0_Channel4_IRQHandler          //; 31:DMA0 Channel4
	.word     DMA0_Channel5_IRQHandler          //; 32:DMA0 Channel5
	.word     DMA0_Channel6_IRQHandler          //; 33:DMA0 Channel6
	.word     ADC0_1_IRQHandler                 //; 34:ADC0 and ADC1
	.word     0                                 //; 35:Reserved
	.word     0                                 //; 36:Reserved
	.word     0                                 //; 37:Reserved
	.word     0                                 //; 38:Reserved
	.word     EXTI5_9_IRQHandler                //; 39:EXTI5 to EXTI9
	.word     TIMER0_BRK_TIMER8_IRQHandler      //; 40:TIMER0 Break and TIMER8
	.word     TIMER0_UP_TIMER9_IRQHandler       //; 41:TIMER0 Update and TIMER9
	.word     TIMER0_TRG_CMT_TIMER10_IRQHandler //; 42:TIMER0 Trigger and Commutation and TIMER10
	.word     TIMER0_Channel_IRQHandler         //; 43:TIMER0 Channel Capture Compare
	.word     TIMER1_IRQHandler                 //; 44:TIMER1
	.word     TIMER2_IRQHandler                 //; 45:TIMER2
	.word     TIMER3_IRQHandler                 //; 46:TIMER3
	.word     I2C0_EV_IRQHandler                //; 47:I2C0 Event
	.word     I2C0_ER_IRQHandler                //; 48:I2C0 Error
	.word     I2C1_EV_IRQHandler                //; 49:I2C1 Event
	.word     I2C1_ER_IRQHandler                //; 50:I2C1 Error
	.word     SPI0_IRQHandler                   //; 51:SPI0
	.word     SPI1_IRQHandler                   //; 52:SPI1
	.word     USART0_IRQHandler                 //; 53:USART0
	.word     USART1_IRQHandler                 //; 54:USART1
	.word     USART2_IRQHandler                 //; 55:USART2
	.word     EXTI10_15_IRQHandler              //; 56:EXTI10 to EXTI15
	.word     RTC_Alarm_IRQHandler              //; 57:RTC Alarm
	.word     USBFS_WKUP_IRQHandler             //; 58:USBFS Wakeup
	.word     TIMER7_BRK_TIMER11_IRQHandler     //; 59:TIMER7 Break and TIMER11
	.word     TIMER7_UP_TIMER12_IRQHandler      //; 60:TIMER7 Update and TIMER12
	.word     TIMER7_TRG_CMT_TIMER13_IRQHandler //; 61:TIMER7 Trigger and Commutation and TIMER13
	.word     TIMER7_Channel_IRQHandler         //; 62:TIMER7 Channel Capture Compare
	.word     0                                 //; 63:Reserved
	.word     EXMC_IRQHandler                   //; 64:EXMC
	.word     0                                 //; 65:Reserved
	.word     TIMER4_IRQHandler                 //; 66:TIMER4
	.word     SPI2_IRQHandler                   //; 67:SPI2
	.word     UART3_IRQHandler                  //; 68:UART3
	.word     UART4_IRQHandler                  //; 69:UART4
	.word     TIMER5_IRQHandler                 //; 70:TIMER5 
	.word     TIMER6_IRQHandler                 //; 71:TIMER6
	.word     DMA1_Channel0_IRQHandler          //; 72:DMA1 Channel0
	.word     DMA1_Channel1_IRQHandler          //; 73:DMA1 Channel1
	.word     DMA1_Channel2_IRQHandler          //; 74:DMA1 Channel2
	.word     DMA1_Channel3_IRQHandler          //; 75:DMA1 Channel3
	.word     DMA1_Channel4_IRQHandler          //; 76:DMA1 Channel4
	.word     0                                 //; 77:Reserved
	.word     0                                 //; 78:Reserved
	.word     0                                 //; 79:Reserved
	.word     0                                 //; 80:Reserved
	.word     0                                 //; 81:Reserved
	.word     0                                 //; 82:Reserved
	.word     USBFS_IRQHandler                  //; 83:USBFS

	.size	g_pfnVectors, .-g_pfnVectors

	.weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler

	.weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

	.weak	MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler

	.weak	BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler

	.weak	UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak	DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler

	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler

              /* external interrupts handler */
	.weak	WWDGT_IRQHandler
	.thumb_set WWDGT_IRQHandler,Default_Handler
	.weak	LVD_IRQHandler
	.thumb_set LVD_IRQHandler,Default_Handler                  
	.weak	TAMPER_IRQHandler
	.thumb_set TAMPER_IRQHandler,Default_Handler           
	.weak	RTC_IRQHandler
	.thumb_set RTC_IRQHandler,Default_Handler               
	.weak	FMC_IRQHandler
	.thumb_set FMC_IRQHandler,Default_Handler                
	.weak	RCU_CTC_IRQHandler
	.thumb_set RCU_CTC_IRQHandler,Default_Handler                 
	.weak	EXTI0_IRQHandler
	.thumb_set EXTI0_IRQHandler,Default_Handler                  
	.weak	EXTI1_IRQHandler
	.thumb_set EXTI1_IRQHandler,Default_Handler                 
	.weak	EXTI2_IRQHandler
	.thumb_set EXTI2_IRQHandler,Default_Handler               
	.weak	EXTI3_IRQHandler
	.thumb_set EXTI3_IRQHandler,Default_Handler                
	.weak	EXTI4_IRQHandler
	.thumb_set EXTI4_IRQHandler,Default_Handler                 
	.weak	DMA0_Channel0_IRQHandler
	.thumb_set DMA0_Channel0_IRQHandler,Default_Handler         
	.weak	DMA0_Channel1_IRQHandler
	.thumb_set DMA0_Channel1_IRQHandler,Default_Handler          
	.weak	DMA0_Channel2_IRQHandler
	.thumb_set DMA0_Channel2_IRQHandler,Default_Handler        
	.weak	DMA0_Channel3_IRQHandler
	.thumb_set DMA0_Channel3_IRQHandler,Default_Handler         
	.weak	DMA0_Channel4_IRQHandler
	.thumb_set DMA0_Channel4_IRQHandler,Default_Handler          
	.weak	DMA0_Channel5_IRQHandler
	.thumb_set DMA0_Channel5_IRQHandler,Default_Handler          
	.weak	DMA0_Channel6_IRQHandler
	.thumb_set DMA0_Channel6_IRQHandler,Default_Handler          
	.weak	ADC0_1_IRQHandler
	.thumb_set ADC0_1_IRQHandler,Default_Handler         
	.weak	EXTI5_9_IRQHandler
	.thumb_set EXTI5_9_IRQHandler,Default_Handler           
	.weak	TIMER0_BRK_TIMER8_IRQHandler
	.thumb_set TIMER0_BRK_TIMER8_IRQHandler,Default_Handler  
	.weak	TIMER0_UP_TIMER9_IRQHandler
	.thumb_set TIMER0_UP_TIMER9_IRQHandler,Default_Handler  
	.weak	TIMER0_TRG_CMT_TIMER10_IRQHandler
	.thumb_set TIMER0_TRG_CMT_TIMER10_IRQHandler,Default_Handler
	.weak	TIMER0_Channel_IRQHandler
	.thumb_set TIMER0_Channel_IRQHandler,Default_Handler        
	.weak	TIMER1_IRQHandler
	.thumb_set TIMER1_IRQHandler,Default_Handler       
	.weak	TIMER2_IRQHandler
	.thumb_set TIMER2_IRQHandler,Default_Handler           
	.weak	TIMER3_IRQHandler
	.thumb_set TIMER3_IRQHandler,Default_Handler           
	.weak	I2C0_EV_IRQHandler
	.thumb_set I2C0_EV_IRQHandler,Default_Handler          
	.weak	I2C0_ER_IRQHandler
	.thumb_set I2C0_ER_IRQHandler,Default_Handler         
	.weak	I2C1_EV_IRQHandler
	.thumb_set I2C1_EV_IRQHandler,Default_Handler         
	.weak	I2C1_ER_IRQHandler
	.thumb_set I2C1_ER_IRQHandler,Default_Handler         
	.weak	SPI0_IRQHandler
	.thumb_set SPI0_IRQHandler,Default_Handler        
	.weak	SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,Default_Handler          
	.weak	USART0_IRQHandler
	.thumb_set USART0_IRQHandler,Default_Handler         
	.weak	USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler         
	.weak	USART2_IRQHandler
	.thumb_set USART2_IRQHandler,Default_Handler        
	.weak	EXTI10_15_IRQHandler
	.thumb_set EXTI10_15_IRQHandler,Default_Handler        
	.weak	RTC_Alarm_IRQHandler
	.thumb_set RTC_Alarm_IRQHandler,Default_Handler        
	.weak	USBFS_WKUP_IRQHandler
	.thumb_set USBFS_WKUP_IRQHandler,Default_Handler        
	.weak	TIMER7_BRK_TIMER11_IRQHandler
	.thumb_set TIMER7_BRK_TIMER11_IRQHandler,Default_Handler 
	.weak	TIMER7_UP_TIMER12_IRQHandler
	.thumb_set TIMER7_UP_TIMER12_IRQHandler,Default_Handler 
	.weak	TIMER7_TRG_CMT_TIMER13_IRQHandler
	.thumb_set TIMER7_TRG_CMT_TIMER13_IRQHandler,Default_Handler
	.weak	TIMER7_Channel_IRQHandler
	.thumb_set TIMER7_Channel_IRQHandler,Default_Handler 
	.weak	EXMC_IRQHandler
	.thumb_set EXMC_IRQHandler,Default_Handler         
	.weak	TIMER4_IRQHandler
	.thumb_set TIMER4_IRQHandler,Default_Handler           
	.weak	SPI2_IRQHandler
	.thumb_set SPI2_IRQHandler,Default_Handler          
	.weak	UART3_IRQHandler
	.thumb_set UART3_IRQHandler,Default_Handler          
	.weak	UART4_IRQHandler
	.thumb_set UART4_IRQHandler,Default_Handler          
	.weak	TIMER5_IRQHandler
	.thumb_set TIMER5_IRQHandler,Default_Handler         
	.weak	TIMER6_IRQHandler
	.thumb_set TIMER6_IRQHandler,Default_Handler        
	.weak	DMA1_Channel0_IRQHandler
	.thumb_set DMA1_Channel0_IRQHandler,Default_Handler          
	.weak	DMA1_Channel1_IRQHandler
	.thumb_set DMA1_Channel1_IRQHandler,Default_Handler          
	.weak	DMA1_Channel2_IRQHandler
	.thumb_set DMA1_Channel2_IRQHandler,Default_Handler         
	.weak	DMA1_Channel3_IRQHandler
	.thumb_set DMA1_Channel3_IRQHandler,Default_Handler         
	.weak	DMA1_Channel4_IRQHandler
	.thumb_set DMA1_Channel4_IRQHandler,Default_Handler          
	.weak	USBFS_IRQHandler
	.thumb_set USBFS_IRQHandler,Default_Handler          
