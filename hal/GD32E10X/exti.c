
#include "board.h"
#include "exti.h"
static void EXTIn_IRQHandler(uint32_t pin_mask) {
	if((EXTI_PD & pin_mask) != 0) { 
		EXTI_PD = pin_mask; // clear pending interrupt
	}
}
void EXTI0_IRQHandler() 		{ EXTIn_IRQHandler(1<<0); }
void EXTI1_IRQHandler() 		{ EXTIn_IRQHandler(1<<1); }
void EXTI2_IRQHandler() 		{ EXTIn_IRQHandler(1<<2); }
void EXTI3_IRQHandler() 		{ EXTIn_IRQHandler(1<<3); }
void EXTI4_IRQHandler() 		{ EXTIn_IRQHandler(1<<4); }
void EXTI9_5_IRQHandler(void)	{ EXTIn_IRQHandler((1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<9)); }
void EXTI15_10_IRQHandler(void)	{ EXTIn_IRQHandler((1<<10)|(1<<11)|(1<<12)|(1<<13)|(1<<14)|(1<<15)); }
