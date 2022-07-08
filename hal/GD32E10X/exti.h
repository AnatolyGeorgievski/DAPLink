/*! прерывания от ног */
#ifndef EXTI_H
#define EXTI_H
#include "board.h"
enum {
	EXTI_FALLING_EDGE = 1,
	EXTI_RISING_EDGE = 2,
};
/*! \brief 
	\param port PORT_A, PORT_B,.. PORT_E идентификатор порта
	
	All GPIO pins can be selected as an EXTI trigger source by configuring AFIO_EXTISSx
 */
static inline
void exti_config(int line, int port)
{
	volatile uint32_t * exti_cr = (volatile uint32_t *)(AFIO + 0x08U + 4*(line>>2));
	*exti_cr &= ~(0xF<<((line&0x3)*4));// выбрать порт и ногу
	*exti_cr |=  port<<((line&0x3)*4) ;// выбрать порт и ногу
}
static inline
void exti_set(int lines, int edge)
{
	if (edge & EXTI_RISING_EDGE) 
		EXTI_RTEN |=  lines;
	else 
		EXTI_RTEN &= ~lines;
	if (edge & EXTI_FALLING_EDGE) 
		EXTI_FTEN |=  lines;
	else 
		EXTI_FTEN &= ~lines;
}
static inline
void exti_disable(int lines)
{
	EXTI_INTEN &= ~lines;
}
static inline
void exti_enable(int lines)
{
	EXTI_PD     =  lines; // pending interrupt 
	EXTI_INTEN |=  lines;
}
#endif//EXTI_H
