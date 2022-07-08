/*!	\ingroup _hal
	\defgroup _hal_pio Cortex-M3 порты ввода/вывода PIO 

Единая модель аппаратуры для ARM7, ARM9 и Cortex-M3, Cortex-M4

	Настройка выводов порта, 
	порты сгруппированы по 16 бит, порты обозначаются буквами A..G

	\{
*/
#ifndef PIO_H
#define PIO_H
// в заголовке board указывается и определяется привязка к аппаратуре
#include "board.h"

//! "Pin types"
/// The pin is controlled by the associated signal of peripheral A.
/*
00: Analog mode
01: Floating input mode
10: Input
*/
// Настраивается в GPIOx_CTL0/ GPIOx_CTL1 x=A..E

/* GPIOx_CTL0/ GPIOx_CTL1 x=A..E
00: Input mode (reset state)
01: Output mode ,max speed 10MHz
10: Output mode ,max speed 2 MHz
11: Output mode ,max speed 50MHz
*/
#define PIO_SET					(1 << 7)
#define PIO_MAX_SPEED			(1 << 6)

#define PIO_OPENDRAIN   		(1 << 2)// применяется только к выходному сигналу
#define PIO_ALTERNATE	 		(2 << 2)// применяется только к выходному сигналу
// применяется только к выходным ногам
#define PIO_OUTPUT_LOW_SPEED 			(2 << 0)//  5 MHz
#define PIO_OUTPUT_MEDIUM_SPEED 		(1 << 0)// 10 MHz
#define PIO_OUTPUT_FAST_SPEED 			(3 << 0)// 50 MHz
#define PIO_OUTPUT_HIGH_SPEED 	(PIO_OUTPUT_FAST_SPEED | PIO_MAX_SPEED)

#define PIO_INPUT_ANALOG		(0 << 2)
#define PIO_INPUT_FLOATING      (1 << 2)// floating
#define PIO_INPUT_PUPD_MASK	    (1 << 3)
#define PIO_INPUT_PULLUP        (PIO_INPUT_PUPD_MASK|PIO_SET)
#define PIO_INPUT_PULLDOWN      (PIO_INPUT_PUPD_MASK)
#define PIO_INPUT				(0)

#define PIO_MODE_MASK 0xF
//! "Pin attributes"
#define PIO_DEFAULT                 0

//! \brief   Performs the low-level initialization of the chip.
typedef struct _PinRef PinRef_t;
struct _PinRef {
	uint32_t pio;  //!< Pointer to the PIO controller which has the pin(s).
	uint16_t mask;  //!< Bitmask indicating which pin(s) to configure.
};
typedef struct _Pin Pin;
struct _Pin {
    uint16_t mask;  //!< Bitmask indicating which pin(s) to configure.
    uint32_t pio;  //!< Pointer to the PIO controller which has the pin(s).
    uint16_t attribute; //!< Pin attribute (\see "Pin attributes").
	uint32_t af_remap;//!< Alternate function 
	
};

//! Вычисляет количество элементов в массиве
#define PIO_LISTSIZE(list)    (sizeof(list) / sizeof(Pin))
int PIO_Configure(const Pin *list, unsigned int size);

/*! \brief выставить логическую единицу на вывод 
	\param pin - описание вывода
*/
static inline void pio_set_output(uint32_t gpiox, uint16_t mask){
    GPIO_BOP(gpiox) = (uint32_t)mask;	// выставить биты
}
static inline void pio_set_reset(uint32_t gpiox, uint16_t set, uint16_t reset){
    GPIO_BOP(gpiox) = ((uint32_t)reset<<16) | set;	// выставить биты
}
static inline void pio_set_state(uint32_t gpiox, uint16_t mask, int enable){
    GPIO_BOP(gpiox) = enable?(uint32_t)mask:(((uint32_t)mask)<<16);	// выставить биты
}

/*! \brief выставить логический ноль на вывод 
	\param pin - описание вывода
*/
static inline void pio_reset_output(uint32_t gpiox, uint16_t mask){
    GPIO_BC(gpiox) = mask;	// очистить биты
}
/*! \brief считать состояние выводов 
	\param pin - описание вывода
*/
static inline unsigned int pio_get_state(uint32_t gpiox, uint16_t mask){
    return GPIO_ISTAT(gpiox) & mask;
}
/*! \brief считать состояние выводов 
	Используется для быстрого переключения состояния.
 */
static inline void pio_mode_select(uint32_t gpiox, uint16_t pin_mask, uint16_t mode){
	mode &= PIO_MODE_MASK;
	while(pin_mask) {
		int i = __builtin_ctz(pin_mask);
		if (i < 8) {
			register uint32_t mask, value;
			mask  = (0x0F<<(4*i));/* clear the specified pin mode bits */
			value = (mode<<(4*i));
			GPIO_CTL0(gpiox) = (GPIO_CTL0(gpiox) & ~mask) | value;
		} else
		{
			register uint32_t mask, value;
			mask  = (0x0F<<(4*(i-8)));/* clear the specified pin mode bits */
			value = (mode<<(4*(i-8)));/* set the specified pin mode bits */
			GPIO_CTL1(gpiox) = (GPIO_CTL1(gpiox) & ~mask) | value;
		}
		pin_mask &=~(1U<<i);
	}// while(pin_mask)
}

/*! \brief переназначение аппаратных функций между портами ввода/вывода


	Функция выполняет настройку карты периферии, каждый бит в маске отвечает за
	переназначение функции аппаратуры между портами ввода/вывода
	функция не используется в модели ARM7 и ARM9 
*/
/*
static inline void PIO_PinRemap(uint32_t  remap, uint32_t remap_mask)
{
	AFIO->MAPR = (AFIO->MAPR & ~remap_mask) | remap;
}
*/
#endif  //PIO_H
//!	\}
