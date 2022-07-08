#include "pio.h"

#define PIO_MD_MASK 0x3

/*! \ingroup _hal_pio
	\brief выполнить конфигурацию портов ввода/вывода
	\return 1 если конфигурация прошла успешно 
	
	*/
int PIO_Configure(const Pin *list, unsigned int size)
{
	while (size) 
	{
		Pin const* pin = list++;
		uint32_t gpiox = pin->pio;
		uint32_t mode = (pin->attribute & PIO_MODE_MASK);

		if ((pin->attribute&0x3)!=PIO_INPUT){// output
			if (pin->attribute&PIO_MAX_SPEED){
				mode |= 0x03;// Set very high output speed(120MHz) when MDx is 0b11
				GPIOx_SPD(gpiox) |=  pin->mask;
			} else {
				GPIOx_SPD(gpiox) &= ~pin->mask;
			}
		}
		if (pin->mask & 0x00FF) {
			int i;
			uint32_t mask = 0, value = 0;
			for (i= 0; i<8; i++) {
				if (pin->mask & (1UL<<i)) {
					mask  |= (0x0F<<(4*i));/* clear the specified pin mode bits */
					value |= (mode<<(4*i));
				}
			}
			uint32_t reg = GPIO_CTL0(gpiox);
			GPIO_CTL0(gpiox) = (reg & ~mask) | value;
		}
		if (pin->mask & 0xFF00) {
			int i;
			uint32_t mask = 0, value = 0;
			for (i= 8; i<16; i++) {
				if (pin->mask & (1UL<<i)) {
					mask  |= (0x0F<<(4*(i-8)));/* clear the specified pin mode bits */
					value |= (mode<<(4*(i-8)));/* set the specified pin mode bits */
				}
			}
			uint32_t reg = GPIO_CTL1(gpiox);
			GPIO_CTL1(gpiox) = (reg & ~mask) | value;
		}
		if (pin->attribute&PIO_SET){
			GPIO_BOP(gpiox) = (uint32_t)(pin->mask);
		} else {
			GPIO_BC(gpiox) = (uint32_t)(pin->mask);
		}
		AFIO_PCF0 |= pin->af_remap;
		size --;
	}	
	return 1;
}
