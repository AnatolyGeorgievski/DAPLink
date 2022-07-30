#include "gd32e10x.h"
#define BOARD_NAME "SH29daplink"
// если определен то используется RTC иначе SysTick
#define BOARD_RTC	// разрешить использовать календарь
#undef  BOARD_RTC_SOURCE_HSE // использовать внешний тактовый сигнал на часы
//#define HAL_PCD_MODULE_ENABLED

#define HXTAL_VALUE    ((uint32_t)8000000) /*!< value of the external oscillator in Hz */
#define HXTAL_VALUE_8M  HXTAL_VALUE


#define BOOT_INTERNAL_FLASH
#define FS_CONFIG_BASE 0
#define TRACE_BUFFER_SIZE 1024

#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bits for pre-emption priority
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bits for pre-emption priority
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority
                                                                 1 bits for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority
                                                                 0 bits for subpriority */

#define SET_BIT(REG, BIT)     ((REG) |=  (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define WRITE_REG(REG, VAL)   ((REG)  =  (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

// bit-band alias
#define BB_ALIAS(p, bit)	((uint32_t*)(((uint32_t)(p) & 0xF0000000) | 0x02000000 | (((uint32_t)(p) & 0xFFFFF)*32) + ((bit)*4)))
#define BB(p)				((uint32_t*)(((uint32_t)(p) & 0xF0000000) | 0x02000000 | (((uint32_t)(p) & 0xFFFFF)*32)))

// Настройка частоты
#define BOARD_MCK			120 /* MHz */
#define BOARD_PCLK1			60  /* MHz APB1 */
#define BOARD_PCLK2			120 /* MHz APB2 */
#define BOARD_RTC_PERIOD 	40  /* us */
#define BOARD_STK_RELOAD	(BOARD_MCK*BOARD_RTC_PERIOD) /* SysTick */

#define BOARD_AHB_ENABLE	\
	(RCU_AHBEN_DMA0EN|RCU_AHBEN_SRAMSPEN|RCU_AHBEN_FMCSPEN| RCU_AHBEN_CRCEN | RCU_AHBEN_USBFSEN)
#define BOARD_AHB_RESET	\
	(~0UL)


#define BOARD_APB1_ENABLE	\
	( RCU_APB1EN_I2C0EN|RCU_APB1EN_I2C1EN|RCU_APB1EN_PMUEN|RCU_APB1EN_USART1EN)
//| RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM13EN
#define BOARD_APB2_ENABLE	\
	(RCU_APB2EN_AFEN|RCU_APB2EN_PAEN| RCU_APB2EN_PBEN|RCU_APB2EN_PCEN|RCU_APB2EN_PDEN|\
	 RCU_APB2EN_ADC0EN|RCU_APB2EN_USART0EN)
 //RCC_APB2ENR_SPI1EN


#define BOARD_APB1_RESET 	\
	(~0UL)

#define BOARD_APB2_RESET 	\
	(~0UL)
	
#define BOARD_ADC1_PIN	{(1<<0), GPIOB, PIO_INPUT_ANALOG, 0} /* PB0 */
#define BOARD_SWO_PIN   {(1<<3), GPIOB, PIO_ALTERNATE, GPIO_SWJ_NONJTRST_REMAP} /* SYS PB3 */
/* PB5 INT прерывание на линии I2C0 SMBA */
#define BOARD_I2C1_PIN	{(1<<11)|(1<<10),GPIOB, PIO_ALTERNATE|PIO_OPENDRAIN|PIO_OUTPUT_HIGH_SPEED}
//	{(1<<12),GPIOB, PIO_INPUT_PULLUP}
#define BOARD_I2C0_PIN	{(1<< 7)|(1<< 6),GPIOB, PIO_ALTERNATE|PIO_OPENDRAIN|PIO_OUTPUT_HIGH_SPEED}, \
	{(1<<5),GPIOB, PIO_INPUT_PULLUP}
#define BOARD_I2C_PIN	{(1<<9)|(1<<8),GPIOB, PIO_ALTERNATE|PIO_OPENDRAIN|PIO_OUTPUT_HIGH_SPEED, AFIO_PCF0_I2C0_REMAP}, \
						{(1<<5),GPIOB, PIO_INPUT_PULLUP}
						
#define BOARD_USBD_PINS {0x1800, GPIOA, PIO_ALTERNATE|PIO_OUTPUT_HIGH_SPEED, }
/* PB12 -- PD - детектирование платы */
#define BOARD_SIGNAL_PIN {(1<<12), GPIOB, PIO_INPUT_PULLDOWN}
#define BOARD_RESET_PIN  {(1<< 9), GPIOB, PIO_OUTPUT_HIGH_SPEED|PIO_SET}
/*	PA0 - кнопки */
#define BOARD_BUTTON_PIN {(1<< 0), GPIOA, PIO_INPUT_PULLUP}
 /* PB6 TX Alternate function push-pull 
	PB7 RX Input floating / Input pull-up
	PA12 TE   */
#if defined(GD32E103xB)
#define BOARD_USART1_PIN 	\
	{(1<<2),GPIOA, PIO_ALTERNATE|PIO_OUTPUT_HIGH_SPEED}, \
	{(1<<3),GPIOA, PIO_INPUT_FLOATING}
#define BOARD_USART0_PIN 	\
	{(1<< 9),GPIOA, PIO_ALTERNATE|PIO_OUTPUT_HIGH_SPEED}, \
	{(1<<10),GPIOA, PIO_INPUT_FLOATING}, \
	{(1<< 8),GPIOB, PIO_OUTPUT_HIGH_SPEED}
#define BOARD_USART0_PIN1 	\
	{(1<<6),GPIOB, PIO_ALTERNATE|PIO_OUTPUT_HIGH_SPEED, AFIO_PCF0_USART0_REMAP}, \
	{(1<<7),GPIOB, PIO_INPUT_FLOATING}, \
	{(1<<12),GPIOA, PIO_OUTPUT_HIGH_SPEED}
#else
	#warning "BOARD.H надо определить пины USART1 для контроллера"
#endif


#define BOARD_ALL_PINS	BOARD_SWO_PIN, BOARD_USBD_PINS, BOARD_I2C0_PIN, BOARD_I2C1_PIN, \
	BOARD_USART0_PIN, BOARD_USART1_PIN, BOARD_SIGNAL_PIN, BOARD_BUTTON_PIN
//BOARD_ADC1_PIN
#define BOARD_SAFE_PINS do { \
			pio_set_reset(GPIOA, 0, (1<<12)); \
			pio_set_reset(GPIOB, 0xFC00, 0); \
		} while(0);

//#define DEBUG_PORT USART0

#define CDC_SUB_ITF_COUNT 3

#define BOARD_USBFS // нужно!!

#define BOARD_ADC0 
#define BOARD_DMA0 
#define BOARD_USART0 
#define BOARD_USART0_BAUDRATE 38400
#define BOARD_USART1 
#define BOARD_USART1_DMA
#define BOARD_USART1_BAUDRATE 115200
//#define BOARD_USART2 

#define PIN_SWCLK_POS 5
#define PIN_SWDIO_POS 4
#define PIN_RESET_POS 9
#define PIN_SWCLK GPIOA, (1U<<PIN_SWCLK_POS)
#define PIN_SWDIO GPIOA, (1U<<PIN_SWDIO_POS)
#define PIN_RESET   GPIOB, (1<< 9)
#define PIN_DETECT  GPIOB, (1<<12)
#define PIN_LED_RUN GPIOB, (1<<13)
