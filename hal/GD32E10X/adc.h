/*!	\ingroup _hal
	\defgroup _hal_adc GD32E10X Аналого-цифровой преобразователь

	\{
*/
#ifndef ADC_H
#define ADC_H
// в заголовке board указывается и определяется привязка к аппаратуре
#include "board.h"
#include "gd32e10x_adc.h"


#define ADC_CHANNEL(n)			(n)
#define ADC_CHANNEL_TEMPERATURE ADC_CHANNEL(16)
#define ADC_CHANNEL_TEMPSENSOR ADC_CHANNEL(16)
#define ADC_CHANNEL_VREFINT ADC_CHANNEL(17)

static inline void adc_enable(uint32_t adcx) {
	ADC_CTL1(adcx) |= (uint32_t)ADC_CTL1_ADCON;
}
static inline void adc_disable(uint32_t adcx) {
	ADC_CTL1(adcx) &= ~((uint32_t)ADC_CTL1_ADCON);
}

static inline uint16_t adc_get_value(uint32_t adcx) {
	return ADC_RDATA(adcx);// & ADC_DR_DATA;
}

void 	 adc_init(uint32_t adcx, int prescaler);
static inline void 	 adc_channel_config(uint32_t adcx, uint32_t channel,  int sampling_time, int rank )
{
	if (rank < 6) { 
		ADC_RSQ2(adcx) = (ADC_RSQ2(adcx) &~(0x1F<<(5*rank))) | (channel <<(5*rank)); 
	} else
	if (rank <12) {
		ADC_RSQ1(adcx) = (ADC_RSQ1(adcx) &~(0x1F<<(5*(rank-6)))) | (channel <<(5*(rank-6))); 
	} else
	if (rank <16) { 
		ADC_RSQ0(adcx) = (ADC_RSQ0(adcx) &~(0x1F<<(5*(rank-12)))) | (channel <<(5*(rank-12))); 
	}
	if(channel < 10){
		ADC_SAMPT1(adcx) = (ADC_SAMPT1(adcx) &~(0x7<<(3*(channel)))) | (sampling_time<<(3*(channel)));
	} else
	if(channel < 18){
		ADC_SAMPT0(adcx) = (ADC_SAMPT0(adcx) &~(0x7<<(3*(channel-10)))) | (sampling_time<<(3*(channel-10)));
	}
/*
	else if(channel < 18){
		ADC_CTL1(ADC0) |= ADC_CTL1_TSVREN;
		ADC_SAMPT0(ADC0) = (ADC_SAMPT0(ADC0) &~(0x7<<(3*(channel-10)))) | (sampling_time<<(3*(channel-10)));
	}
	*/	
}
static inline uint16_t adc_inserted_get_value(uint32_t adcx, uint32_t ch)
{
	return REG32((adcx) + 0x3CU + 4U*ch);
}
uint16_t adc_channel_get_value(uint32_t adcx, int rank);
void adc_open(uint32_t adcx, int32_t flag);
void adc_start_dma(uint32_t adcx, uint16_t* buffer, unsigned int size);
 
#endif //ADC_H
//! \}
