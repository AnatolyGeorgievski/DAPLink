#include <cmsis_os.h>
#include "board.h"
#include "adc.h"
#include "dma.h"

/* value convert 
        temperature = (1.43 - ADC_IDATA0(ADC0)*3.3/4096) * 1000 / 4.3 + 25;
        vref_value = (ADC_IDATA1(ADC0) * 3.3 / 4096);
*/
#define RCU_ADC_PSC_OFFSET          (14U)
void adc_init(uint32_t adcx, int prescaler) 
{
	//rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
    RCU_CFG0 = (RCU_CFG0 &~(RCU_CFG0_ADCPSC_2 | RCU_CFG0_ADCPSC))|(prescaler<<RCU_ADC_PSC_OFFSET);
    /* enable the temperature sensor and Vrefint channel */
    ADC_CTL1(ADC0) |= ADC_CTL1_TSVREN;	
    /* ADC temperature sensor channel config */
//    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_16, ADC_SAMPLETIME_239POINT5);
    /* ADC internal reference voltage channel config */
//    adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_17, ADC_SAMPLETIME_239POINT5);
    /* enable ADC interface */
    adc_enable(ADC0);
    osDelay(1);
    /* ADC calibration and reset calibration */
//    adc_calibration_enable(ADC0);
    /* reset the selected ADC calibration registers */
    ADC_CTL1(ADC0) |= (uint32_t) ADC_CTL1_RSTCLB;
    /* check the RSTCLB bit state */
    while(RESET != (ADC_CTL1(ADC0) & ADC_CTL1_RSTCLB)){
		osThreadYield();
    }
    /* enable ADC calibration process */
    ADC_CTL1(ADC0) |= ADC_CTL1_CLB;
    /* check the CLB bit state */
    while(RESET != (ADC_CTL1(ADC0) & ADC_CTL1_CLB)){
		osThreadYield();
    }
}

static uint16_t adc_data[16]={0};
static uint32_t channel_idx =0;

uint16_t adc_channel_get_value(uint32_t adcx, int rank)
{	// проверки всякие
//	if (ADC0 == adcx)
		return adc_data[rank-1];
//	else 
//	if (ADC0 == adcx)
//		return adc_data1[rank-1];
}

struct _adc_ctx {
	osThreadId owner;
	int32_t flag;
};
static struct _adc_ctx adc_ctx;
void adc_open(uint32_t adcx, int32_t flag) {
	struct _adc_ctx *ctx = &adc_ctx;
	ctx->owner = osThreadGetId();
	ctx->flag  = flag;
}
void adc_start_dma(uint32_t adcx, uint16_t* buffer, unsigned int size)
{
	DMA_Channel_t* dma_adc = DMA0_Channel0;//DMA_channel(DMA0, 0);
	uint32_t mode = 
		DMA_PERIPHERAL_TO_MEMORY |
		DMA_PERIPH_INCREASE_DISABLE| DMA_PERIPHERAL_WIDTH_16BIT| 
		DMA_MEMORY_INCREASE_ENABLE | DMA_MEMORY_WIDTH_16BIT|
		DMA_PRIORITY_HIGH|
		DMA_CHXCTL_CMEN ;// enable DMA circulation mode 
	DMA_init(dma_adc, mode, (void*)&ADC_RDATA(ADC0));
	DMA_send_recv(dma_adc, buffer, size);
}