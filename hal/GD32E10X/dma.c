#include <cmsis_os.h>
#include "dma.h"
typedef struct _Channel Channel_t;
struct _Channel {
	osThreadId owner;
	int32_t flag;
//	uint16_t status;
};
static Channel_t Channels[7] = {NULL};
static void DMA_IRQHandler(uint32_t dmax,/*DMA1_Channel1,*/ int channel)
{// DMA_INTF DMA_INTC
    if (DMA_INTF(dmax) &  DMA_FLAG_ADD(DMA_INTF_FTFIF,  channel)){
		DMA_INTC(dmax) |= DMA_FLAG_ADD(DMA_INTC_FTFIFC, channel);
        if (Channels[channel].owner) {
			osSignalSet(Channels[channel].owner, 1UL<<Channels[channel].flag);
		}
    }
}
#if defined(BOARD_DMA0)
static void __attribute__((constructor)) DMA0_init()
{
}
void DMA0_Channel0_IRQHandler(void){	DMA_IRQHandler(DMA0,/*DMA1_Channel1,*/ 0); }
void DMA0_Channel1_IRQHandler(void){	DMA_IRQHandler(DMA0,/*DMA1_Channel1,*/ 1); }
void DMA0_Channel2_IRQHandler(void){	DMA_IRQHandler(DMA0,/*DMA1_Channel1,*/ 2); }
void DMA0_Channel3_IRQHandler(void){	DMA_IRQHandler(DMA0,/*DMA1_Channel1,*/ 3); }
void DMA0_Channel4_IRQHandler(void){	DMA_IRQHandler(DMA0,/*DMA1_Channel1,*/ 4); }
void DMA0_Channel5_IRQHandler(void){	DMA_IRQHandler(DMA0,/*DMA1_Channel1,*/ 5); }
void DMA0_Channel6_IRQHandler(void){	DMA_IRQHandler(DMA0,/*DMA1_Channel1,*/ 6); }
#endif

