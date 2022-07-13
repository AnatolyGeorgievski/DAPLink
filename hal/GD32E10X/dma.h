/* dma.h */
#ifndef DMA_H
#define DMA_H
#include "board.h"
#include "gd32e10x_dma.h"

/*! 7 channel DMA0 controller and 5 channel DMA1 controller
	\see GD32E10x User Manual, Figure 10-4. DMA0 request mapping
	DMA0 Ch1 USART2_TX
	DMA0 Ch2 USART2_RX
	DMA0 Ch3 USART0_TX
	DMA0 Ch4 USART0_RX
	DMA0 Ch5 USART1_RX I2C0_TX
	DMA0 Ch6 USART1_TX I2C0_RX

 */ 
 
#define DMA_CANNEL(dmax,ch) ((DMA_Channel_t*)(dmax + 0x8U + 20*(ch)))
#define DMA0_Channel0 DMA_CANNEL(DMA0,0)
#define DMA0_Channel1 DMA_CANNEL(DMA0,1)
#define DMA0_Channel2 DMA_CANNEL(DMA0,2)
#define DMA0_Channel3 DMA_CANNEL(DMA0,3)
#define DMA0_Channel4 DMA_CANNEL(DMA0,4)
#define DMA0_Channel5 DMA_CANNEL(DMA0,5)
#define DMA0_Channel6 DMA_CANNEL(DMA0,6)

enum {
	ERR_DMA_OK=0,
	ERR_DMA_BUSY=1, // занят
	ERR_DMA_TE, // transfer error
	ERR_DMA_HT, // half transfer
};

typedef volatile struct _DMA_Channel DMA_Channel_t;
struct _DMA_Channel {
	uint32_t CTL;
	uint32_t CNT;
	uint32_t PADDR;
	uint32_t MADDR;
	uint32_t reserved;
};

/*! спросить количество элементов до конца трансфера */
static inline uint32_t DMA_counter(DMA_Channel_t*  ds)
{
	return ds->CNT & DMA_CHXCNT_CNT;
}
/* оборвать трансфер 
static inline void DMA_break(DMA_Channel_t*  ch, DMA_TypeDef*  dma, uint32_t channel)
{
	uint32_t flag = DMA_ISR_TCIF1<<((channel-1)<<2);
	ch->CCR &= ~DMA_CCR1_EN;
	dma->IFCR = flag;
}
*/
/*!	\brief
	\param mode
	\param paddr адрес регистра переферии
 */
static inline void DMA_init(DMA_Channel_t* ds, uint32_t mode, void* paddr)
{
	ds->CTL   = mode;
	ds->PADDR  = (uint32_t)paddr;
}
/*!	\brief выбрать канал DMA
	\param mode
	\param paddr адрес регистра переферии
 */
static inline DMA_Channel_t* DMA_channel(uint32_t dmax, int index){
	DMA_Channel_t* ds = (void*)(dmax + 0x8U + 20*index);
	return ds;
}
// отослать данные 
static inline void DMA_send_recv(DMA_Channel_t* ds, void* buffer, unsigned int size)
{
	ds->MADDR  = (uint32_t) buffer;
	ds->CNT =  size & DMA_CHXCNT_CNT;
	ds->CTL |=  (uint32_t)DMA_CHXCTL_CHEN;
}
static inline void DMA_enable(DMA_Channel_t* ds)
{
	ds->CTL |= (uint32_t)DMA_CHXCTL_CHEN;
}
static inline void DMA_disable(DMA_Channel_t* ds)
{
	ds->CTL &= ~(uint32_t)DMA_CHXCTL_CHEN;
}
/*! \brief Разрешить прерывание для выбранного канала DMA
	\param ds канал DMA контроллера
	\param intf флаги прерывания
		\arg DMA_CHXCTL_CHEN 	channel enable
		\arg DMA_CHXCTL_FTFIE full enable bit for channel full transfer finish interrupt
		\arg DMA_CHXCTL_HTFIE enable bit for channel half transfer finish interrupt
		\arg DMA_CHXCTL_ERRIE enable bit for channel error interrupt
 */
static inline void DMA_channel_IT_enable(DMA_Channel_t* ds, uint32_t intf)
{
	ds->CTL |= (uint32_t)(intf&0xF);
}
/*! \brief Запретить прерывание для выбранного канала DMA 
	\param ds канал DMA контроллера
	\param intf флаги прерывания
		\arg DMA_CHXCTL_CHEN 	channel enable
		\arg DMA_CHXCTL_FTFIE full enable bit for channel full transfer finish interrupt
		\arg DMA_CHXCTL_HTFIE enable bit for channel half transfer finish interrupt
		\arg DMA_CHXCTL_ERRIE enable bit for channel error interrupt
 */
static inline void DMA_IT_disable(DMA_Channel_t* ds, int index, uint32_t intf)
{
	ds->CTL &= ~(uint32_t)(intf&0xF);
}
#endif //DMA_H
