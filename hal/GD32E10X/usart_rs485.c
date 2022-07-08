/*! USART RS485

 */
#include <cmsis_os.h>
#include <stdio.h>
#include "usart.h"
#include "pio.h"
#include "dma.h"
typedef struct _USART_Handle USART_Handle;
struct _USART_Handle {
	osThreadId owner;
	int8_t flag;
	int8_t uart_id;
	int16_t status;
	osTransfer rx;
	uint8_t* tx_buffer;
	uint16_t tx_size;
	uint16_t tx_idx;
	uint16_t rx_max;
};
typedef struct _USART USART_t;
struct _USART { 
	uint32_t port;
	int IRQn;
	DMA_Channel_t* tx_dma;
	DMA_Channel_t* rx_dma;
};
static USART_Handle usart_ports[3];
static const USART_t usarts[] = {
#ifdef BOARD_USART0
	{USART0, USART0_IRQn, DMA0_Channel3, DMA0_Channel4}, 
#endif
#ifdef BOARD_USART1
	{USART1, USART1_IRQn, DMA0_Channel6, DMA0_Channel5}, 
#endif
#ifdef BOARD_USART2
	{USART2, USART2_IRQn, DMA0_Channel1, DMA0_Channel2}
#endif
};

static void usart_rs485_init(uint32_t usartx)
{
	// usart_baudrate_set	
	if (usartx==USART0) {
		USART_BAUD(USART0) =  (BOARD_PCLK2 + BOARD_USART0_BAUDRATE/2U)/BOARD_USART0_BAUDRATE;
	}
	else 
#if defined (BOARD_USART1) && defined (BOARD_USART1_BAUDRATE)
	if (usartx==USART1){
		USART_BAUD(USART1) =  (BOARD_PCLK1 + BOARD_USART1_BAUDRATE/2U)/BOARD_USART1_BAUDRATE;
	} else
#endif
	{}

}
void* usart_open(uint32_t uart_id, int32_t flag)
{
	const USART_t * usart=&usarts[uart_id]; 
	USART_Handle* h = &usart_ports[uart_id];
	h->uart_id  = uart_id;// номер порта
	h->owner = osThreadGetId();
	h->flag  = flag;
	h->rx.buffer = NULL;
	h->tx_buffer = NULL;
	uint32_t usartx = usarts[uart_id].port;
	usart_rs485_init(usartx);
	USART_CTL0(usartx) |= USART_CTL0_REN| USART_CTL0_TEN;// включить прием и передачу
	USART_CTL0(usartx) |= USART_CTL0_UEN;// включить USART
	USART_CTL0(usartx) |= USART_CTL0_RBNEIE | USART_CTL0_TBEIE | USART_CTL0_TCIE;
	NVIC_SetPriority(usart->IRQn, 1);
	NVIC_ClearPendingIRQ(usart->IRQn);
	//NVIC_EnableIRQ(usart->IRQn);
	return h;
}
#if 0
void* usart_dma_open(uint32_t uart_id, int32_t flag)
{
	uint32_t usartx=usart->port; 
	DMA_init(usart->tx_dma, DMA_CCR_DIR|DMA_CCR_MINC|DMA_CCR_PSIZE_0| DMA_CCR_TCIE | DMA_CCR_TEIE, (void*)&uart->TDR);
	DMA_init(usart->rx_dma, DMA_CCR_MINC| DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE, (void*)&uart->RDR);
	/* configure DMA reception */
	// USART_CTL2(usartx) = (USART_CTL2(usartx) &~ (USART_CTL2_DENR|USART_CTL2_DENT)) | ()
	// USART_CTL2(usartx) |= (USART_CTL2_DENR|USART_CTL2_DENT);
}
#endif
void usart_send(void* vh, void* data, uint16_t size)
{
	USART_Handle* h = vh;
	h->tx_buffer = data;
	h->tx_size = size;
	h->tx_idx = 0;
	h->status=0;
	uint32_t usartx = usarts[h->uart_id].port;
	USART_CTL0(usartx) |= USART_CTL0_TBEIE;
	//return h->tx_transfer;
#ifdef BOARD_USART0_TE
	pio_set_output(BOARD_USART0_TE);
#endif

}
osTransfer* usart_recv(void* vh, void* data, uint16_t size)
{
	USART_Handle* h = vh;
	h->rx.buffer = data;
	h->rx.size = size;
#ifdef BOARD_USART0_RTIMEOUT
	uint32_t usartx = usarts[h->uart_id].port;
    USART_RT(usartx) = (USART_RT(usartx) & ~(USART_RT_RT)) | rtimeout;
#endif
	return &h->rx;
}
#if 0
int usart_ioctl(int )
{
	switch (){
	case GUARD_TIME: // configure guard time value in smartcard mode
	}
}
#endif
static void USARTn_IRQHandler(const uint32_t usartx, USART_Handle* h)
{
	uint16_t stat = USART_STAT0(usartx);
	if (stat & USART_STAT0_RBNE){
		uint8_t data = USART_DATA(usartx);
		
		h->rx.buffer[h->rx.size++] = data;
		if (h->rx.size >= h->rx_max){
			USART_CTL0(usartx)  &= ~USART_CTL0_RBNEIE;
			USART_STAT0(usartx) &= ~USART_STAT0_RBNE;
		}
	}
	if (stat & USART_STAT0_TBE){
		if (h->tx_idx < h->tx_size){
			USART_DATA(usartx) = h->tx_buffer[h->tx_idx++];
			if (h->tx_idx==h->tx_size) {
				USART_CTL0(usartx) &= ~USART_CTL0_TBEIE;
				USART_CTL0(usartx) |=  USART_CTL0_TCIE;
			}
		}
	}
	if (stat & USART_STAT0_TC){
		USART_CTL0(usartx) &= ~USART_CTL0_TCIE;
		h->status = ERR_USART_OK; // трансфер завершен
#ifdef BOARD_USART0_TE// переделать!!!
		pio_reset_output(BOARD_USART0_TE);
#endif


		osSignalSet(h->owner, 1<<h->flag);
		osThreadNotify(h->owner);
	}
}
#ifdef BOARD_USART0
void USART0_IRQHandler(void)
{
	USARTn_IRQHandler(USART0, &usart_ports[0]);
}
#endif
#ifdef BOARD_USART1
void USART1_IRQHandler(void)
{
	USARTn_IRQHandler(USART1, &usart_ports[1]);
}
#endif
#ifdef BOARD_USART2
void USART2_IRQHandler(void)
{
	USARTn_IRQHandler(USART2, &usart_ports[3]);
}
#endif
