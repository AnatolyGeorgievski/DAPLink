#ifndef USART_H
#define USART_H
#include "board.h"
#include <cmsis_os.h>

enum { 
	ERR_USART_OK=0,
	ERR_USART_BUSY,		// в процессе обработки
	ERR_USART_PARITY, 	// ошибка проверки четности
	ERR_USART_FRAME, 
	ERR_USART_NOISE, 
	ERR_USART_OVERRUN, 
	ERR_USART_TIMEOUT, 
	ERR_USART_COUNT // общее число кодов
};

typedef struct _DataUnit osTransfer;
// вместе образуют API драйвера
void* usart_open(uint32_t uart_id, int32_t flag);
void  usart_send(void* h, void* data, uint16_t size);
osTransfer* usart_recv(void* h, void* data, uint16_t size);
void  usart_close(void* h);
uint16_t usart_status(void* h);// есть такой вариант представления ошибки
/*! 
	\arg USART_DENT_ENABLE -- выключить 
	\arg USART_DENT_DISABLE
*/
static inline void usart_dma_transmit_enable(uint32_t uart_id, uint32_t dmacmd)
{
    USART_CTL2(uart_id) =(USART_CTL2(uart_id) & ~USART_CTL2_DENT)|dmacmd;
}
static inline void usart_dma_receive_enable(uint32_t uart_id, uint32_t dmacmd)
{
    USART_CTL2(uart_id) =(USART_CTL2(uart_id) & ~USART_CTL2_DENR)|dmacmd;
}
#endif // USART_H
