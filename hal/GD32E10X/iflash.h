#ifndef IFLASH_H
#define IFLASH_H
#include "board.h"
#include <cmsis_os.h>
enum { 
	ERR_FLASH_OK=0,
	ERR_FLASH_BUSY,		// в процессе обработки
	ERR_FLASH_PAGE, 	// ошибка очистки страницы возникает когда память не инициализирована 0xFFFF
	ERR_FLASH_WRP, 		// не снята блокировка записи
	ERR_FLASH_ALIGN,	// ошибка выравнивания данных
	ERR_FLASH_COUNT // общее число кодов
};

// вместе образуют API драйвера
void* iflash_open(uint32_t block_id, int32_t flag);
void  iflash_send(void* h, void* data, uint16_t size);
void* iflash_recv(void* h, uint32_t block_id);
void  iflash_close(void* h);
uint16_t iflash_status(void* h);// есть такой вариант представления ошибки
void iflash_bootloader(uint32_t offset, size_t size); 
#endif // IFLASH_H
