#ifndef BITMAIN_PIC_H
#define BITMAIN_PIC_H

#include <stdint.h>
#include <stddef.h>
/*! Новый API PIC16F1704 */
#define SET_PIC_FLASH_POINTER                       0x01
#define SEND_DATA_TO_PIC                            0x02    // just send data into pic's cache
#define READ_DATA_FROM_PIC_FLASH                    0x03
#define ERASE_PIC_FLASH                             0x04    // erase 32 bytes one time
#define WRITE_DATA_INTO_FLASH                       0x05    // tell pic write data into flash from cache
#define JUMP_FROM_LOADER_TO_APP                     0x06
#define RESET_PIC                                   0x07
#define GET_PIC_FLASH_POINTER                       0x08
#define SET_VOLTAGE                                 0x10
#define SET_HASH_BOARD_ID                           0x12
#define READ_HASH_BOARD_ID                          0x13
#define ENABLE_VOLTAGE                              0x15
#define SEND_HEART_BEAT                             0x16
#define READ_SOFTWARE_VERSION 		                0x17
#define GET_VOLTAGE                                 0x18
#define WR_TEMP_OFFSET_VALUE                        0x22
#define RD_TEMP_OFFSET_VALUE                        0x23
#define SAVE_FREQ                                   0x24
#define READ_OUT_FREQ                               0x25
#define SET_REMOTE_SENSOR                           0x3B
#define GET_REMOTE_SENSOR                           0x3C
//
#define PIC_COMMAND_1 0x55
#define PIC_COMMAND_2 0xAA
#define PIC_HDR (4) // размер пакета без данных, преамбула(2), код, длина
#define PIC_RSP (3) // шапка отклика, содержит длину, код операции и статус
#define PIC_FCS (2) // размер контрольной суммы кадра
#define PIC_PKT (PIC_HDR+PIC_FCS) // размер пакета без данных

/*! \brief Расчет контрольной суммы кадра -- арифметическая сумма байт.
	начальное значение CMD+Length
 */
static inline uint16_t fcs_update(uint16_t fcs, uint8_t *data, uint8_t len)
{
	int i;
	for (i=0; i< len; i++)
		fcs += data[i];
	return fcs;
}

#ifndef be16toh
#define be32toh(x) __builtin_bswap32(x) 
#define htobe32(x) __builtin_bswap32(x) 
#define be16toh(x) __builtin_bswap16(x) 
#define htobe16(x) __builtin_bswap16(x) 
#endif

#include <cmsis_os.h>

typedef struct _Board Board_t;
struct _Board {
	enum {IDLE, WORK} state;
	uint8_t slave_address;	//!< Адрес контроллера хеш-платы на линии i2c
	uint32_t capabilities;	//!< Флаги поддерживаемых команд
	uint32_t port; 			//!< идентификатор UART порта USART0...USARTn 
	uint32_t baudrate;		//!< Частота передачи данных по интерфейсу UART
	uint8_t voltage;// некоторое число
	uint8_t voltage_settling_time;// время выхода на заданное напряжение
	uint8_t sensor_count;
	struct {
		uint8_t addr;
		uint8_t res0;
		uint16_t value;
	} sensor[8];

	// обмен запросами между процессами
	uint8_t cmd;		//!< код операции 
	uint8_t request_flag;	//!< номер бита флаг отклика
	uint8_t resp_flag;	//!< номер бита флаг отклика
	uint8_t resp_len;	//!< длина отклика
	uint8_t * response;	//!< буфер для ответа, подставляет 
	osThreadId owner;	//!< Идентификатор процесса оброботки протокола
	osThreadId resp_pid;	//!< Источник запроса, идентификатор процесса
		
	
	// Драйвер порта
	void* hdl; // указатель на порт
	void (*reset)(void *h);
	int  (*send_recv)(void *h, uint8_t *buffer, size_t tx_size, size_t rx_size);
	//Pin_t present;
};
enum {
	Alert_flag, 
	Request_flag, 
	Confirm_flag, 
};
// Система команд 
enum _PIC_CMD {
	CMD_ENABLE,
	CMD_DISABLE,
	CMD_VOLTAGE_SETTING_TIME,
	CMD_SET_VOLTAGE,
	CMD_GET_VOLTAGE,
	CMD_GET_ID,
	CMD_SET_ID
};
#endif //BITMAIN_PIC_H
