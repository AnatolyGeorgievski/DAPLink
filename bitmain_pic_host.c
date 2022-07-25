#include "bitmain_pic.h"
#include <cmsis_os.h>
#include <string.h>
#include <time.h>
enum {ERR_OK=0, ERR_FAIL=-1, ERR_BUSY=-2, ERR_CRC=-3 };
#define TEMP_REG_ADDRESS 0 // адрес датчика remote sensor на хеш-плате
#define PIC_HEART_BEAT_INTERVAL 2000
#define PIC_SENSORS_INTERVAL 1000
#define PIC_HEALTH_INTERVAL  1000
#define PIC_FLASH_POINTER_START_ADDRESS 0 // определить

static inline void timer_start(uint32_t *timer, uint32_t timestamp){
	*timer = timestamp;
}
// Такой способ запуска обеспечивает неизменную величину интервала, 
static inline void timer_restart(uint32_t *timer, uint32_t timestamp, uint32_t interval){
	*timer += interval;
}
static inline int timer_expired(uint32_t *timer, uint32_t timestamp, uint32_t interval){
	return (uint32_t)(timestamp - *timer)>=interval;
}

static inline int pic_send_recv(void* h, uint8_t *data, uint8_t len, uint8_t resp_len)
{
	Board_t* brd = h;
	return brd->send_recv(brd->hdl, data, len, resp_len);
}
/*! \brief формирование и отсылка запроса на чтение
	\param[in] data - буфер для заполнения и отсылки пакета
	\param[in] len - длина отклика без учета шапки и контрольной суммы
	\param[out] resp -- буфер для заполнения данными из отклика
	Команда возварщает результат опроса на том же буфере
 */
static int pic_cmd_read(void* brd, uint8_t cmd, uint8_t *data, uint8_t len, uint8_t *resp)
{
	data[0] = 0x55;
	data[1] = 0xAA;
	data[2] = 4;
	data[3] = cmd;
	uint16_t fcs = cmd+4;
	data[4] = fcs>>8;
	data[5] = fcs;

	int resp_len = pic_send_recv(brd, data, 4, len+5);
	int res = (len+5 == resp_len && data[0]==resp_len && data[1]==cmd);
	if (!res) return ERR_FAIL;
	fcs = fcs_update(cmd+len+5, data+2, len+1);
	res = (fcs == be16toh(*(uint16_t*)(data+len+3)));
	if (!res) return ERR_CRC;
	if (data[2]!=0x01) return ERR_BUSY;
	if (resp) {
		int i;
		for(i=0;i<len; i++)
			resp[i] = data[i+3]; 
	}
	return ERR_OK;
}
static int pic_cmd_send(void* brd, uint8_t cmd, uint8_t *data, uint8_t len)
{
	data[0] = 0x55;
	data[1] = 0xAA;
	data[2] = len;
	data[3] = cmd;
	uint16_t fcs = fcs_update(cmd+len, data+4, len-2);
	data[len++] = fcs>>8;
	data[len++] = fcs;

	int resp_len = pic_send_recv(brd, data, len, 2);
	return !(2 == resp_len && data[0]==cmd && data[1] == 0x01);
}
/*! отсылка короткой команды без аргументов */
static int pic_cmd_send0(void* brd, uint8_t cmd)
{
	const uint8_t len = 0;
	uint8_t data[len+6];
	return pic_cmd_send(brd, cmd, data, len);
}
int pic_set_remote_sensor(void* brd, uint8_t sensor_adr, uint8_t sensor_reg)
{
	const uint8_t len = 2;
	uint8_t data[len+6];
	*(uint8_t*)(data+PIC_HDR+0) = sensor_adr;
	*(uint8_t*)(data+PIC_HDR+1) = sensor_reg;
	return pic_cmd_send(brd, SET_PIC_FLASH_POINTER, data, len);
}

int pic_get_remote_sensor(void* brd, uint8_t sensor_addr, uint16_t* sensor_val)
{
	const uint8_t len = 6;
	const uint8_t cmd = GET_REMOTE_SENSOR;
	const int rsp_len = 2;
	uint8_t data[len+6];
	data[0] = 0x55;
	data[1] = 0xAA;
	data[2] = len;
	data[3] = cmd;
	data[4] = sensor_addr;// LM75 = 0x48...0x4B
	data[5] = rsp_len;// длина отклика
	uint16_t fcs = fcs_update(cmd+len, data+4, len-2);
	data[6] = fcs>>8;
	data[7] = fcs;
	int res = pic_send_recv(brd, data, len, PIC_RSP+PIC_FCS+rsp_len);
	if (res == SUCCESS)
		*sensor_val = be16toh(*(uint16_t*)(data+PIC_RSP));
	return res;
}
int pic_set_flash_pointer(void* brd, uint16_t flash_addr)
{
	const uint8_t len = 2;
	uint8_t data[len+6];
	*(uint16_t*)(data+4) = htobe16(flash_addr);
	return pic_cmd_send(brd, SET_PIC_FLASH_POINTER, data, len);
}
int pic_read_flash_pointer(void* brd, uint16_t* flash_addr)
{
	const uint8_t len = 2;
	uint8_t data[len+6];
	int res = pic_cmd_read(brd, GET_PIC_FLASH_POINTER, data, len, NULL);
	if (res != SUCCESS)	return res;
	*flash_addr = be16toh(*(uint16_t*)(data+3));
	return res;
}
int pic_send_data_to_pic(void* brd, uint8_t *buf)
{
	const uint8_t len = 16;
	uint8_t data[len+6];
	__builtin_memcpy(data+PIC_HDR, buf, len);
	return pic_cmd_send(brd, SEND_DATA_TO_PIC, data, len);
}
int pic_read_data_from_flash(void* brd, uint8_t * buf)
{
	const uint8_t len = 16;
	uint8_t data[len+6];
	return pic_cmd_read(brd, READ_DATA_FROM_PIC_FLASH, data, len, buf);
}
static inline int pic_erase_flash(void* brd) {
	return pic_cmd_send0(brd, ERASE_PIC_FLASH);
}
static inline int pic_write_data_into_flash(void* brd) {
	return pic_cmd_send0(brd, WRITE_DATA_INTO_FLASH);
}
static inline int pic_jump_from_loader_to_app(void* brd) {
	return pic_cmd_send0(brd, JUMP_FROM_LOADER_TO_APP);
}
static inline int pic_reset(void* brd) {
	return pic_cmd_send0(brd, RESET_PIC);
}
int pic_set_voltage(void* brd, uint8_t voltage)
{
	const uint8_t len = 1;
	uint8_t data[len+6];
	data[4] = voltage;
	return pic_cmd_send(brd, SET_VOLTAGE, data, len);
}
int pic_set_voltage_setting_time(void* brd, uint16_t settling_time)
{
	const uint8_t len = 2;
	uint8_t data[len+6];
	return pic_cmd_send(brd, SET_VOLTAGE, data, len);
}
int pic_set_hash_board_id_number(void* brd, const uint8_t *buf)
{
	const uint8_t len = 12;
	uint8_t data[len+6];
	__builtin_memcpy(data+PIC_HDR, buf, len);
	return pic_cmd_send(brd, SET_HASH_BOARD_ID, data, len);
}
int pic_get_hash_board_id_number(void* brd, uint8_t* buf)
{
	const uint8_t len = 12;// длинный идентификатор 12 символов
	uint8_t data[len+5];
	return pic_cmd_read(brd, READ_HASH_BOARD_ID, data, len, buf);
}
int pic_get_software_version(void* brd, uint8_t* buf)
{
	const uint8_t len = 1;
	uint8_t data[len+5];
	return pic_cmd_read(brd, READ_SOFTWARE_VERSION, data, len, buf);
}
int pic_get_voltage(void* brd, uint8_t* buf)
{
	const uint8_t len = 1;
	uint8_t data[len+5];
	return pic_cmd_read(brd, GET_VOLTAGE, data, len, buf);
}
/*! в старой версии эта команда не возвращает состояние */
int pic_heart_beat(void* brd)
{
	const uint8_t len = 1;// возварщает один байт 00
	uint8_t data[len+5];
	return pic_cmd_read(brd, SEND_HEART_BEAT, data, len, data+3);
}
int pic_enable_voltage(void* brd)
{
	const uint8_t len = 1;
	uint8_t data[len+6];
	*(data+PIC_HDR) = 1;
	return pic_cmd_send(brd, ENABLE_VOLTAGE, data, len);
}
int pic_disable_voltage(void* brd)
{
	const uint8_t len = 1;
	uint8_t data[len+6];
	*(data+PIC_HDR) = 1;
	return pic_cmd_send(brd, ENABLE_VOLTAGE, data, len);
}
int pic_save_freq(void* brd, uint16_t freq)
{
	const uint8_t len = 2;
	uint8_t data[len+6];
	*(uint16_t*)(data+4) = htobe16(freq);
	return pic_cmd_send(brd, SAVE_FREQ , data, len);
}
/* обновление прошивки контроллера */ 
int pic_upgrade_firmware(void* brd, uint8_t *data, size_t data_len)
{
	pic_erase_flash(brd);
	pic_set_flash_pointer(brd, PIC_FLASH_POINTER_START_ADDRESS);
	const int chunk=16;
	int i;
	for(i=0; i<(data_len/chunk); i++)
    {
        pic_send_data_to_pic(brd, data+i*chunk);
        pic_write_data_into_flash(brd);
    }
	size_t tail = data_len - i*chunk; 
	if (tail) {// лучше избавиться от этого хвоста
		uint8_t buf[chunk];
		memcpy(buf, data+i*chunk, tail);
		memset(buf+tail, 0xFF, chunk-tail);// заполнение
        pic_send_data_to_pic(brd, buf);
        pic_write_data_into_flash(brd);
	}
	pic_reset(brd);
}
/*! */
int pic_host_process(void* arg)
{
	Board_t * board = arg;
	time_t timer_sensor, timer_health, timer_heart_beat;
	time_t timestamp;
	timestamp = clock();
	timer_start(&timer_sensor, timestamp); 
	timer_start(&timer_health, timestamp); 
	timer_start(&timer_heart_beat, timestamp); 
	osEvent event;
	osThreadId self = osThreadGetId();
	int idx=0;// перебор
	while (1) {
		uint32_t SignalMask=(1U<<board->request_flag) | (1<<Alert_flag);
		event = osSignalWait(SignalMask,100);
		if (event.status == osEventTimeout) {
		} else 
		if (event.value.signals & (1<<Alert_flag)) { // внешнее прерывание, сигнал для привлечения внимания
	
		} else 
		if (event.value.signals & (1U<<board->request_flag)) {
			switch (board->cmd) {// обработка команд
			case CMD_ENABLE:
				pic_enable_voltage(board);
				board->state = WORK;
				break;
			case CMD_VOLTAGE_SETTING_TIME:
				pic_set_voltage_setting_time(board, board->voltage_settling_time);// volatile uint16_t volatge_setting (ms)
				break;
			case CMD_SET_VOLTAGE:
				pic_set_voltage(board, board->voltage);// volatile uint16_t voltage (mV)
				break;
			case CMD_GET_VOLTAGE:
				pic_get_voltage(board, board->response);// заполняет буфер отклика
				break;
			case CMD_GET_ID:
				pic_get_hash_board_id_number(board, board->response);
				board->resp_len = 12;
				break;
			case CMD_DISABLE:
				pic_disable_voltage(board);
				board->state = IDLE;
				break;
			default:
				break;
			}
			osSignalClear(self, (1U<<board->request_flag));
			osSignalSet(board->resp_pid, (1U<<board->resp_flag));// уведомить процесс о завершении
		}
		timestamp = clock();
		// выполнить полезную работу
		// сердечный ритм чтобы контроллер чувствовал свою необходимость и не отключался
		if (timer_expired(&timer_heart_beat, timestamp, PIC_HEART_BEAT_INTERVAL)){
			timer_restart(&timer_heart_beat, timestamp, PIC_HEART_BEAT_INTERVAL);
			if(board->state==WORK && pic_heart_beat(board) )
			{// если не отвечает
				
			}
		}
		// выполнить опрос датчиков окружения
		if (timer_expired(&timer_sensor, timestamp, PIC_SENSORS_INTERVAL)){
			timer_restart(&timer_sensor, timestamp, PIC_SENSORS_INTERVAL);
			if (board->sensor_count) {// один датчик за цикл
				pic_set_remote_sensor(board, board->sensor[idx].addr, TEMP_REG_ADDRESS);
				pic_get_remote_sensor(board, board->sensor[idx].addr, &board->sensor[idx].value);
				if(++idx==board->sensor_count) idx = 0;
			}
		}
		// мониторинг параметров питания и теплоносителя
		if (timer_expired(&timer_health, timestamp, PIC_HEALTH_INTERVAL)){
			timer_restart(&timer_health, timestamp, PIC_HEALTH_INTERVAL);
#if 0
//			if (board->capabilities & CAP_VOLTAGE) 	pic_get_voltage (board, &board->power[0]);// запрашиваем напряжение и ток
			if (board->capabilities & CAP_TEMP) 	pic_get_temp    (board, &board->temp [0]);// запрашиваем температуру теплоносителя
			if (board->capabilities & CAP_FANS) 	pic_get_fans    (board, &board->fans [0]);// запрашиваем обороты вентиляторов
			if (board->capabilities & CAP_RELAYS) 	pic_get_relays  (board, &board->relay   );// запрашиваем состояние релейных контактов
			// все это можно запросить одной командой, но для этого нужно поддерживать структурированные форматы данных
#endif
		}
	}
	//if (board->store) 
	{// сохранять настройки во флеш
		
	}
}
void pic_host_start_sequence(Board_t * board)
{
	// сброс в начальное состояние, состояние загрузчика
	board->reset(board); // это ресет для цепочки ASIC, может быть вынести его
	pic_reset(board);
	//  обновление прошивки
#if 0// дописать
	uint8_t revision_id = 0;

	if (pic_get_software_version(board, &revision_id) && revision_id < firmware_id){
		pic_upgrade_firmware(board, firmware_bin, firmware_size);
		pic_reset(board);
	}
#endif
	// переход в состояние приложения
	pic_jump_from_loader_to_app(board);
	pic_set_voltage (board, board->voltage);
	pic_set_voltage_setting_time(board, board->voltage_settling_time);
	pic_enable_voltage(board);
}