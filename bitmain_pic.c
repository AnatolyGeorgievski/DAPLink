#include "board.h"
#include <cmsis_os.h>
#include <stdio.h>
#include <string.h>
#include "pio.h"
#include "rtc.h"
#include "config.h"
#include "bitmain_pic.h"

//#include "adc.h"
//#include "errno.h"

#define PIC_HEART_BEAT_INTERVAL				10 // s
#define SEND_COMMAND_1                      0x55
#define SEND_COMMAND_2                      0xAA
#define CMD_JUMP_FROM_LOADER_TO_APP         0x06
#define CMD_RESET                           0x07

#define CMD_SET_VOLTAGE                     0x10
#define CMD_SET_VOLTAGE_SETTING_TIME        0x11
#define CMD_SET_HASH_BOARD_ID               0x12
#define CMD_GET_HASH_BOARD_ID               0x13
#define CMD_SET_HOST_MAC_ADDRESS            0x14
#define CMD_ENABLE_VOLTAGE                  0x15
#define CMD_HEART_BEAT                      0x16
#define CMD_GET_SOFTWARE_VERSION            0x17
#define CMD_GET_VOLTAGE                     0x18
#define GET_VOLTAGE_SETTING_TIME            0x19

#define PIC_FLASH_SECTOR_LENGTH             32
#define PIC_SOFTWARE_VERSION_LENGTH         1
#define PIC_VOLTAGE_TIME_LENGTH             6

void pic_send_command();
void pic_send_heart_beat();
void pic_jump_from_loader_to_app();
void pic_set_flash_pointer(uint8_t flash_addr_h, uint8_t flash_addr_l);
void pic_read_flash_pointer(uint8_t *read_back_flash_addr_h, uint8_t *read_back_flash_addr_l);
void pic_send_data_to_pic(uint8_t *data);
void pic_write_data_into_flash();
void pic_read_data_from_pic_flash(uint8_t *data);
void pic_reset();
void pic_set_voltage_setting_time(uint8_t *time);
void pic_get_voltage_setting_time(uint8_t *time);
void pic_set_voltage(uint8_t volt_pic_val);
void pic_get_voltage(uint8_t *volt_pic_val);
void pic_set_hash_board_id_number(uint8_t *id);
void pic_get_hash_board_id_number(uint8_t *id);
void pic_enable_voltage();
void pic_disable_voltage();
void pic_read_pic_software_version(uint8_t *version);
void pic_erase_pic_flash();
void pic_erase_flash_all();
void update_pic_program();
void flash_pic_freq(unsigned char *buf1);

static	inline int timer_expired(uint32_t *timer, uint32_t timestamp, uint32_t interval)
{
	return (uint32_t)(timestamp - *timer) >= interval;
}
static	inline void timer_start(uint32_t *timer, uint32_t timestamp)
{
	*timer = timestamp;
	
}
static	inline void timer_restart(uint32_t *timer, uint32_t timestamp, uint32_t interval)
{
	*timer += interval;
}

static enum {
	PIC_STATE_RESET,
	PIC_STATE_LOADER,
	PIC_STATE_APP
} device_state=0;
uint8_t device_id = 3;
uint8_t device_name[] CONFIG_ATTR = BOARD_NAME;
uint8_t device_voltage=0;
uint32_t timer_heart_beat=0;
/*! Обработка команд протокола в режиме "загрузчик"
	\return длина отклика
 */
static int pic_cmd_loader_process(uint8_t *pdu, int len, uint8_t* tr)
{
	int res = 0;
	switch (pdu[0]){
	default:
		res = -1;
	}
	return res;
}
/* Обработка команд протокола в режиме Приложение */
static int pic_cmd_app_process(uint8_t *pdu, int len, uint8_t* resp)
{
	int res = 0;
	uint32_t timestamp;
	switch (pdu[0]){
	case CMD_JUMP_FROM_LOADER_TO_APP:// ничего не происходит, но теперь нельзя прошивать
		device_state = PIC_STATE_APP;
		break;
	case CMD_HEART_BEAT:
		timestamp = time(NULL);
		timer_start(&timer_heart_beat, timestamp);
		break;
	case CMD_ENABLE_VOLTAGE:
		if (pdu[1])
			pio_set_output(DO_PIC_ENABLE);
		else 
			pio_reset_output(DO_PIC_ENABLE);
		break;
	case CMD_SET_HASH_BOARD_ID:
		len --;
		if (len>12) len = 12;
		memcpy(device_name, pdu+1, len);
		break;
	case CMD_GET_HASH_BOARD_ID:
		memcpy(resp, device_name, 12);
		res = 12;
		break;
	case CMD_RESET:
		pio_reset_output(DO_PIC_ENABLE);
		device_state = PIC_STATE_RESET;
		// do software reset
		break;
	case CMD_SET_VOLTAGE:
		device_voltage = pdu[1];
		break;
	case CMD_GET_VOLTAGE:
		resp[0] = device_voltage;
		res = 1;
		break;
	case CMD_GET_SOFTWARE_VERSION:
		resp[0] = device_id;
		break;
	default:
		res = -1;// ошибка
		break;
	}
	return res;
}
/*
int pic_cmd_process(uint8_t *pdu, int len, uint8_t* resp)
{
	if (device_state == PIC_STATE_APP)
		return pic_cmd_app_process(pdu, len, resp);
	else 
		return pic_cmd_loader_process(pdu, len, resp);
}*/