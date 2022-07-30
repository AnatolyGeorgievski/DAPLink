/*	\file i2c_env.c	Датчики окружения и EEPROM
 */
#include "board.h"
#include <cmsis_os.h>
#include <stdio.h>
#include <stdint.h>
#include "i2c.h"
#ifndef I2C0_SIGNAL
#define I2C_flag 0
#else
#define I2C_flag I2C0_SIGNAL
#endif
/*
 125°C 0 1111 1010 0FAh
  25°C 0 0011 0010 032h
 0.5°C 0 0000 0001 001h
   0°C 0 0000 0000 000h
−0.5°C 1 1111 1111 1FFh
 −25°C 1 1100 1110 1CEh
 −55°C 1 1001 0010 192h
*/

int lm75_read(uint32_t i2c, const uint8_t slave_addr, uint8_t reg, int16_t* value)
{
	uint8_t buf[2];
	int err=0;
	osEvent event;
	I2C_Transfer tr= {.address=0, .rx_buffer=NULL, .tx_buffer=NULL, .tx_size = 0, .rx_size=0 };
	tr.address = slave_addr;
	tr.tx_buffer = buf;
	tr.rx_buffer = buf;
	tr.tx_size = 1;
	tr.rx_size = 2;
	buf[0] = reg;
	I2C_send_recv(i2c, &tr);
	event = osSignalWait(1<<I2C_flag, 10);// ждем завершения операции
	if (event.status & osEventTimeout){
		printf("I2C Timeout wr\r\n");
		err++;
	} else {
		if (tr.status != 0) {// выделить ошибку или статус готовности
			printf("nACK");
			err++;
		} else *value = ((uint16_t)buf[0]<<8) | buf[1];
	}
	osSignalClear(osThreadGetId(),1<<I2C_flag);
	return err;	
}

/*! \brief запросить адрес EEPROM */
int at24xx_get_address(uint32_t i2c, const uint8_t slave_addr, uint8_t *address)
{
	int err=0;
	osEvent event;
	I2C_Transfer tr= {.address=0, .rx_buffer=NULL, .tx_buffer=NULL, .tx_size = 0, .rx_size=0 };
	tr.address = slave_addr;
	tr.rx_buffer = address;
	tr.rx_size = 1;
	I2C_send_recv(i2c, &tr);
	event = osSignalWait(1<<I2C_flag, 10);// ждем завершения операции
	if (event.status & osEventTimeout){
		printf("I2C Timeout wr\r\n");
		err++;
	} else {
		if (tr.status != 0) {// выделить ошибку или статус готовности
			printf("nACK");
			
		}
	}
	osSignalClear(osThreadGetId(),1<<I2C_flag);
	return err;	
}
// не отлажено, см документацию на AT24C02D
int at24xx_poll(uint32_t i2c, const uint8_t slave_addr)
{
	int err=0;
	osEvent event;
	I2C_Transfer tr= {.address=0, .rx_buffer=NULL, .tx_buffer=NULL, .tx_size = 0, .rx_size=0 };
	tr.address = slave_addr;
	do {
		I2C_send_recv(i2c, &tr);
		event = osSignalWait(1<<I2C_flag, 10);// ждем завершения операции
		if (event.status & osEventTimeout){
			printf("I2C Timeout wr\r\n");
			err++;
		} else {
			if (tr.status != 0) {// выделить ошибку или статус готовности
				err++; 
//				printf("nACK ");
				osDelay(1);
			} else {
				err=0;
			}
		}
		osSignalClear(osThreadGetId(),1<<I2C_flag);
	}while (err);
	return err;	
}
/*! \brief последовательное чтение из памяти */
int at24xx_read(uint32_t i2c, const uint8_t slave_addr, uint8_t address, uint8_t *buffer, uint32_t len)
{
	uint8_t buf[2];
	buf[0] = address;// адрес в памяти
	int err=0;
	osEvent event;
	I2C_Transfer tr= {.address=0, .rx_buffer=NULL, .tx_buffer=NULL, .tx_size = 0, .rx_size=0 };
	tr.address = slave_addr;
	tr.rx_buffer = buffer;
	tr.rx_size = len;
	tr.tx_buffer = buf;
	tr.tx_size = 0;
	I2C_send_recv(i2c, &tr);
	event = osSignalWait(1<<I2C_flag, 10);// ждем завершения операции
	if (event.status & osEventTimeout){
		printf("I2C Timeout wr\r\n");
		err++;
	} else {
		if (tr.status != 0) {// выделить ошибку или статус готовности
			printf("nACK");
		}
	}
	osSignalClear(osThreadGetId(),1<<I2C_flag);
	return err;	
}
/*! \brief запись страницы в память по установленному адресу

	\param i2c - порт I2C
	\param slave_addr адрес устройства на линии
	\param address адрес в памяти
	\param buffer должно резервироваться место под адрес
	\param len длина последовательности. Можно всю память вычерпать.
	
	\note Если указать нулевую длину, то команда установит указатель для последовательного чтения.
 */
int at24xx_write_page(uint32_t i2c, const uint8_t slave_addr, uint8_t address, uint8_t *buffer, uint32_t len)
{
	int err=0;
	osEvent event;
	I2C_Transfer tr= {.address=0, .rx_buffer=NULL, .tx_buffer=NULL, .tx_size = 0, .rx_size=0 };
	tr.address = slave_addr;
	buffer[0] = address;
	tr.tx_buffer = buffer;
	tr.tx_size = len+1;
	I2C_send_recv(i2c, &tr);
	event = osSignalWait(1<<I2C_flag, 10);// ждем завершения операции
	if (event.status & osEventTimeout){
		printf("I2C Timeout wr\r\n");
		err++;
	} else {
		if (tr.status != 0) {// выделить ошибку или статус готовности
			printf("nACK");
			err++;
		}
	}
	osSignalClear(osThreadGetId(),1<<I2C_flag);
	return err;	
}
