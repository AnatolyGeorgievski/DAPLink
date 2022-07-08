/* crc.h */
#ifndef CRC_H
#define CRC_H
#include <stdint.h>
/* 
CRC-32/CKSUM

	width=32 poly=0x04c11db7 init=0x00000000 refin=false refout=false xorout=0xffffffff check=0x765e7680 residue=0xc704dd7b name="CRC-32/CKSUM"

    Alias: CKSUM, CRC-32/POSIX

CRC-16/CCITT-FALSE
    width=16 poly=0x1021 init=0xffff refin=false refout=false xorout=0x0000 check=0x29b1 name="CRC-16/CCITT-FALSE"

\see стандарт ITU-T Rec. X.25. там описан полином и способ вычисления FCS — контрольной суммы типа CRC16. Отсюда взялось название CRC16_ITU.

Data CRC-16/X-25: X16 + X12 + X5 + 1
	width=16 poly=0x1021 init=0xffff refin=true refout=true xorout=0xffff check=0x906e name="X-25"

XMODEM Alias: ZMODEM, CRC-16/ACORN
    width=16 poly=0x1021 init=0x0000 refin=false refout=false xorout=0x0000 check=0x31c3 name="XMODEM"

MODBUS
    width=16 poly=0x8005 init=0xffff refin=true refout=true xorout=0x0000 check=0x4b37 name="MODBUS"

    CRC presented low byte first.

CRC-15/CAN Alias: CRC-15
    width=15 poly=0x4599 init=0x0000 refin=false refout=false xorout=0x0000 check=0x059e name="CRC-15"

    Robert Bosch GmbH (September 1991), CAN 2.0 Specification

CRC-8/SMBUS Alias: CRC-8
    width=8 poly=0x07 init=0x00 refin=false refout=false xorout=0x00 check=0xf4 name="CRC-8"

Header CRC-8/BACnet: X8 + X7 + 1
	width=8 poly=0x81 init=0xff refin=true refout=true xorout=0xff check=0x89 name="CRC-8/BAC"
	
CRC-7
    width=7 poly=0x09 init=0x00 refin=false refout=false xorout=0x00 check=0x75 name="CRC-7"

    Used in the MultiMediaCard interface.
    JEDEC Standard JESD84-A441 (March 2010)
	
CRC-5/BITMAIN
    width=5 poly=0x05 init=0x1f refin=false refout=false xorout=0x0 check=0x0F name="CRC-5/BITMAIN"
 */
#define CRC32_POLY 0x04C11DB7U
#define CRC16_POLY 0x1021U
#define CRC16M_POLY 0x8005U
#define CRC8_POLY 0x07U
#define CRC8BAC_POLY 0x81U
#define CRC5_POLY 0x05U
uint8_t crc5_btc(uint8_t crc, uint8_t *data, size_t data_len);
uint8_t crc8_bac(uint8_t crc, uint8_t *data, size_t data_len);
uint8_t crc8_smbus(uint8_t crc, uint8_t *data, size_t data_len);
uint16_t crc16_itu(uint16_t crc, uint8_t *data, size_t data_len);
/* Функция предназначена для контроля целостности при записи/чтении флеш. 

 */
static inline uint32_t crc32_eth(uint32_t crc, uint8_t *data, size_t data_len)
{
//	CRC_IDATA = crc;
	CRC_CTL   = CRC_CTL_RST;
//	CRC_POLY  = (uint32_t)CRC32_POLY;// REV_I REV_O -- может устанавливать после RST
	int i = data_len>>2;
	if (i) do {
		REG32(CRC) = __REV(*(uint32_t*)data);
		data+=4;
	} while(--i);
//	for (i=0; i< data_len; i+=4) 
//		REG32(CRC) = data[i];
	return REG32(CRC);
}
static inline uint32_t crc32_cksum(uint32_t crc, uint8_t *data, size_t data_len)
{
//	CRC_IDATA = crc;
	CRC_CTL   = CRC_CTL_RST;
//	CRC_POLY  = (uint32_t)CRC32_POLY;// REV_I REV_O -- может устанавливать после RST
	REG32(CRC) = ~0UL;// после этой операции CRC_INIT==0
	int i = data_len>>2;
	if (i) do {
		REG32(CRC) = __REV(*(uint32_t*)data);
		data+=4;
	} while(--i);
/*	int i;
	for (i=0; i< data_len; i+=4) 
		REG32(CRC) = __REV(*(uint32_t*)(data+i));  */
	return REG32(CRC);
}
#endif//CRC_H
