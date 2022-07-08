/*! \ingroup _hal 
	\defgroup _hal_ident Идентификация системы 

	\see RM0091 Device electronic signature
	\see RM0313 STM32F373 Device electronic signature
	\see RM0366 STM32F301 Device electronic signature
	\see RM0365 STM32F302 Device electronic signature
	\{
*/


#ifndef IDENT_H
#define IDENT_H
#include "board.h"
#include "gd32e10x_rcu.h"
#include <stdio.h> // printf
// Cortex-M3
#define DEV_ID_STM32F10x_LD		0x412
#define DEV_ID_STM32F10x_MD		0x410
#define DEV_ID_STM32F10x_HD		0x414
#define DEV_ID_STM32F107		0x418
#define DEV_ID_STM32F10x_XL		0x430
#define DEV_ID_STM32F100xx_MD	0x420
#define DEV_ID_STM32F100xx_HD	0x428
// Cortex-M4 HWDiv DSP FPU MPU 
#define DEV_ID_STM32F407 		0x413
#define DEV_ID_STM32F427 		0x419

#define DEV_ID_STM32F373 		0x432
#define DEV_ID_STM32F301 		0x439
#define DEV_ID_STM32F302x6_8	0x439
#define DEV_ID_STM32F302xB_C	0x422 
#define DEV_ID_STM32F302xD_E	0x446
#define DEV_ID_STM32F33x		0x438
#define DEV_ID_STM32F401xB_C 	0x423
#define DEV_ID_STM32F401xD_E 	0x433
#define DEV_ID_STM32F410		0x458
#define DEV_ID_STM32F411xC_E 	0x431
#define DEV_ID_STM32F446	 	0x421
#define DEV_ID_STM32F469	 	0x434
#define DEV_ID_STM32L4x6		0x415
#define DEV_ID_STM32L41xxx 		0x464
#define DEV_ID_STM32L42xxx 		0x464
#define DEV_ID_STM32L43xxx 		0x435
#define DEV_ID_STM32L44xxx 		0x435
#define DEV_ID_STM32L45xxx 		0x462
#define DEV_ID_STM32L46xxx 		0x462
// Cortex-M0 
#define DEV_ID_STM32F05x	 	0x440
#define DEV_ID_STM32F09x	 	0x442
#define DEV_ID_STM32F03x	 	0x444
#define DEV_ID_STM32F04x	 	0x445
#define DEV_ID_STM32F07x	 	0x448

#define DEV_ID_STM32G431 		0x468// Category 2 devices 
#define DEV_ID_STM32G474		0x469// Category 3 devices 
#define DEV_ID_STM32G491		0x479// Category 4 devices 
#define DEV_ID_STM32L47_L48XX	0x415
#define DEV_ID_STM32L43_L44XX	0x435
#define DEV_ID_STM32G05_G06XX	0x456
#define DEV_ID_STM32G07_G08XX	0x460
#define DEV_ID_STM32L49_L4AXX	0x461
#define DEV_ID_STM32L45_L46XX	0x462
#define DEV_ID_STM32L41_L42XX	0x464
#define DEV_ID_STM32G03_G04XX	0x466
#define DEV_ID_STM32G0B_G0CXX	0x467
#define DEV_ID_STM32G43_G44XX	0x468
#define DEV_ID_STM32G47_G48XX	0x469
#define DEV_ID_STM32L4R_L4SXX	0x470
#define DEV_ID_STM32L4P_L4QXX	0x471
#define DEV_ID_STM32L55_L56XX	0x472
#define DEV_ID_STM32G49_G4AXX	0x479
#define DEV_ID_STM32U57_U58XX	0x482
#define DEV_ID_STM32WB1XX		0x494
#define DEV_ID_STM32WB5XX		0x495
#define DEV_ID_STM32WB3XX		0x496
#define DEV_ID_STM32WLE_WL5XX	0x497
#define DEVID_STM32H74_H75XX    0x450
#define DEVID_STM32H7A_H7BXX    0x480
#define DEVID_STM32H72_H73XX    0x483
/**
 * Cast a member of a structure out to the containing structure.
 * @param ptr The pointer to the member.
 * @param type The type of the container struct this is embedded in.
 * @param member The name of the member within the struct.
 *
 * This is a mechanism which is used throughout the Linux kernel.
 */
#define container_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (void *) ( (char *)__mptr - offsetof(type,member) ) );})

enum cortex_m_partno {
	CORTEX_M_PARTNO_INVALID,
	CORTEX_M0_PARTNO   = 0xC20,
	CORTEX_M1_PARTNO   = 0xC21,
	CORTEX_M3_PARTNO   = 0xC23,
	CORTEX_M4_PARTNO   = 0xC24,
	CORTEX_M7_PARTNO   = 0xC27,
	CORTEX_M0P_PARTNO  = 0xC60,
	CORTEX_M23_PARTNO  = 0xD20,
	CORTEX_M33_PARTNO  = 0xD21,
	CORTEX_M35P_PARTNO = 0xD31,
	CORTEX_M55_PARTNO  = 0xD22,
};

// 1.5.1. Memory density information
#define FLASHSIZE_BASE 0x1FFFF7E0UL 
// 1.5.2. Unique device ID (96 bits)
#define UID_BASE 0x1FFFF7E8
// 0x1FFFF7AC
#define DBGMCU_IDCODE_DEV_ID 0xFFF
#define DBGMCU_IDCODE_REV_ID 0xFFFF0000
// DBGMCU_IDCODE
/*! \brief идентификация системы
	
	распознается размер флеша
	уникальный идентификатор 96-бит
 */
static inline 
uint32_t sys_flash_size() {
	return *((uint32_t*)FLASHSIZE_BASE) & 0xFFFF; // размер флеша в kB
}
//адрес регистра DBGMCU_IDCODE 0xE0042000
static inline void r3_sys_ident()
{
	uint16_t dev_id = DBG_ID & DBGMCU_IDCODE_DEV_ID;
	uint16_t rev_id =(DBG_ID & DBGMCU_IDCODE_REV_ID)>>16;// A,B...
	char *str;
	switch (dev_id){
	case 0x410: 
		switch (rev_id) {
		case 0x1303: /* gd32f1x0 */
			str = "GD32F1X0";
			break;
		case 0x1704: /* gd32f3x0 */
			str = "GD32F3X0";
			break;
		case 0x1712: /* gd32e23x */
			str = "GD32E103xB";
			break;
		default: /* stm32f101/2/3 medium-density */
			str = "STM32F10x_MD";
			break;
		}
		break;
	case DEV_ID_STM32F373:	str = "STM32F373"; break;
	case DEV_ID_STM32F301:	str = "STM32F301x4-x6-x8/F302x4-x6-x8/F318xx"; break;
	case DEV_ID_STM32F302xB_C:	str = "STM32F302xB-xC"; break;
	case DEV_ID_STM32F302xD_E:	str = "STM32F302xD-xE"; break;
	default:	str = "";  break;
	}
	const uint32_t *UniqueID = ((uint32_t*)UID_BASE);
	uint32_t flash_size = *((uint32_t*)FLASHSIZE_BASE) & 0xFFFF; // размер флеша в kB
	printf("CPUID: 0x%08X\r\n", SCB->CPUID);
	//printf("CONTROL: 0x%08X\r\n", __get_CONTROL());
	printf("%s DevId: 0x%03x, RevId: 0x%04x, Flash: %dkB\r\n", str, dev_id, rev_id, flash_size);
	printf("UniqueID: 0x%08X %08X %08X\r\n", (unsigned int)UniqueID[2], (unsigned int)UniqueID[1], (unsigned int)UniqueID[0]);
}
/*! \brief идентификация типа загрузки и причины перезагрузки 
*/
static inline void r3_reset_ident()
{
	uint32_t reset_type = RCU_RSTSCK; // reset source / clock
	char const*str;
	if (reset_type & RCU_RSTSCK_LPRSTF){
		str = "low power";
	} else 
	if (reset_type & RCU_RSTSCK_SWRSTF){
		str = "software";
	} else 
	if (reset_type & RCU_RSTSCK_PORRSTF){
		str = "power on";
	} else 
	if (reset_type & RCU_RSTSCK_FWDGTRSTF){
		str = "free watchdog";
	} else 
	if (reset_type & RCU_RSTSCK_EPRSTF){
		str = "pin";
	} else 
	if (reset_type & RCU_RSTSCK_WWDGTRSTF){
		str = "window watchdog";
	} else {
		str = "undefined";
	}
	printf("%s reset\r\n", str);
	// очистить признак
	RCU_RSTSCK |= RCU_RSTSCK_RSTFC;
}
static inline int sys_serial_number(char* serial, int n)
{
	const uint32_t *UniqueID = ((uint32_t*)UID_BASE);
	return snprintf(serial,n,/*"%-.*s"*/"%02X%08X", /* 7, (char*)UniqueID+5, */ UniqueID[1] & 0xFF, UniqueID[0]);
}


/*! \brief запросить идентификатор устройства

	уникальный идентификатор устройства может использоваться как Ethernet MAC или USB serial 
	\return ссылку на бинарную строку -- уникальный идентификатор устройства (серийный номер) 96-бит
*/
#endif
//! \}
