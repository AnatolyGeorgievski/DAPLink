#include "board.h"
#include "mpu.h"
#include <stdio.h>
//#include "device_tree.h"
/*!

*/
/*! 
[PM0214] 4.4.13 Memory management fault address register (MMFSR)
	\sa The Definitive Guide to the ARM Cortex-M3
 */
struct _mpu {
	uint32_t address;
	uint32_t attr;
};
#if (__MPU_PRESENT==1)
	
/*! Execute Never (XN) Means the processor prevents instruction accesses. Any
attempt to fetch an instruction from an XN region causes a
memory management fault exception. */
#define MPU_XN (1<<28) // исполнение кода запрещено
// Права доступа Привилегии и Пользователь
#define MPU_AP_PNO_UNO (0<<24) // All accesses generate a permission fault

#define MPU_AP_PRW_UNO (1<<24) // Access from privileged software only
#define MPU_AP_PRW_URO (2<<24) // Writes by unprivileged software generate a permission fault
#define MPU_AP_PRW_URW (3<<24) // Full access

#define MPU_AP_PRO_UNO (5<<24) // Reads by privileged software only
#define MPU_AP_PRO_URO (6<<24) // Read only, by privileged or unprivileged software
#define MPU_AP_PNO_URO (7<<24) // Reads by privileged software only
/*				TEX  C B S
Flash memory 	b000 1 0 0 Normal memory, Non-shareable, write-through
Internal SRAM 	b000 1 0 1 Normal memory, Shareable, write-through
External SRAM 	b000 1 1 1 Normal memory, Shareable, write-back, write-allocate
Peripherals 	b000 0 1 1 Device memory, Shareable
	RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 32K
	CCM (xrw) : ORIGIN = 0x10000000, LENGTH = 0K
	FLASH (rx): ORIGIN = 0x08000000, LENGTH = 256K
 */
// Биты TEX S C B 
#define MPU_STRONG (0<<16)
#define MPU_PERIPH (5<<16)
#define MPU_ESRAM (7<<16)
#define MPU_ISRAM (6<<16)
#define MPU_FLASH (2<<16)
#define MPU_SIZE(nbits) ((nbits)<<1)
/*! субрегионы, маска разбивает на 8 частей. 1-субрегион заблокирован, 0-разрешен
Использование субрегионов позволяет определять блоки памяти не крантные 2^N
*/
#define MPU_SRD(mask) ((mask)<<8) 
#define MPU_EN 1 // разрешить регион
// Кодирование длины региона число нулей CTZ размер блока =2^(n+1)
#define MPU_REGION_SIZE_32B     0x04
#define MPU_REGION_SIZE_64B     0x05
#define MPU_REGION_SIZE_128B    0x06
#define MPU_REGION_SIZE_256B    0x07
#define MPU_REGION_SIZE_512B    0x08
#define MPU_REGION_SIZE_1KB     0x09
#define MPU_REGION_SIZE_2KB     0x0A
#define MPU_REGION_SIZE_4KB     0x0B
#define MPU_REGION_SIZE_8KB     0x0C
#define MPU_REGION_SIZE_16KB    0x0D
#define MPU_REGION_SIZE_32KB    0x0E
#define MPU_REGION_SIZE_64KB    0x0F
#define MPU_REGION_SIZE_128KB   0x10
#define MPU_REGION_SIZE_256KB   0x11
#define MPU_REGION_SIZE_512KB   0x12
#define MPU_REGION_SIZE_1MB     0x13
#define MPU_REGION_SIZE_2MB     0x14
#define MPU_REGION_SIZE_4MB     0x15
#define MPU_REGION_SIZE_8MB     0x16
#define MPU_REGION_SIZE_16MB    0x17
#define MPU_REGION_SIZE_32MB    0x18
#define MPU_REGION_SIZE_64MB    0x19
#define MPU_REGION_SIZE_128MB   0x1A
#define MPU_REGION_SIZE_256MB   0x1B
#define MPU_REGION_SIZE_512MB   0x1C
#define MPU_REGION_SIZE_1GB     0x1D
#define MPU_REGION_SIZE_2GB     0x1E
#define MPU_REGION_SIZE_4GB     0x1F

// [] Figure 6. STM32F373xx memory map
static const struct _mpu mpu[] = {
	{FLASH_BASE, MPU_AP_PRO_URO | MPU_FLASH  | MPU_SIZE(MPU_REGION_SIZE_256KB) | 1}, // Flash memory
#ifdef CCMDATARAM_BASE
	{CCMDATARAM_BASE, MPU_XN|MPU_AP_PRW_URW | MPU_ISRAM  | MPU_SIZE(MPU_REGION_SIZE_64KB) | 1}, // CCM memory
	{CCMDATARAM_BB_BASE, MPU_XN|MPU_AP_PRW_URO | MPU_ISRAM  | MPU_SIZE(MPU_REGION_SIZE_2MB) | 1}, // Flash memory
#endif
	{SRAM_BASE, MPU_XN|MPU_AP_PRW_URW | MPU_ISRAM  | MPU_SRD(0xF0) | MPU_SIZE(MPU_REGION_SIZE_256KB) | 1}, // Flash memory
	{SRAM_BB_BASE, MPU_XN|MPU_AP_PRW_URO | MPU_ISRAM  | MPU_SIZE(MPU_REGION_SIZE_32MB) | 1}, // Flash memory
	{PERIPH_BASE, MPU_XN|MPU_AP_PRW_URO | MPU_PERIPH | MPU_SIZE(MPU_REGION_SIZE_512MB) | 1}, // Flash memory
	{PERIPH_BB_BASE, MPU_XN|MPU_AP_PRW_URO | MPU_PERIPH | MPU_SIZE(MPU_REGION_SIZE_32MB) | 1}, // Flash memory
	{0xE0000000, MPU_XN|MPU_AP_PRW_UNO | MPU_STRONG | MPU_SIZE(MPU_REGION_SIZE_512MB) | 1}, // Flash memory
	{0x60000000, MPU_AP_PRW_URW | MPU_ESRAM  | MPU_SIZE(MPU_REGION_SIZE_1GB) | 1}, // Flash memory
	{0xA0000000, MPU_XN|MPU_AP_PRO_URO | MPU_PERIPH | MPU_SIZE(MPU_REGION_SIZE_1GB) | 1}, // Flash memory
};
static void __attribute__((constructor)) mpu_init(void)
{
	int size = sizeof(mpu)/sizeof(struct _mpu);
	
//	uint32_t* values = dt_id_array(dt, "regions", size);
	MPU->CTRL &= ~(MPU_CTRL_PRIVDEFENA_Msk| MPU_CTRL_ENABLE_Msk);
	if ((MPU->TYPE>>MPU_TYPE_DREGION_Pos) < size) {
		size = (MPU->TYPE>>MPU_TYPE_DREGION_Pos);
		MPU->CTRL |= MPU_CTRL_PRIVDEFENA_Msk;
	}
	if (size==0) return;
	int i;
	for (i=0; i<size; i++){
		mpu_region_set(i, mpu[i].address, mpu[i].attr);	
		printf("MPU region %08X %08X\r\n", MPU->RBAR, MPU->RASR);
	}
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;  // Enable the memory fault exception
	MPU->CTRL  |= /*MPU_CTRL_PRIVDEFENA_Msk |*/ MPU_CTRL_ENABLE_Msk; 
	//printf("MPU enabled %d\r\n", MPU->TYPE>>MPU_TYPE_DREGION_Pos);
}
#endif
