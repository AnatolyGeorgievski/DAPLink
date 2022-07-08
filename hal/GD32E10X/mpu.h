// \see [PM0214] 
#ifndef MPU_H
#define MPU_H
#include "board.h"
#include <stdint.h>
#if (__MPU_PRESENT==1)

/*!	\brief 
	\param region_number region number
	\param base_address address
	\param attr size, attributes in one
 */
static inline 
void mpu_region_set(int region_number, uint32_t base_address, uint32_t attrs)
{
	MPU->RNR  = region_number;// Base Address
	MPU->RBAR = base_address & ~(0x1F);// Base Address
	MPU->RASR = attrs;// Base Address
}
static inline 
void mpu_enable()
{
	MPU->CTRL |= MPU_CTRL_ENABLE_Msk;
}
static inline 
void mpu_disable()
{
	MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;
}
#endif
#endif// MPU_H
