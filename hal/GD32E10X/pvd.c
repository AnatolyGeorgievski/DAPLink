#include "board.h"
#include "gd32e10x_pmu.h"
#include <cmsis_os.h>
/*!
    \brief      select low voltage detector threshold
    \param[in]  lvdt_n:
                only one parameter can be selected which is shown as below:
      \arg        PMU_LVDT_0: voltage threshold is 2.1V
      \arg        PMU_LVDT_1: voltage threshold is 2.3V
      \arg        PMU_LVDT_2: voltage threshold is 2.4V
      \arg        PMU_LVDT_3: voltage threshold is 2.6V
      \arg        PMU_LVDT_4: voltage threshold is 2.7V
      \arg        PMU_LVDT_5: voltage threshold is 2.9V
      \arg        PMU_LVDT_6: voltage threshold is 3.0V
      \arg        PMU_LVDT_7: voltage threshold is 3.1V
    \param[out] none
    \retval     none
*/
void pmu_lvd_select(uint32_t lvdt_n)
{
    /* disable LVD */
    PMU_CTL &= ~PMU_CTL_LVDEN;
    /* clear LVDT bits */
    PMU_CTL &= ~PMU_CTL_LVDT;
    /* set LVDT bits according to lvdt_n */
    PMU_CTL |= lvdt_n;
    /* enable LVD */
    PMU_CTL |= PMU_CTL_LVDEN;
}
/*!	\brief      PMU work at deepsleep mode
    \param [in]  ldo:
                only one parameter can be selected which is shown as below:
      \arg        PMU_LDO_NORMAL: LDO work at normal power mode when pmu enter deepsleep mode
      \arg        PMU_LDO_LOWPOWER: LDO work at low power mode when pmu enter deepsleep mode
    \param [in]  deepsleepmodecmd:
                only one parameter can be selected which is shown as below: 
      \arg        WFI_CMD: use WFI command
      \arg        WFE_CMD: use WFE command
*/
void pmu_to_deepsleepmode(uint32_t ldo, uint8_t deepsleepmodecmd)
{
    static uint32_t reg_snap[ 4 ];      
    /* clear stbmod and ldolp bits */
    PMU_CTL &= ~((uint32_t)(PMU_CTL_STBMOD | PMU_CTL_LDOLP));
    
    /* set ldolp bit according to pmu_ldo */
    PMU_CTL |= ldo;
    
    /* set sleepdeep bit of Cortex-M4 system control register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    reg_snap[ 0 ] = SysTick->CTRL;//REG32( 0xE000E010U ); // SysTick register map STK_CTRL 
    reg_snap[ 1 ] = NVIC->ISER[0];//REG32( 0xE000E100U );// NVIC->ISER[16] из них используется 96 бит
    reg_snap[ 2 ] = NVIC->ISER[1];//REG32( 0xE000E104U );
    reg_snap[ 3 ] = NVIC->ISER[2];//REG32( 0xE000E108U );
    
    SysTick->CTRL = (reg_snap[ 0 ]&0x00010004U); // Count Flag SysTick_CTRL_COUNTFLAG_Msk | SysTick_CTRL_CLKSOURCE_Msk, TickINT disable 
    NVIC->ICER[0]/* REG32( 0xE000E180U ) */ = 0XFF7FF83DU; // clear interrupt enable
    NVIC->ICER[1]/* REG32( 0xE000E184U ) */ = 0XFFFFF8FFU;
    NVIC->ICER[2]/* REG32( 0xE000E188U ) */ = 0xFFFFFFFFU;  
    
    /* select WFI or WFE command to enter deepsleep mode */
    if(WFI_CMD == deepsleepmodecmd){
        __WFI();
    }else{
        __SEV();
        __WFE();
        __WFE();
    }

    SysTick->CTRL/* REG32( 0xE000E010U )*/ = reg_snap[ 0 ] ; 
    NVIC->ISER[0]/* REG32( 0xE000E100U )*/ = reg_snap[ 1 ] ;
    NVIC->ISER[1]/* REG32( 0xE000E104U )*/ = reg_snap[ 2 ] ;
    NVIC->ISER[2]/* REG32( 0xE000E108U )*/ = reg_snap[ 3 ] ;  
    
    /* reset sleepdeep bit of Cortex-M4 system control register */
    SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);	
}
static void __attribute__((constructor)) PVD_init()
{
	// включить детектирование низкого питания по уровню 2.9В
	pmu_lvd_select(PMU_LVDT_5);
	NVIC_EnableIRQ(LVD_IRQn);
}

#define WEAK __attribute__ ((weak))
//extern void WEAK HAL_PWR_PVDCallback();
/*
6.2.2 Programmable voltage detector (PVD)

	\brief обработка прерывания по низкому питанию, 
	уровень срабатывания выставляется в регистре PWR_CR от 2.2 до 2.9В
*/
void LVD_IRQHandler() PRIVILEGED_FUNCTION;
void LVD_IRQHandler()
{
	/* Check PWR exti flag */
	if((EXTI_PD & BIT(16)) != 0)
	{
		/* Clear PWR Exti pending bit */
		EXTI_PD = BIT(16);
		//HAL_PWR_PVDCallback();
		//BOARD_SAFE_PINS;
		/* PWR PVD interrupt user callback */
		char* str = "Low Voltage Detect\r\n";
		debug(str);
/*		while(1) {
			if (ITM->PORT[0].u32!=0){
				if (str[0]=='\0') break;
				ITM->PORT[0].u8 = *str++;
			}
		} */
	}
}