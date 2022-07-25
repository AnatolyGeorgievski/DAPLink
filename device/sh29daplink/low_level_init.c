#include <cmsis_os.h>
#include "board.h"
#include "pio.h"
//#include "gd32e10x_rcu.h"

#define __SYSTEM_CLOCK_120M_PLL_HXTAL           (uint32_t)(120000000)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_120M_PLL_HXTAL;
//#define __SYSTEM_CLOCK_72M_PLL_IRC8M            (uint32_t)(72000000)
//uint32_t SystemCoreClock = __SYSTEM_CLOCK_72M_PLL_IRC8M;
static const Pin pins[] = {BOARD_ALL_PINS};
static void ITM_init(void) 
{
	const int port = 0;//BOARD_DEFAULT_USART;
	CoreDebug->DHCSR |= 1;
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	#if defined(GD32E10X)
	DBG_CTL &= ~DBG_CTL_TRACE_MODE;
	DBG_CTL |=  DBG_CTL_TRACE_IOEN;
	#else// для STM32
	DBGMCU->CR &= ~DBGMCU_CR_TRACE_MODE;
	DBGMCU->CR |=  DBGMCU_CR_TRACE_IOEN; // включить  SWO, трасе моде 00
	#endif
	//*((volatile unsigned *)0xE0000FB0)= 0xC5ACCE55;// lock access
	ITM->LAR = 0xC5ACCE55;// lock access
	ITM->TCR = 0x00010009;// trace control (ITM_TCR_SWOENA_Pos|ITM_TCR_ITMENA_Msk)

	ITM->TER = (0x1UL<<port);// trace enable
	ITM->TPR = (0x1UL<<port);// trace privileges
	//ITM->PORT[port].u8 = '+';
}

#define VECT_TAB_OFFSET  0x00                      /* This value must be a multiple of 0x200. */
#define RCU_MODIFY(__delay)     do{                                     \
                                    volatile uint32_t i;                \
                                    if(0 != __delay){                   \
                                        RCU_CFG0 |= RCU_AHB_CKSYS_DIV2; \
                                        for(i=0; i<__delay; i++){       \
                                        }                               \
                                        RCU_CFG0 |= RCU_AHB_CKSYS_DIV4; \
                                        for(i=0; i<__delay; i++){       \
                                        }                               \
                                    }                                   \
                                }while(0)

#if defined (__SYSTEM_CLOCK_120M_PLL_HXTAL)
/*!
    \brief      configure the system clock to 120M by PLL which selects HXTAL(8M) as its clock source
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_120m_hxtal(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable HXTAL */
    RCU_CTL |= RCU_CTL_HXTALEN;

    /* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
    do{
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
    }while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_HXTALSTB)){
        while(1){
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | FMC_WAIT_STATE_3;
    
    /* HXTAL is stable */
    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* CK_PLL = (CK_PREDIV0) * 30 = 120 MHz */ 
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= (RCU_PLLSRC_HXTAL_IRC48M | RCU_PLL_MUL30);

    RCU_CFG1 &= ~(RCU_CFG1_PLLPRESEL | RCU_CFG1_PREDV0SEL | RCU_CFG1_PLL1MF | RCU_CFG1_PREDV1 | RCU_CFG1_PREDV0);
#ifdef HXTAL_VALUE_8M
    /* CK_PREDIV0 = (CK_HXTAL)/2 *10 /10 = 4 MHz */ 
    RCU_CFG1 |= (RCU_PLLPRESRC_HXTAL | RCU_PREDV0SRC_CKPLL1 | RCU_PLL1_MUL10 | RCU_PREDV1_DIV2 | RCU_PREDV0_DIV10);
#elif defined (HXTAL_VALUE_25M)
    /* CK_PREDIV0 = (CK_HXTAL)/5 *8/10 = 4 MHz */ 
    RCU_CFG1 |= (RCU_PLLPRESRC_HXTAL | RCU_PREDV0SRC_CKPLL1 | RCU_PLL1_MUL8 | RCU_PREDV1_DIV5 | RCU_PREDV0_DIV10);    
#endif
    
    /* enable PLL1 */
    RCU_CTL |= RCU_CTL_PLL1EN;
    /* wait till PLL1 is ready */
    while((RCU_CTL & RCU_CTL_PLL1STB) == 0U){
    }

    /* enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)){
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
    while(0U == (RCU_CFG0 & RCU_SCSS_PLL)){
    }
}
#elif defined (__SYSTEM_CLOCK_72M_PLL_IRC8M)
/*!
    \brief      configure the system clock to 72M by PLL which selects IRC8M as its clock source
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_72m_irc8m(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;
    
    /* enable IRC8M */
    RCU_CTL |= RCU_CTL_IRC8MEN;

    /* wait until IRC8M is stable or the startup time is longer than IRC8M_STARTUP_TIMEOUT */
    do{
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_IRC8MSTB);
    }while((0U == stab_flag) && (IRC8M_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_IRC8MSTB)){
        while(1){
        }
    }
    
    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | FMC_WAIT_STATE_2;
    
    /* IRC8M is stable */
    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* CK_PLL = (CK_IRC8M/2) * 18 = 72 MHz */
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= RCU_PLL_MUL18;

    /* enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)){
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
    while(0U == (RCU_CFG0 & RCU_SCSS_PLL)){
    }
}
#endif
/*!
    \brief      setup the microcontroller system, initialize the system
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SystemInit (void)
{
  /* FPU settings */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
#endif
    /* reset the RCU clock configuration to the default reset state */
    /* Set IRC8MEN bit */
    RCU_CTL |= RCU_CTL_IRC8MEN;

    RCU_MODIFY(0x50);

    RCU_CFG0 &= ~RCU_CFG0_SCS;

    /* Reset HXTALEN, CKMEN, PLLEN, PLL1EN and PLL2EN bits */
    RCU_CTL &= ~(RCU_CTL_PLLEN |RCU_CTL_PLL1EN | RCU_CTL_PLL2EN | RCU_CTL_CKMEN | RCU_CTL_HXTALEN);
    /* disable all interrupts */
    RCU_INT = 0x00ff0000U;

    /* Reset CFG0 and CFG1 registers */
    RCU_CFG0 = 0x00000000U;
    RCU_CFG1 = 0x00000000U;

    /* reset HXTALBPS bit */
    RCU_CTL &= ~(RCU_CTL_HXTALBPS);


    /* configure the system clock source, PLL Multiplier, AHB/APBx prescalers and Flash settings */
    system_clock_120m_hxtal();
	//system_clock_72m_irc8m();
	
	/*  When the port output speed is more than 50 MHz, the user should enable the
		I/O compensation cell. Refer to CPS_EN bit in AFIO_CPSCTL register. */
    
#ifdef VECT_TAB_SRAM
    SCB->VTOR = NVIC_VECTTAB_RAM | (VECT_TAB_OFFSET & NVIC_VECTTAB_OFFSET_MASK);
    __DSB();
#else
    SCB->VTOR = NVIC_VECTTAB_FLASH | (VECT_TAB_OFFSET & NVIC_VECTTAB_OFFSET_MASK);
    __DSB();
#endif

  /* Configure the System clock source, PLL Multiplier and Divider factors, 
     AHB/APBx prescalers and Flash settings ----------------------------------*/
	SysTick_Config(BOARD_STK_RELOAD);
#if 1
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_0, 0, 0));// установить самый высокий приоритет в группе
#else
	NVIC_SetPriority(SysTick_IRQn, (1<<(__NVIC_PRIO_BITS-1)));// установить высокий приоритет
#endif
// Настройка кэш памяти
// Настройка памяти
// Инициализация контроллера прерываний
// Настройка собачего таймера
// Инициализация портов ввода/вывода (AHB2)

	RCU_AHBEN   |=  BOARD_AHB_ENABLE;
	RCU_AHBRST  |=  BOARD_AHB_RESET;
	RCU_AHBRST  &= ~BOARD_AHB_RESET;

	RCU_APB1EN   =  BOARD_APB1_ENABLE;
	// ресет для всей периферии
	RCU_APB1RST |=  BOARD_APB1_RESET;
	RCU_APB1RST &= ~BOARD_APB1_RESET;

	RCU_APB2EN   =  BOARD_APB2_ENABLE;
	// ресет для всей периферии
	RCU_APB2RST |=  BOARD_APB2_RESET;
	RCU_APB2RST &= ~BOARD_APB2_RESET;

	AFIO_CPSCTL |= AFIO_CPSCTL_CPS_EN;
    while(0U == (AFIO_CPSCTL & AFIO_CPSCTL_CPS_RDY)){
    }

//	SetSysClock();
// это требует инициализации 
	PIO_Configure(pins, PIO_LISTSIZE(pins));
/*	ITM_init();

	while (1){
		volatile int count = 0x7FFFFF*(8)/120;
		while (count--)__NOP();
		ITM->PORT[0].u8 = '#';
	} */
  //	AFIO->MAPR = BOARD_REMAP;
// Выполнить процедуру переназначения переферии на порты ввода/вывода

}