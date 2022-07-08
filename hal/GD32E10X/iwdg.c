/*! \brief Independent watchdog Free running watchdog timer

	Перезагрузка возникает когда функция очистки не вызывается более чем 1 сек.
 */

#include "board.h"
#include "gd32e10x_fwdgt.h"
#include "module.h"

#ifndef BOARD_IWDG_PRESCALER
#define BOARD_IWDG_PRESCALER FWDGT_PSC_DIV16 // \16 ресет происходит через 16*4095/40кГц
#endif
#ifndef BOARD_IWDG_RELOAD
#define BOARD_IWDG_RELOAD 0xFFF
#endif
#ifndef BOARD_IWDG_WINDOW
#define BOARD_IWDG_WINDOW BOARD_IWDG_RELOAD
#endif

static void* /* __attribute__((constructor))*/ _init()
{
	
	
	
	uint32_t timeout, flag_status;
	
	//RCU_RSTSCK_IRC40KEN;
	
	FWDGT_CTL = FWDGT_WRITEACCESS_ENABLE;

	timeout = FWDGT_PSC_TIMEOUT;
    /* wait until the PUD flag to be reset */
    do{
       flag_status = FWDGT_STAT & FWDGT_STAT_PUD; // prescaler divider value update
    }while(((uint32_t)RESET != flag_status) && (--timeout > 0U));

	FWDGT_PSC = BOARD_IWDG_PRESCALER & FWDGT_PSC_PSC;

    timeout = FWDGT_RLD_TIMEOUT;
    /* wait until the RUD flag to be reset */
    do{
       flag_status = FWDGT_STAT & FWDGT_STAT_RUD; // counter reload value update
    }while(((uint32_t)RESET != flag_status) && (--timeout > 0U));

	FWDGT_RLD = BOARD_IWDG_RELOAD & FWDGT_RLD_RLD;
	
//	FWDGT_CTL = FWDGT_WRITEACCESS_DISABLE;
	FWDGT_CTL = FWDGT_KEY_RELOAD;
	FWDGT_CTL = FWDGT_KEY_ENABLE;
	return NULL;
}

static void _scan(void* data)
{
//	debug("IWDog\r\n");
	FWDGT_CTL = FWDGT_KEY_RELOAD;
}

MODULE(IWatchdog, _init, _scan);
