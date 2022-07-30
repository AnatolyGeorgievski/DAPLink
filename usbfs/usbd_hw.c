/* usb */
#include "board.h"
#include <stdio.h>
#include <cmsis_os.h>
#include "drv_usb_hw.h"
#include "drv_usbd_int.h"
#include "usbd_core.h"
#include "cdc_acm_core.h"

usb_core_driver cdc_acm;
//static osThreadId owner[USBD_ITF_MAX_NUM];
//static int32_t flag[USBD_ITF_MAX_NUM];
void usb_open(int32_t itf, int32_t flag_idx)
{
extern void cdc_acm_open(uint8_t itf, int32_t flag);
	cdc_acm_open(itf, flag_idx);

	//flag[itf] = flag_idx;
	//owner[itf] = osThreadGetId();
if (itf==0) {// do_once
#ifdef USE_IRC48M
	RCU_ADDCTL |= RCU_ADDCTL_IRC48MEN;/* enable IRC48M clock */
	while ((RCU_ADDCTL & RCU_ADDCTL_IRC48MSTB)==0);/* wait till IRC48M is ready */
	RCU_ADDCTL |= RCU_ADDCTL_CK48MSEL;
#else
# if BOARD_MCK==120
	RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_USBFSPSC) | (RCU_CKUSB_CKPLL_DIV2_5);
# elif BOARD_MCK==96
	RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_USBFSPSC) | (RCU_CKUSB_CKPLL_DIV2);
# elif BOARD_MCK==72
	RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_USBFSPSC) | (RCU_CKUSB_CKPLL_DIV1_5);
# elif BOARD_MCK==48
	RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_USBFSPSC) | (RCU_CKUSB_CKPLL_DIV1);
# else
	#error "usb prescaler"
# endif
	//timer_prescaler = (BOARD_MCK/12)-1;
#endif
	RCU_AHBEN |= RCU_AHBEN_USBFSEN;

	if( usb_timer_init()) {
//		puts("ERR:DWT->CYCCOUNT\r\n");
		return;
	}
	usbd_init (&cdc_acm, USB_CORE_ENUM_FS, &cdc_desc, &cdc_class);
	NVIC_SetPriority(USBFS_IRQn, (1<<(__NVIC_PRIO_BITS-1)));
	NVIC_EnableIRQ(USBFS_IRQn);
}
	//puts("USB:done\r\n");
}
int usb_is_configured()
{
	return  (USBD_CONFIGURED == cdc_acm.dev.cur_status);
}
/*! \brief управление по выбранному интерфейсу */
int usb_ctrl(int itf, int options, void* req){
	usb_core_driver *udev = &cdc_acm;
	usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];
	int res = 0;
	switch (options & 0xF){
	case 0: {// _GET_REQUEST
		if (req){
			*(usb_req*)req = cdc->req;
			res = cdc->req.wLength;
		}
	}	break;
	case 1: {// _RECV_BUFFER
		cdc->cmd = req;
		res = cdc->data_length;// atomic_exchange??
		cdc->data_length = 0;
	}	break;
	case 2: {// _SEND_BUFFER
		cdc->resp = req;
		cdc->resp_length = options>>8;
	}	break;
	case 3: {// _FEATURES
		if (0<itf && itf <= CDC_SUB_ITF_COUNT)
			cdc->features[itf-1] = req;
	}	break;
	default: 
		res = -1;
		break;
	}
	return res;
}
/*! \brief отослать по выбранному интерфейсу */
int usb_recv(int itf, uint8_t* data, size_t len){
	// \todo мапить интерфейс и едндпоинты
	uint8_t ep_num  = itf;//CDC_DATA_OUT_EP;
//	len = ((usb_core_driver *)&cdc_acm)->dev.transc_out[ep_num].xfer_count; -- вынес в функцию usbd_ep_recev
	return usbd_ep_recev(&cdc_acm, ep_num, data, USB_CDC_DATA_PACKET_SIZE);
}
/*
int usb_send_recv(uint8_t* data, size_t *len){
	// установить буфер
	usbd_ep_recev(&cdc_acm, CDC_DATA_OUT_EP, data, USB_CDC_DATA_PACKET_SIZE);
	// установить максимальный размер данных на приеме
	return 0;
}*/
int usb_send(int itf, uint8_t* data, size_t len) {
	// \todo мапить интерфейс и едндпоинты
	uint8_t ep_num  = itf | 0x80;//CDC_DATA_IN_EP;
	return usbd_ep_send (&cdc_acm, ep_num, data, len);
/*
	if (0U == cdc_acm_check_ready(&cdc_acm)) {
		cdc_acm_data_receive(&cdc_acm);
	} else {
		//osDelay(5);
		
		cdc_acm_data_send(&cdc_acm);
	}
	return len;*/
}
void  USBFS_IRQHandler (void)
{
    usbd_isr (&cdc_acm);
}
