BOARD = SH29REL24_04
CHIP  = GD32E10X
CHIPX = GD32E103xB
ARCH  = CORTEXM4F

# SRC += test_main.c 
#	r3core/r3cmd_secure.c -- подписывает команды протокола CMAC
#	r3core/r3cmd_apdu.c 
#	hal/$(CHIP)/usart_rs485.c 
#	hal/$(CHIP)/dma.c 
#	hal/$(CHIP)/iwdg.c 
# adc_scan.c
# sfc_up_03.c 
# main_04.c 
USE_USBFS=1
USE_CMSIS_DAP=1
ifdef USE_CMSIS_DAP
SRC+= \
	dap/DAP.c \
	dap/SW_DP.c \
	dap/JTAG_DP.c \
	dap/UART.c \
	dap/SWO.c
endif
ifdef USE_USBFS
SRC+= \
	usbfs/drv_usb_core.c \
	usbfs/drv_usb_dev.c \
	usbfs/drv_usbd_int.c \
	usbfs/usbd_core.c \
	usbfs/usbd_enum.c \
	usbfs/usbd_transc.c \
	usbfs/usbd_hw.c \
	usbfs/cdc_acm_core.c
endif

#	bitmain_pic.c 

SRC+= \
	hal/$(CHIP)/i2c_master.c \
	hal/$(CHIP)/usart_rs485.c \
	r3core/tracer_itm.c \
	r3core/config.c \
	r3core/r3cmd.c \
	r3core/r3cmd_trace.c \
	r3core/r3cmd_flash.c \
	r3core/crc16.c \
	bitmain_pic_host.c \
	crc5.c \
	i2c_env.c \
	device/$(DEVICE)/main.c 
