/*!
    \file    cdc_acm_core.c
    \brief   CDC ACM driver

    \version 2020-08-05, V2.0.0, firmware for GD32E10x
    \version 2020-12-10, V2.0.1, firmware for GD32E10x
    \version 2020-12-14, V2.0.2, firmware for GD32E10x
    \version 2020-12-31, V2.1.0, firmware for GD32E10x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/
#define CDC_SUB_ITF_COUNT 3
//#define DEBUG_ITF
#include "cdc_acm_core.h"
#include <cmsis_os.h>
#include <stdio.h>
#if 1
#define USBD_VID                          0x9148U//0x28E9U
#define USBD_PID                          0x0004U//0x018AU
#else
#define USBD_VID                          0x28E9U
#define USBD_PID                          0x018AU
#endif

/* note:it should use the C99 standard when compiling the below codes */
/* USB standard device descriptor */
const usb_desc_dev cdc_dev_desc =
{
    .header = 
     {
         .bLength          = USB_DEV_DESC_LEN, 
         .bDescriptorType  = USB_DESCTYPE_DEV,
     },
    .bcdUSB                = 0x0200U,
    .bDeviceClass          = USB_CLASS_CDC,
    .bDeviceSubClass       = 0x00U,
    .bDeviceProtocol       = 0x00U,
    .bMaxPacketSize0       = USB_FS_EP0_MAX_LEN,
    .idVendor              = USBD_VID,
    .idProduct             = USBD_PID,
    .bcdDevice             = 0x0100U,
    .iManufacturer         = STR_IDX_MFC,
    .iProduct              = STR_IDX_PRODUCT,
    .iSerialNumber         = STR_IDX_SERIAL,
    .bNumberConfigurations = USBD_CFG_MAX_NUM,
};

/* USB device configuration descriptor */
const usb_cdc_desc_config_set cdc_config_desc = 
{
    .config = 
    {
        .header = 
         {
             .bLength         = sizeof(usb_desc_config), 
             .bDescriptorType = USB_DESCTYPE_CONFIG,
         },
        .wTotalLength         = sizeof(usb_cdc_desc_config_set),
        .bNumInterfaces       = (0x01U+CDC_SUB_ITF_COUNT),
        .bConfigurationValue  = 0x01U,
        .iConfiguration       = 0x00U,
        .bmAttributes         = 0x80U,// Bus Powered
        .bMaxPower            = 0x32U// 100mA
    },
#if defined(DEBUG_ITF)
	.debug_itf = 
	{
        .header = 
        {
			.bLength         = sizeof(usb_desc_itf), 
			.bDescriptorType = USB_DESCTYPE_ITF,
		},
        .bInterfaceNumber     = 0x00U,
        .bAlternateSetting    = 0x00U,
        .bNumEndpoints        = 0x02U,
        .bInterfaceClass      = 0xFF, // Custom
        .bInterfaceSubClass   = 0xFF,
        .bInterfaceProtocol   = 0xFF,
        .iInterface           = 0x00U
	},
	.debug_out_endpoint = 
	{
		.header = 
		 {
			 .bLength         = sizeof(usb_desc_ep), 
			 .bDescriptorType = USB_DESCTYPE_EP, 
		 },
		.bEndpointAddress     = CDC_DATA_OUT_EP,
		.bmAttributes         = USB_EP_ATTR_BULK,
		.wMaxPacketSize       = USB_CDC_DATA_PACKET_SIZE,
		.bInterval            = 0x00U
	},
	.debug_in_endpoint = 
	{
		.header = 
		 {
			 .bLength         = sizeof(usb_desc_ep), 
			 .bDescriptorType = USB_DESCTYPE_EP 
		 },
		.bEndpointAddress     = CDC_DATA_IN_EP,
		.bmAttributes         = USB_EP_ATTR_BULK,
		.wMaxPacketSize       = USB_CDC_DATA_PACKET_SIZE,
		.bInterval            = 0x00U
	},
#endif
#if (CDC_SUB_ITF_COUNT>0)
    .cmd_itf = 
    {
        .header = 
         {
             .bLength         = sizeof(usb_desc_itf), 
             .bDescriptorType = USB_DESCTYPE_ITF 
         },
        .bInterfaceNumber     = 0x00U,
        .bAlternateSetting    = 0x00U,
        .bNumEndpoints        = 0x00U,//0x01U,
        .bInterfaceClass      = USB_CLASS_CDC,
        .bInterfaceSubClass   = USB_CDC_SUBCLASS_ACM,
        .bInterfaceProtocol   = USB_CDC_PROTOCOL_AT,// было _AT ... AT Commands: V.250 etc
        .iInterface           = 0x00U
    },

    .cdc_header = 
    {// 5.2.3.1 Header Functional Descriptor
        .header =
         {
            .bLength         = sizeof(usb_desc_header_func), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype  = 0x00U,/* 00h Header Functional Descriptor, which marks the beginning of the
										concatenated set of functional descriptors for the interface.*/
        .bcdCDC              = 0x0110U// можно повысить до 0x0120 - версия спецификации
    },

    .cdc_call_managment = 
    {
        .header = 
         {
            .bLength         = sizeof(usb_desc_call_managment_func), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype  = 0x01U,/* 01h Call Management Functional Descriptor */
        .bmCapabilities      = 0x00U,/* 5.3.1 Call Management Functional Descriptor */
        .bDataInterface      = 0x01U
    },

    .cdc_acm = 
    {
        .header = 
         {
            .bLength         = sizeof(usb_desc_acm_func), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype  = 0x02U, /* 02h Abstract Control Management Functional Descriptor */
        .bmCapabilities      = 0x02U,
/* 5.3.2 Abstract Control Management Functional Descriptor [PSTN120]
		D3: 1 - Device supports the notification Network_Connection.
		D2: 1 - Device supports the request Send_Break
		D1: 1 - Device supports the request combination of Set_Line_Coding, Set_Control_Line_State,
				Get_Line_Coding, and the notification Serial_State.
		D0: 1 - Device supports the request combination of Set_Comm_Feature, Clear_Comm_Feature, and
				Get_Comm_Feature. */
    },

    .cdc_union = 
    {
        .header = 
         {
            .bLength         = sizeof(usb_desc_union_func), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype  = 0x06U, /* 06h Union Functional Descriptor */
        .bControlInterface   = 0x00U,
        .bSubordinateInterface = {0x01U,2,3}, // по числу интерфейсов, перечисление 1..2..3
    },
#if 0
    .cdc_cmd_endpoint = 
    {
        .header = 
         {
            .bLength         = sizeof(usb_desc_ep), 
            .bDescriptorType = USB_DESCTYPE_EP,
         },
        .bEndpointAddress    = CDC_CMD_EP,
        .bmAttributes        = USB_EP_ATTR_INT,
        .wMaxPacketSize      = USB_CDC_CMD_PACKET_SIZE,
        .bInterval           = 0x0AU
    },
#else
#endif
	.cdc_sub_interfaces = 
	{
		[0] = {
			.cdc_data_interface = 
			{
				.header = 
				 {
					.bLength         = sizeof(usb_desc_itf), 
					.bDescriptorType = USB_DESCTYPE_ITF,
				 },
				.bInterfaceNumber    = 0x01U,
				.bAlternateSetting   = 0x00U,
				.bNumEndpoints       = 0x02U,
				.bInterfaceClass     = USB_CLASS_DATA,// Data Interface Class
				.bInterfaceSubClass  = 0x00U,
				.bInterfaceProtocol  = USB_CDC_PROTOCOL_NONE,
				.iInterface          = 0x00U
			},

			.cdc_out_endpoint = 
			{
				.header = 
				 {
					 .bLength         = sizeof(usb_desc_ep), 
					 .bDescriptorType = USB_DESCTYPE_EP, 
				 },
				.bEndpointAddress     = CDC_DATA_OUT_EP,
				.bmAttributes         = USB_EP_ATTR_BULK,
				.wMaxPacketSize       = USB_CDC_DATA_PACKET_SIZE,
				.bInterval            = 0x00U
			},

			.cdc_in_endpoint = 
			{
				.header = 
				 {
					 .bLength         = sizeof(usb_desc_ep), 
					 .bDescriptorType = USB_DESCTYPE_EP 
				 },
				.bEndpointAddress     = CDC_DATA_IN_EP,
				.bmAttributes         = USB_EP_ATTR_BULK,
				.wMaxPacketSize       = USB_CDC_DATA_PACKET_SIZE,
				.bInterval            = 0x00U
			}
		},
#if (CDC_SUB_ITF_COUNT>1)
		[1] = {
			.cdc_data_interface = 
			{
				.header = 
				 {
					.bLength         = sizeof(usb_desc_itf), 
					.bDescriptorType = USB_DESCTYPE_ITF,
				 },
				.bInterfaceNumber    = 0x02U,
				.bAlternateSetting   = 0x00U,
				.bNumEndpoints       = 0x02U,
				.bInterfaceClass     = USB_CLASS_DATA,// Data Interface Class
				.bInterfaceSubClass  = 0x00U,
				.bInterfaceProtocol  = USB_CDC_PROTOCOL_NONE,
				.iInterface          = 0x00U /* Index of string descriptor describing this interface */
			},

			.cdc_out_endpoint = 
			{
				.header = 
				 {
					 .bLength         = sizeof(usb_desc_ep), 
					 .bDescriptorType = USB_DESCTYPE_EP, 
				 },
				.bEndpointAddress     = CDC_DATA1_OUT_EP,
				.bmAttributes         = USB_EP_ATTR_BULK,
				.wMaxPacketSize       = USB_CDC_DATA_PACKET_SIZE,
				.bInterval            = 0x00U
			},

			.cdc_in_endpoint = 
			{
				.header = 
				 {
					 .bLength         = sizeof(usb_desc_ep), 
					 .bDescriptorType = USB_DESCTYPE_EP 
				 },
				.bEndpointAddress     = CDC_DATA1_IN_EP,
				.bmAttributes         = USB_EP_ATTR_BULK,
				.wMaxPacketSize       = USB_CDC_DATA_PACKET_SIZE,
				.bInterval            = 0x00U
			}
		},
#endif
#if (CDC_SUB_ITF_COUNT>2)
		[2] = {
			.cdc_data_interface = 
			{
				.header = 
				 {
					.bLength         = sizeof(usb_desc_itf), 
					.bDescriptorType = USB_DESCTYPE_ITF,
				 },
				.bInterfaceNumber    = 0x03U,
				.bAlternateSetting   = 0x00U,
				.bNumEndpoints       = 0x02U,
				.bInterfaceClass     = USB_CLASS_DATA,// Data Interface Class
				.bInterfaceSubClass  = 0x00U,
				.bInterfaceProtocol  = USB_CDC_PROTOCOL_NONE,
				.iInterface          = 0x00U /* Index of string descriptor describing this interface */
			},

			.cdc_out_endpoint = 
			{
				.header = 
				 {
					 .bLength         = sizeof(usb_desc_ep), 
					 .bDescriptorType = USB_DESCTYPE_EP, 
				 },
				.bEndpointAddress     = CDC_DATA2_OUT_EP,
				.bmAttributes         = USB_EP_ATTR_BULK,
				.wMaxPacketSize       = USB_CDC_DATA_PACKET_SIZE,
				.bInterval            = 0x00U
			},

			.cdc_in_endpoint = 
			{
				.header = 
				 {
					 .bLength         = sizeof(usb_desc_ep), 
					 .bDescriptorType = USB_DESCTYPE_EP 
				 },
				.bEndpointAddress     = CDC_DATA2_IN_EP,
				.bmAttributes         = USB_EP_ATTR_BULK,
				.wMaxPacketSize       = USB_CDC_DATA_PACKET_SIZE,
				.bInterval            = 0x00U
			}
		},
#endif
	}
#endif
};

/* USB language ID Descriptor */
static const usb_desc_LANGID usbd_language_id_desc = 
{
    .header = 
     {
         .bLength         = sizeof(usb_desc_LANGID), 
         .bDescriptorType = USB_DESCTYPE_STR,
     },
    .wLANGID              = ENG_LANGID
};

/* USB manufacture string */
static const usb_desc_str manufacturer_string = 
{
    .header = 
     {
         .bLength         = USB_STRING_LEN(10), 
         .bDescriptorType = USB_DESCTYPE_STR,
     },
    .unicode_string = {'G', 'i', 'g', 'a', 'D', 'e', 'v', 'i', 'c', 'e'}
};

/* USB product string */
static const usb_desc_str product_string = 
{
    .header = 
     {
         .bLength         = USB_STRING_LEN(12), 
         .bDescriptorType = USB_DESCTYPE_STR,
     },
    .unicode_string = {'G', 'D', '3', '2', '-', 'C', 'D', 'C', '_', 'A', 'C', 'M'}
};

/* USB interface string */
static const usb_desc_str interface_string = 
{
    .header = 
     {
         .bLength         = USB_STRING_LEN(13), 
         .bDescriptorType = USB_DESCTYPE_STR,
     },
    .unicode_string = {'C','M','S','I','S', '-', 'D', 'A', 'P', 'L', 'i', 'n', 'k'}
};

/* USBD serial string */
static usb_desc_str serial_string = 
{
    .header = 
     {
         .bLength         = USB_STRING_LEN(12), 
         .bDescriptorType = USB_DESCTYPE_STR,
     }
};

/* USB string descriptor set */
void *const usbd_cdc_strings[] = 
{
    [STR_IDX_LANGID]  = (uint8_t *)&usbd_language_id_desc,
    [STR_IDX_MFC]     = (uint8_t *)&manufacturer_string,
    [STR_IDX_PRODUCT] = (uint8_t *)&product_string,
    [STR_IDX_SERIAL]  = (uint8_t *)&serial_string,
    [STR_IDX_CONFIG]  = (uint8_t *)&product_string,
    [STR_IDX_ITF]  = (uint8_t *)&interface_string
};

usb_desc cdc_desc = 
{
    .dev_desc    = (uint8_t *)&cdc_dev_desc,
    .config_desc = (uint8_t *)&cdc_config_desc,
    .strings     = usbd_cdc_strings
};

/* local function prototypes ('static') */
static uint8_t cdc_acm_init   (usb_dev *udev, uint8_t config_index);
static uint8_t cdc_acm_deinit (usb_dev *udev, uint8_t config_index);
static uint8_t cdc_acm_req    (usb_dev *udev, usb_req *req);
static uint8_t cdc_ctlx_out   (usb_dev *udev);
static uint8_t cdc_acm_in     (usb_dev *udev, uint8_t ep_num);
static uint8_t cdc_acm_out    (usb_dev *udev, uint8_t ep_num);

/* USB CDC device class callbacks structure */
usb_class_core cdc_class =
{
    .command   = NO_CMD,
    .alter_set = 0U,

    .init      = cdc_acm_init,
    .deinit    = cdc_acm_deinit,

    .req_proc  = cdc_acm_req,
    .ctlx_out  = cdc_ctlx_out,
    .data_in   = cdc_acm_in,
    .data_out  = cdc_acm_out
};
#if 0
/*!
    \brief      check CDC ACM is ready for data transfer
    \param[in]  udev: pointer to USB device instance
    \param[out] none
    \retval     0 if CDC is ready, 5 else
*/
uint8_t cdc_acm_check_ready(usb_dev *udev)
{
    if (udev->dev.class_data[CDC_COM_INTERFACE] != NULL) {
        usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

        if ((1U == cdc->packet_receive) && (1U == cdc->packet_sent)) {
            return 0U;
        }
    }

    return 1U;
}

/*!
    \brief      send CDC ACM data
    \param[in]  udev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
void cdc_acm_data_send (usb_dev *udev)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    if (0U != cdc->receive_length) {
        cdc->packet_sent = 0U;

        usbd_ep_send (udev, CDC_DATA_IN_EP, (uint8_t*)(cdc->data), cdc->receive_length);

        cdc->receive_length = 0U;
    }
}

/*!
    \brief      receive CDC ACM data
    \param[in]  udev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
void cdc_acm_data_receive (usb_dev *udev)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    cdc->packet_receive = 0U;
    cdc->packet_sent = 0U;

    usbd_ep_recev(udev, CDC_DATA_OUT_EP, (uint8_t*)(cdc->data), USB_CDC_DATA_PACKET_SIZE);
}
#endif
/*!
    \brief      initialize the CDC ACM device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_acm_init (usb_dev *udev, uint8_t config_index)
{
    static usb_cdc_handler cdc_handler;
#ifdef DEBUG_ITF
	/* initialize the data TX endpoint */
	usbd_ep_setup (udev, &(cdc_config_desc.debug_in_endpoint));
	/* initialize the data RX endpoint */
	usbd_ep_setup (udev, &(cdc_config_desc.debug_out_endpoint));
#endif
#if (CDC_SUB_ITF_COUNT>0)
	int itf;
	for (itf=0; itf<CDC_SUB_ITF_COUNT; itf++){
		/* initialize the data TX endpoint */
		usbd_ep_setup (udev, &(cdc_config_desc.cdc_sub_interfaces[itf].cdc_in_endpoint));
		/* initialize the data RX endpoint */
		usbd_ep_setup (udev, &(cdc_config_desc.cdc_sub_interfaces[itf].cdc_out_endpoint));
	}

    /* initialize the command TX endpoint */
    //usbd_ep_setup (udev, &(cdc_config_desc.cdc_cmd_endpoint));
#endif
    /* initialize CDC handler structure */
    cdc_handler.packet_receive = 1U;
    cdc_handler.packet_sent = 1U;
    cdc_handler.receive_length = 0U;

    cdc_handler.line_coding = (acm_line){
        .dwDTERate   = 115200,
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits   = 0x08
    };

    udev->dev.class_data[CDC_COM_INTERFACE] = (void *)&cdc_handler;

    return USBD_OK;
}

/*!
    \brief      de-initialize the CDC ACM device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_acm_deinit (usb_dev *udev, uint8_t config_index)
{
    /* deinitialize the data TX/RX endpoint */
    usbd_ep_clear (udev, CDC_DATA_IN_EP);
    usbd_ep_clear (udev, CDC_DATA_OUT_EP);

    /* deinitialize the command TX endpoint */
    usbd_ep_clear (udev, CDC_CMD_EP);

    return USBD_OK;
}

/*!
    \brief      handle the CDC ACM class-specific requests
    \param[in]  udev: pointer to USB device instance
    \param[in]  req: device class-specific request
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_acm_req (usb_dev *udev, usb_req *req)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    usb_transc *transc = NULL;

    switch (req->bRequest) {
    case SEND_ENCAPSULATED_COMMAND:// Notification 
        transc = &udev->dev.transc_out[0];
		debug("CDC:send enc cmd\r\n");
        /* set the value of the current command to be processed */
        udev->dev.class_core->alter_set = req->bRequest;

        /* enable EP0 prepare to receive command data packet */
        transc->remain_len = req->wLength;
        transc->xfer_buf = cdc->cmd;
        break;

    case GET_ENCAPSULATED_RESPONSE:
        /* no operation for this driver */
        break;

    case SET_COMM_FEATURE:
        /* no operation for this driver */
        break;

    case GET_COMM_FEATURE:
        /* no operation for this driver */
        break;

    case CLEAR_COMM_FEATURE:
        /* no operation for this driver */
        break;

    case SET_LINE_CODING:
	debug("CDC:Line Coding\r\n");
        transc = &udev->dev.transc_out[0];
        
        /* set the value of the current command to be processed */
        udev->dev.class_core->alter_set = req->bRequest;

        /* enable EP0 prepare to receive command data packet */
        transc->remain_len = req->wLength;
        transc->xfer_buf = cdc->cmd;
        break;

    case GET_LINE_CODING:
	debug("CDC:Get Line Coding\r\n");
        transc = &udev->dev.transc_in[0];
        
        cdc->cmd[0] = (uint8_t)(cdc->line_coding.dwDTERate);
        cdc->cmd[1] = (uint8_t)(cdc->line_coding.dwDTERate >> 8);
        cdc->cmd[2] = (uint8_t)(cdc->line_coding.dwDTERate >> 16);
        cdc->cmd[3] = (uint8_t)(cdc->line_coding.dwDTERate >> 24);
        cdc->cmd[4] = cdc->line_coding.bCharFormat;
        cdc->cmd[5] = cdc->line_coding.bParityType;
        cdc->cmd[6] = cdc->line_coding.bDataBits;

        transc->xfer_buf = cdc->cmd;
        transc->remain_len = 7U;
        break;

    case SET_CONTROL_LINE_STATE:
        /* no operation for this driver */
        break;

    case SEND_BREAK: if (1){
		static uint8_t s[32];
		snprintf(s, 32, "USB:Break if=%d dur=%d ms\r\n", req->wIndex, req->wValue); // Duration of Break,  0xFFFF - включить и не ждать 
		debug(s);
	}
		debug("CDC:Break\r\n");
        /* no operation for this driver */
        break;

    default:
        break;
    }

    return USBD_OK;
}

/*!
    \brief      handle CDC ACM setup data
    \param[in]  udev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_ctlx_out (usb_dev *udev)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    switch (udev->dev.class_core->alter_set) {
        /* process the command data */
	case SEND_ENCAPSULATED_COMMAND:
		//printf("", cmd);
		break;
	case SET_LINE_CODING:
        cdc->line_coding.dwDTERate = (uint32_t)((uint32_t)cdc->cmd[0] | 
                                               ((uint32_t)cdc->cmd[1] << 8U) | 
                                               ((uint32_t)cdc->cmd[2] << 16U) | 
                                               ((uint32_t)cdc->cmd[3] << 24U));
        cdc->line_coding.bCharFormat = cdc->cmd[4];
        cdc->line_coding.bParityType = cdc->cmd[5];
        cdc->line_coding.bDataBits   = cdc->cmd[6];
		break;
	default: break;
    }
    udev->dev.class_core->alter_set = NO_CMD;

    return USBD_OK;
}

/*!
    \brief      handle CDC ACM data
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_acm_in (usb_dev *udev, uint8_t ep_num)
{
    usb_transc *transc = &udev->dev.transc_in[EP_ID(ep_num)];

    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    if ((0U == transc->xfer_len % transc->max_len) && (0U != transc->xfer_len)) {
        usbd_ep_send (udev, ep_num, NULL, 0U);// 
    } else {
        cdc->packet_sent = 1U;
    }

    return USBD_OK;
}
#include <cmsis_os.h>
static osThreadId usbd_owner=NULL;
static int32_t usbd_flag;
void cdc_acm_open(int32_t flag)
{
	usbd_flag = flag;
	usbd_owner = osThreadGetId();
}
/*!
    \brief      handle CDC ACM data
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_acm_out (usb_dev *udev, uint8_t ep_num)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];

    cdc->packet_receive = 1U;
    cdc->receive_length = ((usb_core_driver *)udev)->dev.transc_out[ep_num].xfer_count;
	if (usbd_owner) {
		osSignalSet(usbd_owner, 1UL<<usbd_flag);
		osThreadNotify(usbd_owner);
	}
    return USBD_OK;
}
