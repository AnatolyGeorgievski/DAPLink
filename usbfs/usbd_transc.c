/*!
    \file    usbd_transc.c
    \brief   USB transaction core functions

    \version 2020-08-05, V2.0.0, firmware for GD32E10x
    \version 2020-12-10, V2.0.1, firmware for GD32E10x
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
#include <stdio.h>
#include "usbd_enum.h"
#include "usbd_transc.h"

/* USB send control transaction status */
static usbd_status usbd_ctl_status_send (usb_core_driver *udev);
/* USB control receive status */
static usbd_status usbd_ctl_status_recev (usb_core_driver *udev);


/*!
    \brief      USB send control transaction status
    \param[in]  udev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation cur_status
*/
static usbd_status  usbd_ctl_status_send (usb_core_driver *udev)
{
    udev->dev.control.ctl_state = (uint8_t)USB_CTL_STATUS_IN;

    (void)usbd_ep_send (udev, 0U, NULL, 0U);

    usb_ctlep_startout(udev);

    return USBD_OK;
}

/*!
    \brief      USB control receive status
    \param[in]  udev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation cur_status
*/
static usbd_status usbd_ctl_status_recev (usb_core_driver *udev)
{
    udev->dev.control.ctl_state = (uint8_t)USB_CTL_STATUS_OUT;

    (void)usbd_ep_recev (udev, 0U, NULL, 0U);

    usb_ctlep_startout(udev);

    return USBD_OK;
}
//kprintf(const char* fmt) 

static  char s[32];

/*!
    \brief      USB setup stage processing
    \param[in]  udev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation cur_status
*/
uint8_t usbd_setup_transc (usb_core_driver *udev)
{
    usb_reqsta reqstat = REQ_NOTSUPP;

    usb_req req = udev->dev.control.req;

    switch (req.bmRequestType & USB_REQTYPE_MASK) {
    /* standard device request */
    case USB_REQTYPE_STRD:
        reqstat = usbd_standard_request (udev, &req);
        break;

    /* device class request */
    case USB_REQTYPE_CLASS:
        reqstat = usbd_class_request (udev, &req);
		if(0)debug("Cr.");
        break;

    /* vendor defined request */
    case USB_REQTYPE_VENDOR:
        reqstat = usbd_vendor_request (udev, &req);
		if(0)debug("Vr.");
        break;

    default:
        break;
    }

    if (REQ_SUPP == reqstat) {
        if (0U == req.wLength) {// status send
			usb_transc *transc = &udev->dev.transc_in[0];
			udev->dev.control.ctl_state = (uint8_t)USB_CTL_STATUS_IN;
			transc->xfer_buf = NULL;
			transc->xfer_len = 0;
			transc->xfer_count = 0U;
			/* start the transfer */
			(void)usb_transc_inxfer (udev, transc);
			usb_ctlep_startout(udev);
        } else {
            if (req.bmRequestType & 0x80U) {
				usb_transc *transc = &udev->dev.transc_in[0];
				transc->xfer_len = transc->remain_len;
				transc->xfer_count = 0U;
				/* start the transfer */
				(void)usb_transc_inxfer (udev, transc);
				if (transc->remain_len > transc->max_len) {
					udev->dev.control.ctl_state = (uint8_t)USB_CTL_DATA_IN;
				} else
				{
					udev->dev.control.ctl_state = (uint8_t)USB_CTL_LAST_DATA_IN;
				}
            } else {
				usb_transc *transc = &udev->dev.transc_out[0];
				transc->xfer_len = transc->remain_len;
				transc->xfer_count = 0U;
				/* start the transfer */
				usb_transc_outxfer (udev, transc);
				if (transc->remain_len > transc->max_len) {
					udev->dev.control.ctl_state = (uint8_t)USB_CTL_DATA_OUT;
				} else {
					udev->dev.control.ctl_state = (uint8_t)USB_CTL_LAST_DATA_OUT;
				}
            }
        }
    } else if (REQ_NOTSUPP == reqstat) {
        usbd_enum_error (udev, &req);
		if(0){
			snprintf(s, 32, "NSUP%02X %02X ", (req.bmRequestType & USB_REQTYPE_MASK), req.bRequest);
			debug(s);
		}

    } else {
		// Сюда наши попали
	}

    return (uint8_t)USBD_OK;
}

/*!
    \brief      data out stage processing
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier(0..7)
    \param[out] none
    \retval     USB device operation cur_status
*/
uint8_t usbd_out_transc (usb_core_driver *udev, uint8_t ep_num)
{
    if (0U == ep_num) {
        usb_transc *transc = &udev->dev.transc_out[0];

        switch (udev->dev.control.ctl_state) {
        case USB_CTL_DATA_OUT:
            transc->remain_len -= transc->max_len;
			transc->xfer_len = transc->remain_len;
			transc->xfer_count = 0U;// тут может иначе надо??
			usb_transc_outxfer (udev, transc);
			if (transc->remain_len <= transc->max_len)
				udev->dev.control.ctl_state = (uint8_t)USB_CTL_LAST_DATA_OUT;
            break;
        case USB_CTL_LAST_DATA_OUT:
            if (udev->dev.cur_status == (uint8_t)USBD_CONFIGURED) {
                if (udev->dev.class_core->ctlx_out != NULL) {
                    (void)udev->dev.class_core->ctlx_out (udev);
					//debug("SS.");
                }
            }

            transc->remain_len = 0U;
			udev->dev.control.ctl_state = (uint8_t)USB_CTL_STATUS_IN;
			(void)usbd_ep_send (udev, 0U, NULL, 0U);
			usb_ctlep_startout(udev);
            break;

        default:
            break;
        }
    } else if ((udev->dev.class_core->data_out != NULL) && (udev->dev.cur_status == (uint8_t)USBD_CONFIGURED)) {
        (void)udev->dev.class_core->data_out (udev, ep_num);
    } else {
        /* no operation */
    }

    return (uint8_t)USBD_OK;
}
/*!
    \brief      data in stage processing
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier(0..7)
    \param[out] none
    \retval     USB device operation cur_status
*/
uint8_t usbd_in_transc (usb_core_driver *udev, uint8_t ep_num)
{
    if (0U == ep_num) {
        usb_transc *transc = &udev->dev.transc_in[0];

        switch (udev->dev.control.ctl_state) {
        case USB_CTL_DATA_IN:
            /* update transfer length */
            transc->remain_len -= transc->max_len;
#if 0
            if ((uint8_t)USB_USE_DMA == udev->bp.transfer_mode) {
                transc->xfer_buf += transc->max_len;
            }
#endif
			/* setup the transfer */
			transc->xfer_len = transc->remain_len;
			transc->xfer_count = 0U;
#if 0
			if ((uint8_t)USB_USE_DMA == udev->bp.transfer_mode) {
				transc->dma_addr = (uint32_t)transc->xfer_buf;
			}
#endif
			/* start the transfer */
			(void)usb_transc_inxfer (udev, transc);
//			(void)usbd_ep_send(udev, 0U, transc->xfer_buf, transc->remain_len);// это плохо, внутри происходит копирование transc->xfer_buf
			if (transc->remain_len <= transc->max_len)
				udev->dev.control.ctl_state = (uint8_t)USB_CTL_LAST_DATA_IN;
            break;

        case USB_CTL_LAST_DATA_IN:
            /* last packet is MPS multiple, so send ZLP packet */
            if (udev->dev.control.ctl_zlp) {
                (void)usbd_ep_send (udev, 0U, NULL, 0U);
                udev->dev.control.ctl_zlp = 0U;
            } else {
                if (udev->dev.cur_status == (uint8_t)USBD_CONFIGURED) {
                    if (udev->dev.class_core->ctlx_in != NULL) {
                        (void)udev->dev.class_core->ctlx_in (udev);
                    }
                }

                transc->remain_len = 0U;
				udev->dev.control.ctl_state = (uint8_t)USB_CTL_STATUS_OUT;
				(void)usbd_ep_recev (udev, 0U, NULL, 0U);
				usb_ctlep_startout(udev);
            }
            break;

        default:
            break;
        }
    } else 
	{
        if ((udev->dev.cur_status == (uint8_t)USBD_CONFIGURED) && (udev->dev.class_core->data_in != NULL)) {
            (void)udev->dev.class_core->data_in (udev, ep_num);
        }
    }

    return (uint8_t)USBD_OK;
}
