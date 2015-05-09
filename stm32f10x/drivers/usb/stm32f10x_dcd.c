#include <rtthread.h>
#include <stm32f10x.h>
#include <rtdevice.h>
#include "usb_core.h"
#include "usb_regs.h"
#include "usb_conf.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include <com_lib.h>

#ifdef RT_USING_USB_DEVICE

#define vSetEPRxStatus(st) (SaveRState = st)
#define vSetEPTxStatus(st) (SaveTState = st)
#define Send0LengthData() { _SetEPTxCount(ENDP0, 0); \
    vSetEPTxStatus(EP_TX_VALID); \
  }

static volatile rt_uint32_t ControlState = WAIT_SETUP;

#define EP_MAX_BUF_SZ 64
struct buffer_desc {
	void *buf;
	rt_uint16_t remain;
	rt_uint16_t offset;
};

struct stm32_device {
	struct udcd stm32_dcd;
	//volatile void *ep_buf[8];
	volatile struct buffer_desc epin_desc[8]; 
	volatile struct buffer_desc epout_desc[8]; 
};

static struct stm32_device udev;

static rt_uint16_t ep_addr = 0xC0;
static rt_uint8_t ep_in_num = 1;
static rt_uint8_t ep_out_num = 1;

__IO uint16_t wIstr;  /* ISTR register last read value */
__IO uint8_t bIntPackSOF = 0;  /* SOFs received between 2 consecutive packets */
__IO uint32_t esof_counter =0; /* expected SOF counter */
__IO uint32_t wCNTR=0;

uint16_t  wInterrupt_Mask;


/**
* @brief  Setup0_Process
*         Handle the setup stage
* @retval status
*/
static struct ureqest setup;

uint8_t Setup0_Process(void)
{

  union
  {
    uint8_t* b;
    uint16_t* w;
  } pBuf;
  struct udev_msg msg;
  uint16_t offset = 1;
  
  pBuf.b = PMAAddr + (uint8_t *)(_GetEPRxAddr(ENDP0) * 2); /* *2 for 32 bits addr */

  setup.request_type = *pBuf.b++; /* bmRequestType */
  setup.request = *pBuf.b++; /* bRequest */
  pBuf.w += offset;  /* word not accessed because of 32 bits addressing */
  setup.value = *pBuf.w++; /* wValue */
  pBuf.w += offset;  /* word not accessed because of 32 bits addressing */
  setup.index  = *pBuf.w++; /* wIndex */
  pBuf.w += offset;  /* word not accessed because of 32 bits addressing */
  setup.length = *pBuf.w; /* wLength */

  ControlState = SETTING_UP;

  msg.type = USB_MSG_SETUP_NOTIFY;
  msg.dcd = &udev.stm32_dcd;
  msg.content.setup_msg.packet = (rt_uint32_t*)&setup;

  rt_usbd_post_event(&msg, sizeof(struct udev_msg));

  return 0;
}

void USBD_DataOutStage(uint8_t epnum)
{
	rt_uint16_t size;
	void *buf;
    struct udev_msg msg;
	volatile struct buffer_desc *pdesc = &udev.epout_desc[epnum];

	size = GetEPRxCount(epnum & 0x7F);
	buf = (void *)((rt_uint32_t)pdesc->buf + pdesc->offset);
	PMAToUserBufferCopy(buf, GetEPRxAddr(epnum), size);
	pdesc->remain -= size;
	pdesc->offset += size;
	if ((pdesc->remain > 0) && (size == EP_MAX_BUF_SZ))
	{
		if (pdesc->remain >= EP_MAX_BUF_SZ)
		{
			size = EP_MAX_BUF_SZ;
		}
		else
		{
			size = pdesc->remain;
		}
		SetEPRxCount(epnum, size);
		SetEPRxValid(epnum);
	}
	else
	{
	    if(epnum == 0)
	    {
	    	if (ControlState == WAIT_STATUS_OUT)
	    	{
	    		ControlState = STALLED;
				SetEPRxCount(ENDP0, EP_MAX_BUF_SZ);

				vSetEPRxStatus(EP_RX_STALL);
				vSetEPTxStatus(EP_TX_STALL);
				_SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);
			}
			else if (ControlState == OUT_DATA)
			{
				ControlState = WAIT_STATUS_IN;
				Send0LengthData();
				_SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);
			}
			else
			{
				rt_kprintf("OUT ERROR\n");
				_SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);
			}
	    }
	    else
	    {
	        msg.type = USB_MSG_DATA_NOTIFY;
	        msg.dcd = &udev.stm32_dcd;
	        msg.content.ep_msg.ep_addr = epnum;
	        msg.content.ep_msg.size = pdesc->offset;

	        rt_usbd_post_event(&msg, sizeof(struct udev_msg));
	    }
	}

}

void USBD_DataInStage(uint8_t epnum)
{
	struct udev_msg msg;
	void *buf;
	rt_uint16_t size;

	volatile struct buffer_desc *pdesc = &udev.epin_desc[epnum];

    if (pdesc->remain == 0)
    {
    	if(epnum == 0)
	    {
	    	if (ControlState == IN_DATA)
	    	{
	    		ControlState = LAST_IN_DATA;
				SetEPTxCount(0, 0);
				vSetEPTxStatus(EP_TX_VALID);
				_SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);
			}
			else if (ControlState == LAST_IN_DATA)
			{
				ControlState = WAIT_STATUS_OUT;
		    	vSetEPTxStatus(EP_TX_STALL);
				_SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);
			}
			else if (ControlState == WAIT_STATUS_IN)
			{
				ControlState = STALLED;
				SetEPRxCount(ENDP0, EP_MAX_BUF_SZ);

				vSetEPRxStatus(EP_RX_STALL);
				vSetEPTxStatus(EP_TX_STALL);
				_SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);
				rt_completion_done(&udev.stm32_dcd.completion);
			}
			else
			{
				rt_kprintf("IN ERROR\n");
				vSetEPTxStatus(EP_TX_STALL);
				_SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);
			}
	    }
		else
		{
	        msg.type = USB_MSG_DATA_NOTIFY;
	        msg.dcd = &udev.stm32_dcd;
	        msg.content.ep_msg.ep_addr = epnum | USB_DIR_IN;
	        msg.content.ep_msg.size = pdesc->offset;

	        rt_usbd_post_event(&msg, sizeof(struct udev_msg));
		}
    }
	else
	{
		buf = (void *)((rt_uint32_t)pdesc->buf + pdesc->offset);
		if (pdesc->remain >= EP_MAX_BUF_SZ)
		{
			if (epnum == 0)
				ControlState = IN_DATA;
			size = EP_MAX_BUF_SZ;
		}
		else
		{
			if (epnum == 0)
				ControlState = LAST_IN_DATA;
			size = pdesc->remain;
		}

		pdesc->remain -= size;
		pdesc->offset += size;
		UserToPMABufferCopy(buf, GetEPTxAddr(epnum), size);

		/* Update the data length in the control register */
		if (epnum == 0)
		{
			SetEPTxCount(0, size);
			vSetEPTxStatus(EP_TX_VALID);
			vSetEPRxStatus(EP_RX_VALID);
			_SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);
		}
		else
		{
			SetEPTxCount(epnum, size);
			SetEPTxValid(epnum);
		}
    }

}

void SOF_Callback(void)
{
    struct udev_msg msg;

    msg.type = USB_MSG_SOF;
    msg.dcd = &udev.stm32_dcd;

    rt_usbd_post_event(&msg, sizeof(struct udev_msg));
}

static void usb_reset(void)
{
  u32 i;
  u32 nEP = 8;
  struct udev_msg msg;
  rt_kprintf("RESET\n");
  ep_addr = 0xC0;
  ep_in_num = 1;
  ep_out_num = 1;
  SetBTABLE(BTABLE_ADDRESS);

  /* Initialize Endpoint 0 */
  SetEPType(ENDP0, EP_CONTROL);
  SetEPTxStatus(ENDP0, EP_TX_STALL);
  SetEPRxAddr(ENDP0, ENDP0_RXADDR);
  SetEPTxAddr(ENDP0, ENDP0_TXADDR);
  Clear_Status_Out(ENDP0);
  SetEPRxCount(ENDP0, 64);
  SetEPRxValid(ENDP0);

  /* Set this device to response on default address */

  /* set address in every used endpoint */
  for (i = 0; i < nEP; i++)
  {
  	_SetEPAddress((u8)i, (u8)i);
  } /* for */
  _SetDADDR(0 | DADDR_EF); /* set device address and enable function */

  msg.type = USB_MSG_RESET;
  msg.dcd = &udev.stm32_dcd;

  rt_usbd_post_event(&msg, sizeof(struct udev_msg));
}


/******************************************************************************
*
* Function Name  : USB_Istr
* Description    : STR events interrupt service routine
* Input          :
* Output         :
* Return         :
*******************************************************************************/
void USB_Istr(void)
{

  wIstr = _GetISTR();

#if (IMR_MSK & ISTR_RESET)
  if (wIstr & ISTR_RESET & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_RESET);
    usb_reset();
#ifdef RESET_CALLBACK
    RESET_Callback();
#endif
  }
#endif
  /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_DOVR)
  if (wIstr & ISTR_DOVR & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_DOVR);
#ifdef DOVR_CALLBACK
    DOVR_Callback();
#endif
  }
#endif
  /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_ERR)
  if (wIstr & ISTR_ERR & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_ERR);
#ifdef ERR_CALLBACK
    ERR_Callback();
#endif
  }
#endif
  /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_WKUP)
  if (wIstr & ISTR_WKUP & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_WKUP);
    Resume(RESUME_EXTERNAL);
#ifdef WKUP_CALLBACK
    WKUP_Callback();
#endif
  }
#endif
  /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_SUSP)
  if (wIstr & ISTR_SUSP & wInterrupt_Mask)
  {

    /* check if SUSPEND is possible */
    if (fSuspendEnabled)
    {
      Suspend();
    }
    else
    {
      /* if not possible then resume after xx ms */
      Resume(RESUME_LATER);
    }
    /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
    _SetISTR((u16)CLR_SUSP);
#ifdef SUSP_CALLBACK
    SUSP_Callback();
#endif
  }
#endif
  /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_SOF)
  if (wIstr & ISTR_SOF & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_SOF);
    bIntPackSOF++;

#ifdef SOF_CALLBACK
    SOF_Callback();
#endif
  }
#endif
  /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_ESOF)
  if (wIstr & ISTR_ESOF & wInterrupt_Mask)
  {
    _SetISTR((u16)CLR_ESOF);
    /* resume handling timing is made with ESOFs */
    Resume(RESUME_ESOF); /* request without change of the machine state */

#ifdef ESOF_CALLBACK
    ESOF_Callback();
#endif
  }
#endif
  /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_CTR)
  if (wIstr & ISTR_CTR & wInterrupt_Mask)
  {
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the sub */
    CTR_LP();
#ifdef CTR_CALLBACK
    CTR_Callback();
#endif
  }
#endif
} /* USB_Istr */

#if 0
/******************************************************************************
*
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 
interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN_RX0_IRQHandler(void)
{
    USB_Istr();
}
#endif

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* enter interrupt */
  rt_interrupt_enter();
  USB_Istr();
  rt_interrupt_leave();
}


static rt_err_t ep_stall(uep_t ep)
{
	uep_desc_t ep_desc;
	rt_uint8_t epnum;
	rt_uint8_t dir;
	
    if(ep == 0)
    {
    	rt_kprintf("ep0 stall\n");
		SetEPRxStatus(0, EP_RX_STALL);
    	SetEPTxStatus(0, EP_TX_STALL);
    }
    else
    {
		ep_desc = ep->ep_desc;
		epnum = ep_desc->bEndpointAddress & USB_EPNO_MASK;
		dir = ep_desc->bEndpointAddress & USB_DIR_MASK;
		if (dir == USB_DIR_IN)
		{
			SetEPTxStatus(epnum, EP_TX_STALL);
		}
		else
		{
			SetEPRxStatus(epnum, EP_RX_STALL);
		}
	}

    return RT_EOK;
}

static rt_err_t set_address(rt_uint8_t address)
{
    u32 i;
	u32 nEP = 8;

	/* set address in every used endpoint */
	for (i = 0; i < nEP; i++)
	{
		_SetEPAddress((u8)i, (u8)i);
	} /* for */
	_SetDADDR(address | DADDR_EF); /* set device address and enable function */

	uart2_send_cmd_status(COM_CMD_SEND_CONNECT_STATUS, RT_NULL, 0, 0, 0);

    return RT_EOK;
}

static rt_err_t clear_feature(rt_uint16_t value, rt_uint16_t index)
{

    return RT_ERROR;
}

static rt_err_t set_feature(rt_uint16_t value, rt_uint16_t index)
{

    return RT_ERROR;
}

static rt_err_t ep_alloc(uep_t ep)
{
    uep_desc_t ep_desc = ep->ep_desc;

    RT_ASSERT(ep != RT_NULL);

    if(ep_desc->bEndpointAddress & USB_DIR_IN)
    {
        if (ep_in_num > 3)
            return -RT_ERROR;
        ep_desc->bEndpointAddress |= ep_in_num++;
    }
    else
    {
        if (ep_out_num > 3)
            return -RT_ERROR;
        ep_desc->bEndpointAddress |= ep_out_num++;
    }

    return RT_EOK;
}

static rt_err_t ep_free(uep_t ep)
{
    return RT_EOK;
}

static rt_err_t ep_run(uep_t ep)
{
	uep_desc_t ep_desc;
	rt_uint8_t epnum;
	rt_uint8_t dir;
	rt_uint16_t ep_type;
	
	RT_ASSERT(ep != RT_NULL);
	
    ep_desc = ep->ep_desc;
	epnum = ep_desc->bEndpointAddress & USB_EPNO_MASK;
	dir = ep_desc->bEndpointAddress & USB_DIR_MASK;

	switch (ep_desc->bmAttributes & USB_EP_ATTR_TYPE_MASK)
	{
	case USB_EP_ATTR_CONTROL:
		ep_type = EP_CONTROL;
		break;
	case USB_EP_ATTR_ISOC:
		ep_type = EP_ISOCHRONOUS;
		break;
	case USB_EP_ATTR_BULK:
		ep_type = EP_BULK;
		break;
	case USB_EP_ATTR_INT:
	default:
		ep_type = EP_INTERRUPT;
		break;
	}

	SetEPType(epnum, ep_type);

	if (dir == USB_DIR_IN)
	{
		
		//SetEPTxAddr(epnum, 0x40 + 0x40 * (2 * epnum + 1));
		SetEPTxAddr(epnum, ep_addr);
		ep_addr += ep_desc->wMaxPacketSize;
		SetEPTxCount(epnum, ep_desc->wMaxPacketSize);
		if (ep_type != EP_ISOCHRONOUS)
			ClearDTOG_TX(epnum);
		SetEPTxStatus(epnum, EP_TX_NAK);
		rt_kprintf("IN EP%d[0x%02x]\n", epnum, GetEPTxAddr(epnum));
	}
	else
	{
		//SetEPRxAddr(epnum, 0x40 + 0x40 * (2 * epnum));
		SetEPRxAddr(epnum, ep_addr);
		ep_addr += ep_desc->wMaxPacketSize;
		SetEPRxCount(epnum, ep_desc->wMaxPacketSize);
		if (ep_type != EP_ISOCHRONOUS)
			ClearDTOG_RX(epnum);
		SetEPRxStatus(epnum, EP_RX_VALID);
		rt_kprintf("OUT EP%d[0x%02x]\n", epnum, GetEPRxAddr(epnum));
	}

    return RT_EOK;
}

static rt_err_t ep_stop(uep_t ep)
{
	uep_desc_t ep_desc;
	rt_uint8_t epnum;
	rt_uint8_t dir;
    RT_ASSERT(ep != RT_NULL);

	ep_desc = ep->ep_desc;
	epnum = ep_desc->bEndpointAddress & USB_EPNO_MASK;
	dir = ep_desc->bEndpointAddress & USB_DIR_MASK;

    if (dir == USB_DIR_IN)
	{
		SetEPTxStatus(epnum, EP_TX_DIS);
	}
	else
	{
		SetEPRxStatus(epnum, EP_RX_DIS);
	}

    return RT_EOK;
}

static rt_err_t ep_read(uep_t ep, void *buffer, rt_size_t size)
{
	int epnum;
	volatile struct buffer_desc *pdesc;

	epnum = ep ? ep->ep_desc->bEndpointAddress & 0x7F : 0;

	pdesc = &udev.epout_desc[epnum];
	pdesc->buf = buffer;
	pdesc->remain = size;
	pdesc->offset = 0;
	if (pdesc->remain >= EP_MAX_BUF_SZ)
	{
		size = EP_MAX_BUF_SZ;
	}

	//rt_kprintf("R EP%d(%d)\n", ep ? ep->ep_desc->bEndpointAddress & 0x7F : 0, size);

    if(ep == 0)
    {
		ControlState = OUT_DATA;
		SetEPRxCount(0, size);
		vSetEPRxStatus(EP_RX_VALID); /* enable for next data reception */
		_SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);
	}
    else
    {
		SetEPRxCount(epnum, size);
		SetEPRxValid(epnum);
    }

    return RT_EOK;
}

void usb_write_buffer(rt_uint8_t epnum, void *buffer, rt_size_t size)
{
	volatile struct buffer_desc *pdesc = &udev.epin_desc[epnum];

	pdesc->buf = buffer;
	pdesc->remain = size;
	pdesc->offset = 0;

	if (size > 0)
	{
		if (pdesc->remain >= EP_MAX_BUF_SZ)
		{
			size = EP_MAX_BUF_SZ;
		}

		pdesc->remain -= size;
		pdesc->offset += size;

		UserToPMABufferCopy(buffer, GetEPTxAddr(epnum), size);
	}

	/* Update the data length in the control register */
	SetEPTxCount(epnum, size);
}

static rt_size_t ep_write(uep_t ep, void *buffer, rt_size_t size)
{
    uep_desc_t ep_desc;

	//rt_kprintf("W EP%d(%d)\n", ep ? ep->ep_desc->bEndpointAddress & 0x7F : 0, size);

    if(ep == 0)
    {
    	if (size == 0)
			ControlState = WAIT_STATUS_IN;
		else if (size >= EP_MAX_BUF_SZ)
			ControlState = IN_DATA;
		else
			ControlState = LAST_IN_DATA;
		usb_write_buffer(0, buffer, size);
		vSetEPTxStatus(EP_TX_VALID);
		vSetEPRxStatus(EP_RX_VALID);
		_SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);
    }
    else
    {
    	ep_desc = ep->ep_desc;
		usb_write_buffer(ep_desc->bEndpointAddress & 0x7F, buffer, size);
		SetEPTxValid(ep_desc->bEndpointAddress & 0x7F);
    }

    return size;
}

static rt_err_t send_status(void)
{
	ControlState = WAIT_STATUS_IN;
	usb_write_buffer(ENDP0, 0, 0);
	vSetEPTxStatus(EP_TX_VALID);
	_SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);

    return RT_EOK;
}

static struct udcd_ops stm32_dcd_ops =
{
    set_address,
    clear_feature,
    set_feature,
    ep_alloc,
    ep_free,
    ep_stall,
    ep_run,
    ep_stop,
    ep_read,
    ep_write,
    send_status,
};

// GPIO define
#define USBHUB_RESET      GPIOB
#define USBHUB_RESET_PIN  GPIO_Pin_1
#define RCC_APB2Periph_GPIO_USBHUB_RESET      RCC_APB2Periph_GPIOB

void rt_hw_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_USBHUB_RESET, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin   = USBHUB_RESET_PIN;
    GPIO_Init(USBHUB_RESET, &GPIO_InitStructure);
    GPIO_SetBits(USBHUB_RESET, USBHUB_RESET_PIN);
}
void usbhub_reset(void)
{
	GPIO_ResetBits(USBHUB_RESET, USBHUB_RESET_PIN);
	rt_thread_delay(2);
	GPIO_SetBits(USBHUB_RESET, USBHUB_RESET_PIN);
}

static rt_err_t stm32_dcd_init(rt_device_t device)
{
    rt_kprintf("stm32 usb init\n");

	rt_hw_gpio_init();
	//usbhub_reset();
    Set_System();
	Set_USBClock();
	USB_Interrupts_Config();
	/* Connect the device */
  	PowerOn();

  	/* Perform basic device initialization operations */
  	USB_SIL_Init();

	USB_Cable_Config(ENABLE);

    return RT_EOK;
}

void rt_hw_usbd_init(void)
{
    udev.stm32_dcd.parent.type = RT_Device_Class_USBDevice;
    udev.stm32_dcd.parent.init = stm32_dcd_init;

    udev.stm32_dcd.ops = &stm32_dcd_ops;
    rt_completion_init(&udev.stm32_dcd.completion);

    rt_device_register(&udev.stm32_dcd.parent, "usbd", 0);
}

void usb_init(void)
{
	rt_thread_t heartbeat_thread;
#ifdef RT_USB_DEVICE_CDC
	rt_usb_vcom_init();
#endif
#ifdef RT_USB_DEVICE_MSTORAGE
	rt_hw_flash_init();
#endif
	rt_hw_usbd_init();
	rt_usb_device_init();
}

#ifdef RT_USING_FINSH
extern void finsh_set_device(const char* device_name);

void set_finsh_dev(char *device)
{
#ifdef RT_USING_CONSOLE
	rt_console_set_device(device);
#endif
	finsh_set_device( device );
}
#endif

#ifdef RT_USING_FINSH
#include "finsh.h"
FINSH_FUNCTION_EXPORT(usbhub_reset, reset usb hub chip);
FINSH_FUNCTION_EXPORT(rt_hw_gpio_init, init gpio);
FINSH_FUNCTION_EXPORT(set_finsh_dev, set finsh device);
FINSH_FUNCTION_EXPORT(usb_init, init usb device);

#endif

#endif
