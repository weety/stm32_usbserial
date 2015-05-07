/*
 * File      : cdc_vcom.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-10-02     Yi Qiu       first version
 * 2012-12-12     heyuanjie87  change endpoints and function handler 
 * 2013-06-25     heyuanjie87  remove SOF mechinism
 * 2013-07-20     Yi Qiu   do more test
 */

#include "cdc_vcom.h"
#include <rthw.h>

#ifdef RT_USB_DEVICE_CDC

#define TX_TIMEOUT              100
#define CDC_RX_BUFSIZE          2048
static rt_uint8_t rx_rbp[CDC_RX_BUFSIZE];
static struct rt_ringbuffer rx_ringbuffer;
static struct serial_ringbuffer vcom_int_rx;

static rt_vcom_t _vcom;

#define CDC_MaxPacketSize 64
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t rx_buf[CDC_RX_BUFSIZE];

volatile static rt_bool_t vcom_connected = RT_FALSE;
volatile static rt_bool_t vcom_in_sending = RT_FALSE;

static struct udevice_descriptor dev_desc =
{
    USB_DESC_LENGTH_DEVICE,     //bLength;
    USB_DESC_TYPE_DEVICE,       //type;
    USB_BCD_VERSION,            //bcdUSB;
    USB_CLASS_CDC,              //bDeviceClass;
    0x00,                       //bDeviceSubClass;
    0x00,                       //bDeviceProtocol;
    CDC_MaxPacketSize,          //bMaxPacketSize0;
    _VENDOR_ID,                 //idVendor;
    _PRODUCT_ID,                //idProduct;
    USB_BCD_DEVICE,             //bcdDevice;
    USB_STRING_MANU_INDEX,      //iManufacturer;
    USB_STRING_PRODUCT_INDEX,   //iProduct;
    USB_STRING_SERIAL_INDEX,    //iSerialNumber;
    USB_DYNAMIC,                //bNumConfigurations;
};

/* communcation interface descriptor */
const static struct ucdc_comm_descriptor _comm_desc =
{
#ifdef RT_USB_DEVICE_COMPOSITE
    /* Interface Association Descriptor */
    USB_DESC_LENGTH_IAD,
    USB_DESC_TYPE_IAD,
    USB_DYNAMIC,
    0x02,
    USB_CDC_CLASS_COMM,
    USB_CDC_SUBCLASS_ACM,
    USB_CDC_PROTOCOL_V25TER,
    0x00,
#endif
    /* Interface Descriptor */
    USB_DESC_LENGTH_INTERFACE,
    USB_DESC_TYPE_INTERFACE,
    USB_DYNAMIC,
    0x00,
    0x01,
    USB_CDC_CLASS_COMM,
    USB_CDC_SUBCLASS_ACM,
    USB_CDC_PROTOCOL_V25TER,
    0x00,
    /* Header Functional Descriptor */
    0x05,
    USB_CDC_CS_INTERFACE,
    USB_CDC_SCS_HEADER,
    0x0110,
    /* Call Management Functional Descriptor */
    0x05,
    USB_CDC_CS_INTERFACE,
    USB_CDC_SCS_CALL_MGMT,
    0x00,
    USB_DYNAMIC,
    /* Abstract Control Management Functional Descriptor */
    0x04,
    USB_CDC_CS_INTERFACE,
    USB_CDC_SCS_ACM,
    0x02,
    /* Union Functional Descriptor */
    0x05,
    USB_CDC_CS_INTERFACE,
    USB_CDC_SCS_UNION,
    USB_DYNAMIC,
    USB_DYNAMIC,
    /* Endpoint Descriptor */
    USB_DESC_LENGTH_ENDPOINT,
    USB_DESC_TYPE_ENDPOINT,
    USB_DYNAMIC | USB_DIR_IN,
    USB_EP_ATTR_INT,
    0x08,
    0xFF,
};

/* data interface descriptor */
const static struct ucdc_data_descriptor _data_desc =
{
    /* interface descriptor */
    USB_DESC_LENGTH_INTERFACE,
    USB_DESC_TYPE_INTERFACE,
    USB_DYNAMIC,
    0x00,
    0x02,
    USB_CDC_CLASS_DATA,
    0x00,
    0x00,
    0x00,
    /* endpoint, bulk out */
    USB_DESC_LENGTH_ENDPOINT,
    USB_DESC_TYPE_ENDPOINT,
    USB_DYNAMIC | USB_DIR_OUT,
    USB_EP_ATTR_BULK,
    USB_CDC_BUFSIZE,
    0x00,
    /* endpoint, bulk in */
    USB_DESC_LENGTH_ENDPOINT,
    USB_DESC_TYPE_ENDPOINT,
    USB_DYNAMIC | USB_DIR_IN,
    USB_EP_ATTR_BULK,
    USB_CDC_BUFSIZE,
    0x00,
};

const static char* _ustring[] =
{
    "Language",
    "EZVIZ",
    "F1 Virtual Serial",
    "28160705",
    "Config",
    "Intf",
};

static void _vcom_reset_state(void)
{
    int lvl = rt_hw_interrupt_disable();
    vcom_connected = RT_FALSE;
    vcom_in_sending = RT_FALSE;
    /*rt_kprintf("reset USB serial\n", cnt);*/
    rt_hw_interrupt_enable(lvl);
}

/**
 * This function will handle cdc bulk in endpoint request.
 *
 * @param func the usb function object.
 * @param size request size.
 *
 * @return RT_EOK.
 */
static rt_err_t _ep_in_handler(ufunction_t func, rt_size_t size)
{
    cdc_eps_t eps;

    RT_ASSERT(func != RT_NULL);

    RT_DEBUG_LOG(RT_DEBUG_USB, ("_ep_in_handler %d\n", size));

    eps = (cdc_eps_t)func->eps;
    if ((size != 0) && (size % CDC_MaxPacketSize == 0))
    {
        /* don't have data right now. Send a zero-length-packet to
         * terminate the transaction.
         *
         * FIXME: actually, this might not be the right place to send zlp.
         * Only the rt_device_write could know how much data is sending. */
        vcom_in_sending = RT_TRUE;

        dcd_ep_write(func->device->dcd, eps->ep_in, RT_NULL, 0);

        return RT_EOK;
    }
    
    rt_completion_done(&_vcom.wait);
    
    return RT_EOK;
}

/**
 * This function will handle cdc bulk out endpoint request.
 *
 * @param func the usb function object.
 * @param size request size.
 *
 * @return RT_EOK.
 */
static rt_err_t _ep_out_handler(ufunction_t func, rt_size_t size)
{
    rt_uint32_t level;
    cdc_eps_t eps;

    RT_ASSERT(func != RT_NULL);

    RT_DEBUG_LOG(RT_DEBUG_USB, ("_ep_out_handler %d\n", size));
    
    eps = (cdc_eps_t)func->eps;
    /* receive data from USB VCOM */
    level = rt_hw_interrupt_disable();

    rt_ringbuffer_put(&rx_ringbuffer, eps->ep_out->buffer, size);
    rt_hw_interrupt_enable(level);

    /* notify receive data */
    rt_hw_serial_isr(&_vcom.sdev);

    dcd_ep_read(func->device->dcd, eps->ep_out, eps->ep_out->buffer,
                eps->ep_out->ep_desc->wMaxPacketSize);

    return RT_EOK;
}

/**
 * This function will handle cdc interrupt in endpoint request.
 *
 * @param device the usb device object.
 * @param size request size.
 *
 * @return RT_EOK.
 */
static rt_err_t _ep_cmd_handler(ufunction_t func, rt_size_t size)
{
    RT_ASSERT(func != RT_NULL);

    RT_DEBUG_LOG(RT_DEBUG_USB, ("_ep_cmd_handler\n"));

    return RT_EOK;
}

/**
 * This function will handle cdc_get_line_coding request.
 *
 * @param device the usb device object.
 * @param setup the setup request.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _cdc_get_line_coding(udevice_t device, ureq_t setup)
{
    struct ucdc_line_coding data;
    rt_uint16_t size;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(setup != RT_NULL);

    data.dwDTERate = 115200;
    data.bCharFormat = 0;
    data.bDataBits = 8;
    data.bParityType = 0;
    size = setup->length > 7 ? 7 : setup->length;

    dcd_ep_write(device->dcd, 0, (void*)&data, size);

    return RT_EOK;
}

/**
 * This function will handle cdc_set_line_coding request.
 *
 * @param device the usb device object.
 * @param setup the setup request.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _cdc_set_line_coding(udevice_t device, ureq_t setup)
{
    struct ucdc_line_coding data;
    rt_err_t ret;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(setup != RT_NULL);

    rt_completion_init(&device->dcd->completion);

    dcd_ep_read(device->dcd, 0, (void*)&data, setup->length);

    ret = rt_completion_wait(&device->dcd->completion, 100);
    if(ret != RT_EOK)
    {
        rt_kprintf("_cdc_set_line_coding timeout\n");
    }

    return RT_EOK;
}

/**
 * This function will handle cdc interface request.
 *
 * @param device the usb device object.
 * @param setup the setup request.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _interface_handler(ufunction_t func, ureq_t setup)
{
    RT_ASSERT(func != RT_NULL);
    RT_ASSERT(func->device != RT_NULL);
    RT_ASSERT(setup != RT_NULL);

    switch(setup->request)
    {
    case CDC_SEND_ENCAPSULATED_COMMAND:
        break;
    case CDC_GET_ENCAPSULATED_RESPONSE:
        break;
    case CDC_SET_COMM_FEATURE:
        break;
    case CDC_GET_COMM_FEATURE:
        break;
    case CDC_CLEAR_COMM_FEATURE:
        break;
    case CDC_SET_LINE_CODING:
        _cdc_set_line_coding(func->device, setup);
        vcom_connected = RT_TRUE;
        break;
    case CDC_GET_LINE_CODING:
        _cdc_get_line_coding(func->device, setup);
        break;
    case CDC_SET_CONTROL_LINE_STATE:
        dcd_send_status(func->device->dcd);
        break;
    case CDC_SEND_BREAK:
        break;
    default:
        rt_kprintf("unknown cdc request\n",setup->request_type);
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * This function will run cdc function, it will be called on handle set configuration request.
 *
 * @param func the usb function object.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _function_run(ufunction_t func)
{
    cdc_eps_t eps;
    RT_ASSERT(func != RT_NULL);

    RT_DEBUG_LOG(RT_DEBUG_USB, ("cdc function run\n"));
    eps = (cdc_eps_t)func->eps;

    eps->ep_out->buffer = rx_buf;

    _vcom_reset_state();

    dcd_ep_read(func->device->dcd, eps->ep_out, eps->ep_out->buffer,
                eps->ep_out->ep_desc->wMaxPacketSize);

    return RT_EOK;
}

/**
 * This function will stop cdc function, it will be called on handle set configuration request.
 *
 * @param func the usb function object.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _function_stop(ufunction_t func)
{
    RT_ASSERT(func != RT_NULL);

    RT_DEBUG_LOG(RT_DEBUG_USB, ("cdc function stop\n"));

    _vcom_reset_state();

    return RT_EOK;
}

static struct ufunction_ops ops =
{
    _function_run,
    _function_stop,
    RT_NULL,
};

/**
 * This function will configure cdc descriptor.
 *
 * @param comm the communication interface number.
 * @param data the data interface number.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _cdc_descriptor_config(ucdc_comm_desc_t comm, 
    rt_uint8_t cintf_nr, ucdc_data_desc_t data, rt_uint8_t dintf_nr)
{
    comm->call_mgmt_desc.data_interface = dintf_nr;
    comm->union_desc.master_interface = cintf_nr;
    comm->union_desc.slave_interface0 = dintf_nr;
#ifdef RT_USB_DEVICE_COMPOSITE
    comm->iad_desc.bFirstInterface = cintf_nr;
#endif

    return RT_EOK;
}

/**
 * This function will create a cdc function instance.
 *
 * @param device the usb device object.
 *
 * @return RT_EOK on successful.
 */
ufunction_t rt_usbd_function_cdc_create(udevice_t device)
{
    ufunction_t func;
    cdc_eps_t eps;
    uintf_t intf_comm, intf_data;
    ualtsetting_t comm_setting, data_setting;
    ucdc_data_desc_t data_desc;
    ucdc_comm_desc_t comm_desc;

    /* parameter check */
    RT_ASSERT(device != RT_NULL);

    /* set usb device string description */
    rt_usbd_device_set_string(device, _ustring);
    /* create a cdc function */
    func = rt_usbd_function_new(device, &dev_desc, &ops);
    /* create a cdc function endpoints collection */
    eps = rt_malloc(sizeof(struct cdc_eps));
    func->eps = (void*)eps;

    /* create a cdc communication interface and a cdc data interface */
    intf_comm = rt_usbd_interface_new(device, _interface_handler);
    intf_data = rt_usbd_interface_new(device, _interface_handler);

    /* create a communication alternate setting and a data alternate setting */
    comm_setting = rt_usbd_altsetting_new(sizeof(struct ucdc_comm_descriptor));
    data_setting = rt_usbd_altsetting_new(sizeof(struct ucdc_data_descriptor));

    /* config desc in alternate setting */
    rt_usbd_altsetting_config_descriptor(comm_setting, &_comm_desc,
                                         (rt_off_t)&((ucdc_comm_desc_t)0)->intf_desc);
    rt_usbd_altsetting_config_descriptor(data_setting, &_data_desc, 0);
    /* configure the cdc interface descriptor */
    _cdc_descriptor_config(comm_setting->desc, intf_comm->intf_num, data_setting->desc, intf_data->intf_num);

    /* create a command endpoint */
    comm_desc = (ucdc_comm_desc_t)comm_setting->desc;
    eps->ep_cmd = rt_usbd_endpoint_new(&comm_desc->ep_desc, _ep_cmd_handler);

    /* add the command endpoint to the cdc communication interface */
    rt_usbd_altsetting_add_endpoint(comm_setting, eps->ep_cmd);

    /* add the communication alternate setting to the communication interface,
       then set default setting of the interface */
    rt_usbd_interface_add_altsetting(intf_comm, comm_setting);
    rt_usbd_set_altsetting(intf_comm, 0);

    /* add the communication interface to the cdc function */
    rt_usbd_function_add_interface(func, intf_comm);

    /* create a bulk in and a bulk endpoint */
    data_desc = (ucdc_data_desc_t)data_setting->desc;
    eps->ep_out = rt_usbd_endpoint_new(&data_desc->ep_out_desc, _ep_out_handler);
    eps->ep_in = rt_usbd_endpoint_new(&data_desc->ep_in_desc, _ep_in_handler);

    /* add the bulk out and bulk in endpoints to the data alternate setting */
    rt_usbd_altsetting_add_endpoint(data_setting, eps->ep_in);
    rt_usbd_altsetting_add_endpoint(data_setting, eps->ep_out);

    /* add the data alternate setting to the data interface
            then set default setting of the interface */
    rt_usbd_interface_add_altsetting(intf_data, data_setting);
    rt_usbd_set_altsetting(intf_data, 0);

    /* add the cdc data interface to cdc function */
    rt_usbd_function_add_interface(func, intf_data);
    
    _vcom.udev = device;
    _vcom.eps = eps;

    return func;
}

/**
* UART device in RT-Thread
*/
static rt_err_t _vcom_configure(struct rt_serial_device *serial,
                                struct serial_configure *cfg)
{
    return RT_EOK;
}

static rt_err_t _vcom_control(struct rt_serial_device *serial,
                              int cmd, void *arg)
{
    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        break;
    }

    return RT_EOK;
}

static int _vcom_getc(struct rt_serial_device *serial)
{
    int result;
    rt_uint8_t ch;
    rt_uint32_t level;

    result = -1;

    level = rt_hw_interrupt_disable();
    if (RT_RINGBUFFER_SIZE(&rx_ringbuffer))
    {
        rt_ringbuffer_getchar(&rx_ringbuffer, &ch);
        result = ch;
    }
    rt_hw_interrupt_enable(level);

    return result;
}

static rt_size_t _vcom_int_tx(struct rt_serial_device *serial, 
    const char *buf, rt_size_t size)
{
    rt_vcom_t *vcom;
    rt_size_t tsize = 0;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(buf != RT_NULL);
    
    if (!vcom_connected)
    {
        return 0;
    }
            
    vcom = (rt_vcom_t*)serial->parent.user_data;    
    rt_completion_init(&vcom->wait);    
    dcd_ep_write(vcom->udev->dcd, vcom->eps->ep_in, (void*)buf, size);  
    if (rt_completion_wait(&vcom->wait, TX_TIMEOUT) == RT_EOK)
    {
        tsize = size;
    }
    else
    {
        rt_kprintf("vcom tx timeout\n");
    }
    
    return tsize;
}

static const struct rt_uart_ops usb_vcom_ops =
{
    _vcom_configure,
    _vcom_control,
    RT_NULL,
    _vcom_getc,
    RT_NULL,
    _vcom_int_tx,
};

void rt_usb_vcom_init(void)
{
    struct serial_configure config;

    /* initialize ring buffer */
    rt_ringbuffer_init(&rx_ringbuffer, rx_rbp, CDC_RX_BUFSIZE);

    config.baud_rate = BAUD_RATE_115200;
    config.bit_order = BIT_ORDER_LSB;
    config.data_bits = DATA_BITS_8;
    config.parity = PARITY_NONE;
    config.stop_bits = STOP_BITS_1;
    config.invert = NRZ_NORMAL;

    _vcom.sdev.ops = &usb_vcom_ops;
    _vcom.sdev.int_rx = &vcom_int_rx;
    _vcom.sdev.config = config;

    /* register vcom device */
    rt_hw_serial_register(&_vcom.sdev, "vcom",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX,
                          &_vcom);
}

#endif

