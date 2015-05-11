#ifndef  __CDC_VCOM_H__
#define  __CDC_VCOM_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "cdc.h"

struct rt_vcom
{
    udevice_t udev;
    rt_serial_t sdev;
    cdc_eps_t eps;
    struct ucdc_line_coding line_code;
    struct rt_completion wait;
};
typedef struct rt_vcom rt_vcom_t;

#endif
