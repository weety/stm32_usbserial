
#include <board.h>
#include <rtthread.h>
#include "vcom_proc.h"

#define VCOM_BUFSZ 2048
static rt_uint32_t vcom_buffer[VCOM_BUFSZ/4];
static struct rt_semaphore vcom_sem;
rt_err_t vcom_rx_ind(rt_device_t dev, rt_size_t size)
{
	rt_sem_release(&vcom_sem);

	return RT_EOK;
}

void vcom_thread_entry(void* parameter)
{
	rt_err_t err;
	rt_size_t size = 0;
	rt_uint8_t *buf;
	rt_device_t vcom_dev;
	int count = 4;
	int offset = 0;

	buf = (rt_uint8_t *)vcom_buffer;

	rt_sem_init(&vcom_sem, "vcom", 0, RT_IPC_FLAG_FIFO);
	vcom_dev = rt_device_find("vcom");

	if (!vcom_dev)
	{
		rt_kprintf("open device vcom failed\n");
	}

	err = rt_device_open(vcom_dev, RT_DEVICE_OFLAG_RDWR);
	if (err != RT_EOK)
	{
		rt_kprintf("open vcom failed\n");
		return;
	}

	rt_device_set_rx_indicate(vcom_dev, vcom_rx_ind);

	while(1)
	{
		err = rt_sem_take(&vcom_sem, RT_WAITING_FOREVER);
		if (err != RT_EOK)
		{
			rt_kprintf("wait sem vcom error\n");
			break;
		}

		size = rt_device_read(vcom_dev, 0, buf, VCOM_BUFSZ);
		if (size > 0)
		{
			rt_device_write(vcom_dev, 0, buf, size);
		}

	}

	rt_device_set_rx_indicate(vcom_dev, RT_NULL);
	rt_device_close(vcom_dev);
	rt_sem_detach(&vcom_sem);
}


void vcom_init(void)
{
	
	rt_thread_t vcom_thread;

	vcom_thread = rt_thread_create("vcom",
								vcom_thread_entry, RT_NULL,
								512, 16, 20);

	if (vcom_thread != RT_NULL)
		rt_thread_startup(vcom_thread);
}


