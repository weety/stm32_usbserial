
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
	SDK_HEAD *phead;
	COM_HEAD *pshead;
	volatile rt_uint32_t status = IDLE;
	int count = 4;
	int rx_valid = 1;
	int offset = 0;

	buf = (rt_uint8_t *)vcom_buffer;
	phead = (SDK_HEAD *)vcom_buffer;

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
		rx_valid = 1;

		while(rx_valid)
		{
			switch (status)
			{
			case IDLE:
				size = rt_device_read(vcom_dev, 0, buf, 1);
				if (size == 0)
				{
					rx_valid = 0;
					break;
				}

				if (*buf == 0x01)
				{
					status = CHECK_HEAD;
					count = 3;
					offset++;
				}
				//rt_kprintf("IDLE:0x%02x\n", *buf);
				break;
			case CHECK_HEAD:
				size = rt_device_read(vcom_dev, 0, buf + offset, count);
				if (size == 0)
				{
					rx_valid = 0;
					break;
				}
				if (size < count)
					rx_valid = 0;
				count -= size;
				offset += size;
				if (count == 0)
				{
					//rt_kprintf("CHECK_HEAD: 0x%04x\n", phead->protocol);
					if (phead->protocol == SDK_PROTOCOL_TYPE)
					{
						status = FOUND_HEAD;
						count = sizeof(SDK_HEAD) - 4;
						//rt_kprintf("CHECK_HEAD: found head\n");
					}
					else
					{
						count = 4;
						offset = 0;
						status = IDLE;
					}
				}
				break;
			case FOUND_HEAD:
				size = rt_device_read(vcom_dev, 0, buf + offset, count);
				if (size == 0)
				{
					rx_valid = 0;
					break;
				}
				if (size < count)
					rx_valid = 0;
				count -= size;
				offset += size;
				if (count == 0)
				{
					if (phead->id == SDK_PROTOCOL_ID)
					{
						/*rt_kprintf("FOUND_HEAD: version= 0x%04x, id=0x%04x, 0x%04x, packsize=%d\n", 
							phead->version, phead->id, phead->packet_type, phead->length);*/
						count = phead->length + 4;
						status = DATA;
					}
					else
					{
						count = 4;
						offset = 0;
						status = IDLE;
					}
				}
				break;
			case DATA:
				size = rt_device_read(vcom_dev, 0, buf + offset, count);
				if (size == 0)
				{
					rx_valid = 0;
					break;
				}
				if (size < count)
					rx_valid = 0;
				count -= size;
				offset += size;
				if (count == 0)
				{
					status = COMPLITE;
				}
				break;
			case COMPLITE:
				status = IDLE;
				count = 4;
				offset = 0;
				err = uart2_send_cmd_status(COM_CMD_FROM_PC, buf, sizeof(SDK_HEAD) + phead->length + 4, 
									0, RT_TICK_PER_SECOND * 10);
				if (err == RT_EOK)
				{
					pshead = (COM_HEAD *)buf;
					rt_device_write(vcom_dev, 0, buf + sizeof(COM_HEAD), pshead->length);
				}
				else
					rt_kprintf("send sdk cmd timeout %d\n", err);
				break;
			default:
				break;
			}

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


