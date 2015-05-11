
#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <stm32f10x.h>
#include "vcom_proc.h"

#define VCOM_BUFSZ 2048
static rt_uint32_t vcom_buffer[VCOM_BUFSZ/4];
static struct rt_semaphore vcom_sem;
static rt_device_t vcom_dev;

#define BUFSZ 2048
rt_uint32_t uart1_buffer[BUFSZ / 4];
static struct rt_semaphore uart1_sem;
static rt_device_t uart1_dev;

static int led_status = 0;
static struct rt_semaphore led_sem;
static void rt_led_on(void);
static void rt_led_off(void);

extern void
rt_vcom_cfg_sethook(void (*hook)(struct serial_configure *cfg));

rt_err_t uart1_rx_ind(rt_device_t dev, rt_size_t size)
{
	rt_sem_release(&uart1_sem);
	if (!led_status)
	{
		led_status = 1;
		rt_sem_release(&led_sem);
	}

	return RT_EOK;
}

void uart1_thread_entry(void* parameter)
{
	rt_err_t err;
	rt_size_t size = 0;
	char *buf;
	int count = 4;
	int offset = 0;
	
	buf = (char *)uart1_buffer;

	rt_device_set_rx_indicate(uart1_dev, uart1_rx_ind);

	while(1)
	{
		err = rt_sem_take(&uart1_sem, RT_WAITING_FOREVER);
		if (err != RT_EOK)
		{
			rt_kprintf("wait sem uart2 error\n");
			break;
		}

		size = rt_device_read(uart1_dev, 0, buf, BUFSZ);
		if (size > 0)
		{
			rt_device_write(vcom_dev, 0, buf, size);
		}

	}

	rt_device_set_rx_indicate(uart1_dev, RT_NULL);
	rt_device_close(uart1_dev);
	rt_sem_detach(&uart1_sem);
}

void uart1_config(struct serial_configure *cfg)
{
	rt_device_control(uart1_dev, RT_DEVICE_CTRL_CONFIG, cfg);
}

rt_err_t vcom_rx_ind(rt_device_t dev, rt_size_t size)
{
	rt_sem_release(&vcom_sem);
	if (!led_status)
	{
		led_status = 1;
		rt_sem_release(&led_sem);
	}

	return RT_EOK;
}

void vcom_thread_entry(void* parameter)
{
	rt_err_t err;
	rt_size_t size = 0;
	rt_uint8_t *buf;
	int count = 4;
	int offset = 0;

	buf = (rt_uint8_t *)vcom_buffer;

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
			rt_device_write(uart1_dev, 0, buf, size);
		}

	}

	rt_device_set_rx_indicate(vcom_dev, RT_NULL);
	rt_device_close(vcom_dev);
	rt_sem_detach(&vcom_sem);
}

// GPIO define
#define LED_GPIO      GPIOB
#define LED_GPIO_PIN  GPIO_Pin_9
#define RCC_APB2Periph_GPIO_LED      RCC_APB2Periph_GPIOB

static void rt_led_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_LED, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin   = LED_GPIO_PIN;
    GPIO_Init(LED_GPIO, &GPIO_InitStructure);
    GPIO_SetBits(LED_GPIO, LED_GPIO_PIN);
}

static void rt_led_on(void)
{
	GPIO_SetBits(LED_GPIO, LED_GPIO_PIN);
}

static void rt_led_off(void)
{
	GPIO_ResetBits(LED_GPIO, LED_GPIO_PIN);
}

void led_thread_entry(void* parameter)
{
	rt_err_t err;
	int count = 0;

	while(1)
	{
		led_status = 0;
		rt_led_on();
		err = rt_sem_take(&led_sem, RT_WAITING_FOREVER);
		if (err != RT_EOK)
		{
			rt_kprintf("wait sem led error\n");
			break;
		}

		for (count = 0; count < 20; count++)
		{
			rt_led_on();
			rt_thread_delay(2);
			rt_led_off();
			rt_thread_delay(3);
		}
	}

	rt_sem_detach(&led_sem);
}

void vcom_init(void)
{
	rt_err_t err;
	struct serial_configure config;
	rt_thread_t uart1_thread;
	rt_thread_t vcom_thread;
	rt_thread_t led_thread;

	rt_led_gpio_init();

	rt_sem_init(&led_sem, "led", 0, RT_IPC_FLAG_FIFO);

	rt_sem_init(&uart1_sem, "uart1", 0, RT_IPC_FLAG_FIFO);

	uart1_dev = rt_device_find("uart1");

	if (!uart1_dev)
	{
		rt_kprintf("open device uart1 failed\n");
	}

	err = rt_device_open(uart1_dev, RT_DEVICE_OFLAG_RDWR);
	if (err != RT_EOK)
	{
		rt_kprintf("open uart1 failed\n");
		return;
	}

	config.baud_rate = 115200;
    config.bit_order = BIT_ORDER_LSB;
    config.data_bits = DATA_BITS_8;
    config.parity    = PARITY_NONE;
    config.stop_bits = STOP_BITS_1;
    config.invert    = NRZ_NORMAL;

	rt_device_control(uart1_dev, RT_DEVICE_CTRL_CONFIG, &config);

	rt_vcom_cfg_sethook(uart1_config);

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

	uart1_thread = rt_thread_create("uart1",
								uart1_thread_entry, RT_NULL,
								512, 15, 20);

	if (uart1_thread != RT_NULL)
		rt_thread_startup(uart1_thread);
	

	vcom_thread = rt_thread_create("vcom",
								vcom_thread_entry, RT_NULL,
								512, 16, 20);

	if (vcom_thread != RT_NULL)
		rt_thread_startup(vcom_thread);

	led_thread = rt_thread_create("led",
								led_thread_entry, RT_NULL,
								512, 17, 20);

	if (led_thread != RT_NULL)
		rt_thread_startup(led_thread);
}


