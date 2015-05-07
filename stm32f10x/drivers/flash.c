
#include <rtthread.h>
#include <com_lib.h>

struct flash_device
{
    struct rt_device dev;
    struct rt_device_blk_geometry geometry;
};

struct flash_device fl_dev;

#define SECTOR_SIZE         512
#define BLOCK_COUNT         4096


static rt_err_t rt_flash_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t rt_flash_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_flash_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t rt_flash_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    struct flash_device *blk_dev = (struct flash_device *)dev->user_data;
    switch (cmd)
    {
    case RT_DEVICE_CTRL_BLK_GETGEOME:
        rt_memcpy(args, &blk_dev->geometry, sizeof(struct rt_device_blk_geometry));
        break;
    default:
        break;
    }
    return RT_EOK;
}

static rt_size_t rt_flash_read(rt_device_t dev,
                               rt_off_t    pos,
                               void       *buffer,
                               rt_size_t   size)
{
    rt_err_t err = RT_EOK;
	rt_uint32_t retry = 3;
    struct flash_device *blk_dev = (struct flash_device *)dev->user_data;

	do {
	    err = uart2_send_cmd_status(COM_CMD_READ_MEMORY, buffer, 
					size * SECTOR_SIZE, pos, RT_TICK_PER_SECOND * 2);
	} while ((err != RT_EOK) && retry--);

	if (retry == 0)
		rt_kprintf("read flash failed, %d, %d\n", pos, err);
    
    return SECTOR_SIZE*size;
}

static rt_size_t rt_flash_write(rt_device_t dev,
                                rt_off_t    pos,
                                const void *buffer,
                                rt_size_t   size)
{
    rt_err_t err = RT_EOK;
	rt_uint32_t retry = 3;
    struct flash_device *blk_dev = (struct flash_device *)dev->user_data;

	do {
	    err = uart2_send_cmd_status(COM_CMD_WRITE_MEMORY, (rt_uint8_t *)buffer, 
					size * SECTOR_SIZE, pos, RT_TICK_PER_SECOND * 2);
	} while ((err != RT_EOK) && retry--);

	if (retry == 0)
		rt_kprintf("write flash failed, %d\n", pos);

    return SECTOR_SIZE * size;
}


rt_int32_t rt_hw_flash_init(void)
{
    rt_int32_t err = 0;
    struct flash_device *blk_dev = &fl_dev;

    /* register flash device */
    blk_dev->dev.type  = RT_Device_Class_Block;
    blk_dev->dev.init = rt_flash_init;
    blk_dev->dev.open = rt_flash_open;
    blk_dev->dev.close = rt_flash_close;
    blk_dev->dev.read = rt_flash_read;
    blk_dev->dev.write = rt_flash_write;
    blk_dev->dev.control = rt_flash_control;
    blk_dev->dev.user_data = blk_dev;

    blk_dev->geometry.bytes_per_sector = SECTOR_SIZE;
    blk_dev->geometry.block_size = SECTOR_SIZE;
    blk_dev->geometry.sector_count = BLOCK_COUNT;

    rt_device_register(&blk_dev->dev, "flash", RT_DEVICE_FLAG_RDWR);
    
    return err;
}


