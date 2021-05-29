/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-28     hw630       the first version
 */

#include "kobuki_serial.h"

#define DBG_SECTION_NAME  "kobuki_serial"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

static struct rt_semaphore rx_sem;
static rt_device_t kobuki_serial;

static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);

    return RT_EOK;
}

int kobuki_serial_setup()
{
    kobuki_serial = rt_device_find(KOBUKI_SERIAL_NAME);
    if (!kobuki_serial)
    {
        LOG_E("Failed to open device %s", KOBUKI_SERIAL_NAME);
        return -1;
    }
    rt_sem_init(&rx_sem, "kobuki_rx_sem", 0, RT_IPC_FLAG_FIFO);
    rt_device_open(kobuki_serial, RT_DEVICE_FLAG_INT_RX);
    rt_device_set_rx_indicate(kobuki_serial, uart_input);

    return 0;
}

char kobuki_serial_read()
{
    char ch;
    while (rt_device_read(kobuki_serial, -1, &ch, 1) != 1)
    {
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
    }
    return ch;
}

int kobuki_serial_write_char(uint8_t ch)
{
    return rt_device_write(kobuki_serial, 0, &ch, 1);
}

int  kobuki_serial_write(uint8_t* dat, int len)
{
    return rt_device_write(kobuki_serial, 0, dat, len);
}
