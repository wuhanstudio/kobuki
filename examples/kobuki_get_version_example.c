/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-29     hw630       the first version
 */

#include <stdlib.h>
#include <rtthread.h>
#include <kobuki.h>

#define THREAD_PRIORITY      9
#define THREAD_TIMESLICE     5

static char thread1_stack[1024];
static struct rt_thread thread1;

static char thread2_stack[1024];
static struct rt_thread thread2;

struct kobuki robot;

static void kobuki_thread_entry(void *param)
{
    while(1)
    {
        kobuki_loop(&robot);
    }
}

static void kobuki_get_version_entry(void* param)
{
    rt_uint32_t e;

    while(!robot.connected)
    {
        rt_thread_mdelay(50);
    }
    rt_kprintf("\n");

    kobuki_get_hardware_version();
    while (!rt_event_recv(&(robot.event), KOBUKI_RECV_HARDWARE_EVENT,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &e) == RT_EOK);
    rt_kprintf("Hardware Version: %d.%d.%d\n", robot.harware_version_major, robot.harware_version_minor, robot.harware_version_patch);

    kobuki_get_firmware_version();
    while (!rt_event_recv(&(robot.event), KOBUKI_RECV_FIRMWARE_EVENT,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &e) == RT_EOK);
    rt_kprintf("Firmware Version: %d.%d.%d\n", robot.firmware_version_major, robot.firmware_version_minor, robot.firmware_version_patch);

    kobuki_get_uuid();
    while (!rt_event_recv(&(robot.event), KOBUKI_RECV_UUID_EVENT,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, &e) == RT_EOK);
    rt_kprintf("Kobuki UUID: %X-%X-%X\n", robot.uuid_0, robot.uuid_1, robot.uuid_2);

    rt_thread_detach(&thread1);
    kobuki_close(&robot);
}

void kobuki_get_version(int argc, char* argv[])
{
    if (kobuki_init(&robot) < 0)
    {
        rt_kprintf("Failed to init kobuki\n");
        return;
    }

    rt_thread_init(&thread1,
                   "kobuki",
                   kobuki_thread_entry,
                   RT_NULL,
                   &thread1_stack[0],
                   sizeof(thread1_stack),
                   THREAD_PRIORITY - 1, THREAD_TIMESLICE);
    rt_thread_startup(&thread1);

    rt_thread_init(&thread2,
                       "kobuki_version",
                       kobuki_get_version_entry,
                       RT_NULL,
                       &thread2_stack[0],
                       sizeof(thread2_stack),
                       THREAD_PRIORITY - 1, THREAD_TIMESLICE);
    rt_thread_startup(&thread2);
}
MSH_CMD_EXPORT(kobuki_get_version, kobuki get version)
