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
#include <stdio.h>
#include <rtthread.h>
#include <kobuki.h>

#define THREAD_PRIORITY      9
#define THREAD_TIMESLICE     5

static char thread1_stack[1024];
static struct rt_thread thread1;

static char thread2_stack[1024];
static struct rt_thread thread2;

static struct kobuki robot;

static void kobuki_thread_entry(void *param)
{
    while(1)
    {
        kobuki_loop(&robot);
    }
}

static void kobuki_get_odometry_entry(void* param)
{
    while(!robot.connected)
    {
        rt_thread_mdelay(50);
    }
    rt_kprintf("\n");

    while(1)
    {
        printf("[kobuki] x: %f \t y: %f \t theta: %f \t v_x: %f \t v_theta: %f\n", robot.x, robot.y, robot.theta, robot.v_x, robot.v_theta);
        rt_thread_mdelay(50);
    }

    // rt_thread_detach(&thread1);
    // kobuki_close(&robot);
}

void kobuki_get_odometry(int argc, char* argv[])
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
                       "kobuki_odom",
                       kobuki_get_odometry_entry,
                       RT_NULL,
                       &thread2_stack[0],
                       sizeof(thread2_stack),
                       THREAD_PRIORITY - 1, THREAD_TIMESLICE);
    rt_thread_startup(&thread2);
}
MSH_CMD_EXPORT(kobuki_get_odometry, kobuki get odometry)
