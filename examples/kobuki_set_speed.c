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

void kobuki_set_speed_demo(int argc, char* argv[])
{
    char *eptr;

    kobuki_setup();
    if(argc == 2)
    {
        kobuki_set_speed(strtod(argv[1], &eptr), strtod(argv[2], &eptr));
    }
    else
    {
        rt_kprintf("kobuki_set_speed_demo [tv] [rv]\n");
    }
}
MSH_CMD_EXPORT(kobuki_set_speed_demo, kobuki_set_speed_demo [tv] [rv])
