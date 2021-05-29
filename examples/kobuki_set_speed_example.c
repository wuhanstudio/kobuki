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

void kobuki_set_speed_example(int argc, char* argv[])
{
    kobuki_init();
    if(argc == 3)
    {
        kobuki_set_speed(atof(argv[1]), atof(argv[2]));
    }
    else
    {
        rt_kprintf("kobuki_set_speed_example [tv] [rv]\n");
    }
    kobuki_close();
}
MSH_CMD_EXPORT(kobuki_set_speed_example, kobuki_set_speed_example [tv] [rv])
