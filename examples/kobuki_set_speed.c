/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-29     hw630       the first version
 */

#include <rtthread.h>
#include <kobuki.h>

void kobuki_set_speed_demo(int argc, char* argv[])
{
    kobuki_setup();
    kobuki_set_speed(1.0, 0.0);
}
MSH_CMD_EXPORT(kobuki_set_speed_demo, kobuki_set_speed_demo [tv] [rv])
