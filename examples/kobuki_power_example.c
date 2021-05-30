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

void kobuki_enable_3v3_power()
{
    struct kobuki robot;
    kobuki_init(&robot);
    kobuki_enalbe_3v3();
    kobuki_close(&robot);
}
MSH_CMD_EXPORT(kobuki_enable_3v3_power, kobuki enable 3v3 power)

void kobuki_enable_5v_power()
{
    struct kobuki robot;
    kobuki_init(&robot);
    kobuki_enable_5v();
    kobuki_close(&robot);
}
MSH_CMD_EXPORT(kobuki_enable_5v_power, kobuki enable 5v power)

void kobuki_enable_12v_5a_power()
{
    struct kobuki robot;
    kobuki_init(&robot);
    kobuki_enable_12v_5a();
    kobuki_close(&robot);
}
MSH_CMD_EXPORT(kobuki_enable_12v_5a_power, kobuki enable 12v 5a power)

void kobuki_enable_12v_1_5a_power()
{
    struct kobuki robot;
    kobuki_init(&robot);
    kobuki_enable_12v_1_5a();
    kobuki_close(&robot);
}
MSH_CMD_EXPORT(kobuki_enable_12v_1_5a_power, kobuki_enable 12v 1.5a power)
