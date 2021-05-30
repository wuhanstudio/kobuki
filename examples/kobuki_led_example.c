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

void kobuki_led1_red(int argc, char* argv[])
{
    struct kobuki robot;
    kobuki_init(&robot);
    kobuki_set_led1_red();
    kobuki_close(&robot);
}
MSH_CMD_EXPORT(kobuki_led1_red, kobuki set led1 red)

void kobuki_led1_green(int argc, char* argv[])
{
    struct kobuki robot;
    kobuki_init(&robot);
    kobuki_set_led1_green();
    kobuki_close(&robot);
}
MSH_CMD_EXPORT(kobuki_led1_green, kobuki set led1 green)

void kobuki_led2_red(int argc, char* argv[])
{
    struct kobuki robot;
    kobuki_init(&robot);
    kobuki_set_led2_red();
    kobuki_close(&robot);
}
MSH_CMD_EXPORT(kobuki_led2_red, kobuki set led2 red)

void kobuki_led2_green(int argc, char* argv[])
{
    struct kobuki robot;
    kobuki_init(&robot);
    kobuki_set_led2_green();
    kobuki_close(&robot);
}
MSH_CMD_EXPORT(kobuki_led2_green, kobuki set led2 green)
