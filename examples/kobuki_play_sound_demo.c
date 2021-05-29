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

void kobuki_play_sound_demo(int argc, char* argv[])
{
    kobuki_init();
    if(argc == 3)
    {
        kobuki_play_sound(atof(argv[1]), atof(argv[2]));
    }
    else
    {
        rt_kprintf("kobuki_play_sound_demo [note] [duration]\n");
    }
    kobuki_close();
}
MSH_CMD_EXPORT(kobuki_play_sound_demo, kobuki play sound demo)

void kobuki_play_sound_on_demo()
{
    kobuki_init();
    kobuki_play_sound_on();
    kobuki_close();
}
MSH_CMD_EXPORT(kobuki_play_sound_on_demo, kobuki play sound on demo)

void kobuki_play_sound_off_demo()
{
    kobuki_init();
    kobuki_play_sound_off();
    kobuki_close();
}
MSH_CMD_EXPORT(kobuki_play_sound_off_demo, kobuki play sound off demo)

void kobuki_play_sound_recharge_demo()
{
    kobuki_init();
    kobuki_play_sound_recharge();
    kobuki_close();
}
MSH_CMD_EXPORT(kobuki_play_sound_recharge_demo, kobuki play sound recharge demo)

void kobuki_play_sound_button_demo()
{
    kobuki_init();
    kobuki_play_sound_button();
    kobuki_close();
}
MSH_CMD_EXPORT(kobuki_play_sound_button_demo, kobuki play sound button demo)

void kobuki_play_sound_cleaning_start_demo()
{
    kobuki_init();
    kobuki_play_sound_cleaning_start();
    kobuki_close();
}
MSH_CMD_EXPORT(kobuki_play_sound_cleaning_start_demo, kobuki play sound cleaning start demo)

void kobuki_play_sound_cleaning_end_demo()
{
    kobuki_init();
    kobuki_play_sound_cleaning_end_demo();
    kobuki_close();
}
MSH_CMD_EXPORT(kobuki_play_sound_cleaning_end_demo, kobuki play sound cleaning end demo)
