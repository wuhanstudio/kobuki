/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-29     hw630       the first version
 */

/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-08     obito0       first version
 */

#include <rtthread.h>
#include <kobuki.h>

void kobuki_play_sound(int argc, char* argv[])
{
    kobuki_setup();

    while (1)
    {
        kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_ON);
        rt_thread_mdelay(1000);
        kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_OFF);
        rt_thread_mdelay(1000);
        kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_RECHARGE);
        rt_thread_mdelay(1000);
        kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_BUTTON);
        rt_thread_mdelay(1000);
        kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_ERROR);
        rt_thread_mdelay(1000);
        kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_CLEANING_START);
        rt_thread_mdelay(1000);
        kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_CLEANING_END);
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(kobuki_play_sound, kobuki play sound demo)
