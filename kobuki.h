/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-28     hw630       the first version
 */
#ifndef APPLICATIONS_KOBUKI_H_
#define APPLICATIONS_KOBUKI_H_

/* Sound Sequence */
typedef enum
{
    KOBUKI_SOUND_SEQUENCE_ON,
    KOBUKI_SOUND_SEQUENCE_OFF,
    KOBUKI_SOUND_SEQUENCE_RECHARGE,
    KOBUKI_SOUND_SEQUENCE_BUTTON,
    KOBUKI_SOUND_SEQUENCE_ERROR,
    KOBUKI_SOUND_SEQUENCE_CLEANING_START,
    KOBUKI_SOUND_SEQUENCE_CLEANING_END
} kobuki_sound_sequence;

void kobuki_setup();
void kobuki_set_speed(double tv, double rv);
void kobuki_play_sound_sequence(uint8_t number);

#endif /* APPLICATIONS_KOBUKI_H_ */
