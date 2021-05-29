/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-28     hw630       the first version
 */
#ifndef KOBUKI_H_
#define KOBUKI_H_

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

void kobuki_init();
void kobuki_set_speed(double tv, double rv);
void kobuki_play_sound(uint16_t note, uint8_t duration);
void kobuki_play_sound_sequence(uint8_t number);
void kobuki_close();

#endif /* KOBUKI_H_ */
