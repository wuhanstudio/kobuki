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

#include <stdint.h>

void kobuki_init();

void kobuki_set_speed(double tv, double rv);

void kobuki_enalbe_3v3();
void kobuki_enable_5v();
void kobuki_enable_12v_5a();
void kobuki_enable_12v_1_5a();
void kobuki_set_led1_red();
void kobuki_set_led1_green();
void kobuki_set_led2_red();
void kobuki_set_led2_green();

void kobuki_set_controller_gain(uint32_t kp, uint32_t ki, uint32_t kd);
void kobuki_reset_controller_gain();

void kobuki_get_hardware_version();
void kobuki_get_firmware_version();
void kobuki_get_uuid();

void kobuki_play_sound(uint16_t note, uint8_t duration);
void kobuki_play_sound_on();
void kobuki_play_sound_off();
void kobuki_play_sound_recharge();
void kobuki_play_sound_button();
void kobuki_play_sound_error();
void kobuki_play_sound_cleaning_start();
void kobuki_play_sound_cleaning_end();

void kobuki_close();

#endif /* KOBUKI_H_ */
