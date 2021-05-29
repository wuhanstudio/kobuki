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

#ifndef KOBUKI_PACKET_BUFFER
    #define KOBUKI_PACKET_BUFFER 128
#endif

struct kobuki {
    uint16_t timestamp;
    uint8_t bumper;
    uint8_t wheel_drop;
    uint8_t cliff;
    uint16_t left_encoder;
    uint16_t right_encoder;
    uint8_t left_pwm;
    uint8_t right_pwm;
    uint8_t button;
    uint8_t battery;
    uint8_t wheel_overcurrent;
    uint8_t packet[KOBUKI_PACKET_BUFFER];
} kobuki;
typedef struct kobuki* kobuki_t;

void kobuki_init();
void kobuki_loop(kobuki_t robot);
void kobuki_close();

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
void kobuki_get_controller_gain();
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

#endif /* KOBUKI_H_ */
