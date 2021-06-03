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

#include <rtthread.h>

#ifndef KOBUKI_PACKET_BUFFER
    #define KOBUKI_PACKET_BUFFER 128
#endif

#ifndef KOBUKI_SYNC_TIMEOUT
    #define KOBUKI_SYNC_TIMEOUT 100
#endif

#define KOBUKI_RECV_HARDWARE_EVENT          (1 << 1)
#define KOBUKI_RECV_FIRMWARE_EVENT          (1 << 2)
#define KOBUKI_RECV_UUID_EVENT              (1 << 3)
#define KOBUKI_RECV_CONTROLLER_INFO_EVENT   (1 << 4)

struct kobuki {
    // triggered when new data comes in
    struct rt_event event;

    // odometry
    float x;
    float y;
    float theta;
    float v_x;
    float v_theta;

    // connection status
    uint8_t  connected;
    uint32_t last_sync_tick;
    uint16_t timestamp;

    // bumper flag
    uint8_t  bumper;
    uint8_t  left_bumper;
    uint8_t  central_bumper;
    uint8_t  right_bumper;

    // wheel drop flag
    uint8_t  wheel_drop;
    uint8_t  left_wheel_drop;
    uint8_t  right_wheel_drop;

    // cliff flag
    uint8_t  cliff;
    uint8_t  left_cliff;
    uint8_t  central_cliff;
    uint8_t  right_cliff;

    // raw cliff sensor data
    uint16_t left_cliff_adc;
    uint16_t central_cliff_adc;
    uint16_t right_cliff_adc;

    // encoder data
    uint16_t left_encoder;
    uint16_t right_encoder;

    // motor pwm
    uint8_t  left_pwm;
    uint8_t  right_pwm;

    // button status
    uint8_t  button;
    uint8_t  button_0;
    uint8_t  button_1;
    uint8_t  button_2;

    // charger status
    uint8_t  charger;
    uint8_t  charger_discharging;
    uint8_t  charger_docking_charged;
    uint8_t  charger_docking_charging;
    uint8_t  charger_adapter_charged;
    uint8_t  charger_adapter_charging;

    // battery level
    uint8_t  battery;

    // wheel overcurrent flag
    uint8_t  wheel_overcurrent;
    uint8_t  left_wheel_overcurrent;
    uint8_t  right_wheel_overcurrent;

    // docking id flag
    uint8_t docking_ir_left;
    uint8_t docking_ir_center;
    uint8_t docking_ir_right;

    uint8_t docking_ir_near_left;
    uint8_t docking_ir_near_center;
    uint8_t docking_ir_near_right;

    uint8_t docking_ir_far_left;
    uint8_t docking_ir_far_center;
    uint8_t docking_ir_far_right;

    // inertial data
    uint16_t inertial_angle;
    uint16_t inertial_angle_rate;

    // motor current
    uint16_t left_motor_current;
    uint16_t right_motor_current;

    // version
    uint8_t harware_version_major;
    uint8_t harware_version_minor;
    uint8_t harware_version_patch;

    uint8_t firmware_version_major;
    uint8_t firmware_version_minor;
    uint8_t firmware_version_patch;

    // uuid
    uint32_t uuid_0;
    uint32_t uuid_1;
    uint32_t uuid_2;

    // digital input
    uint16_t digital_input;
    uint8_t  digital_input_0;
    uint8_t  digital_input_1;
    uint8_t  digital_input_2;
    uint8_t  digital_input_3;

    // analog input
    uint16_t analog_input_0;
    uint16_t analog_input_1;
    uint16_t analog_input_2;
    uint16_t analog_input_3;

    // PID controller
    uint32_t kp;
    uint32_t ki;
    uint32_t kd;

    uint8_t  packet[KOBUKI_PACKET_BUFFER];
};
typedef struct kobuki* kobuki_t;

int  kobuki_init(kobuki_t robot);
void kobuki_loop(kobuki_t robot);
void kobuki_close(kobuki_t robot);

void kobuki_set_speed(double tv, double rv);

void kobuki_enalbe_3v3(void);
void kobuki_enable_5v(void);
void kobuki_enable_12v_5a(void);
void kobuki_enable_12v_1_5a(void);

void kobuki_set_led1_red(void);
void kobuki_set_led1_green(void);
void kobuki_set_led2_red(void);
void kobuki_set_led2_green(void);

void kobuki_set_controller_gain(uint32_t kp, uint32_t ki, uint32_t kd);
void kobuki_get_controller_gain(void);
void kobuki_reset_controller_gain(void);

void kobuki_get_hardware_version(void);
void kobuki_get_firmware_version(void);
void kobuki_get_uuid(void);

void kobuki_play_sound(uint16_t note, uint8_t duration);
void kobuki_play_sound_on(void);
void kobuki_play_sound_off(void);
void kobuki_play_sound_recharge(void);
void kobuki_play_sound_button(void);
void kobuki_play_sound_error(void);
void kobuki_play_sound_cleaning_start(void);
void kobuki_play_sound_cleaning_end(void);

#endif /* KOBUKI_H_ */
