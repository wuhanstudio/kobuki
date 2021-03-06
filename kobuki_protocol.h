/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-28     hw630       the first version
 */
#ifndef KOBUKI_PROTOCOL_H_
#define KOBUKI_PROTOCOL_H_

#include <stdint.h>
#include "kobuki_serial.h"

// Command Packets

#define KOBUKI_BYTE_STREAM_HEADER_0         0xAA
#define KOBUKI_BYTE_STREAM_HEADER_1         0x55

#define KOBUKI_BASE_CONTROL_HEADER          0x01
#define KOBUKI_BASE_CONTROL_LENGTH          0x04

#define KOBUKI_SOUND_HEADER                 0x03
#define KOBUKI_SOUND_LENGTH                 0x03

#define KOBUKI_SOUND_SEQUENCE_HEADER        0x04
#define KOBUKI_SOUND_SEQUENCE_LENGTH        0x01

#define KOBUKI_REQUEST_EXTRA_HEADER         0x09
#define KOBUKI_REQUEST_EXTRA_LENGTH         0x02

#define KOBUKI_GENERAL_OUTPUT_HEADER        0x0C
#define KOBUKI_GENERAL_OUTPUT_LENGTH        0x02

#define KOBUKI_SET_CONTROLLER_GAIN_HEADER   0x0D    // Possible typo in documentation
#define KOBUKI_SET_CONTROLLER_GAIN_LENGTH   0x0D

#define KOBUKI_GET_CONTROLLER_GAIN_HEADER   0x0E    // Possible typo in documentation
#define KOBUKI_GET_CONTROLLER_GAIN_LENGTH   0x0E

// Feedback Packets

#define KOBUKI_BASIC_SENSOR_DATA_HEADER     0x01
#define KOBUKI_BASIC_SENSOR_DATA_LENGTH     0x0F

#define KOBUKI_DOCKING_IR_HEADER            0x03
#define KOBUKI_DOCKING_ID_LENGTH            0x03

#define KOBUKI_INERTIAL_SENSOR_DATA_HEADER  0x04
#define KOBUKI_INERTIAL_SENSOR_DATA_LENGTH  0x07

#define KOBUKI_CLIFF_SENSOR_DATA_HEADER     0x05
#define KOBUKI_CLIFFL_SENSOR_DATA_LENGTH    0x06

#define KOBUKI_CURRENT_HEADER               0x06
#define KOBUKI_CURRENT_LENGTH               0x02

#define KOBUKI_HARDWARE_VERSION_HEADER      0x0A
#define KOBUKI_HARDWARE_VERSION_LENGTH      0x04

#define KOBUKI_FIRMWARE_VERSION_HEADER      0x0B
#define KOBUKI_FIRMWARE_VERSION_LENGTH      0x04

#define KOBUKI_3D_GYRO_RAW_DATA_HEADER      0x0D

#define KOBUKI_GENERAL_PURPOSE_INPUT_HEADER 0x10
#define KOBUKI_GENERAL_PURPOSE_INPUT_LENGTH 0x10

#define KOBUKI_UUID_HEADER                  0x13
#define KOBUKI_UUID_LENGTH                  0x0C

#define KOBUKI_CONTROLLER_INFO_HEADER       0x21    // Possible typo in documentation
#define KOBUKI_CONTROLLER_INFO_LENGTH       0x15

#define KOBUKI_LEFT_BUMPER_FLAG             0x04
#define KOBUKI_CENTRAL_BUMPER_FLAG          0x02
#define KOBUKI_RIGHT_BUMPER_FLAG            0x01

#define KOBUKI_LEFT_WHEEL_DROP_FLAG         0x02
#define KOBUKI_RIGHT_WHEEL_DROP_FLAG        0x01

#define KOBUKI_LEFT_CLIFF_FLAG              0x04
#define KOBUKI_CENTRAL_CLIFF_FLAG           0x02
#define KOBUKI_RIGHT_CLIFF_FLAG             0x01

#define KOBUKI_BUTTON_0_FLAG                0x01
#define KOBUKI_BUTTON_1_FLAG                0x02
#define KOBUKI_BUTTON_2_FLAG                0x04

#define KOBUKI_DISCHARGING_FLAG             0x00
#define KOBUKI_DOCKING_CHARGED_FLAG         0x02
#define KOBUKI_DOCKING_CHARGING_FLAG        0x06
#define KOBUKI_ADAPTER_CHARGED_FLAG         0x18
#define KOBUKI_ADAPTER_CHARGING_FLAG        0x22

#define KOBUKI_LEFT_WHEEL_OVERCURRENT_FLAG  0x01
#define KOBUKI_RIGHT_WHEEL_OVERCURRENT_FLAG 0x02

#define KOBUKI_DOCKING_IR_NEAR_LEFT_FLAG    0x01
#define KOBUKI_DOCKING_IR_NEAR_CENTER_FLAG  0x02
#define KOBUKI_DOCKING_IR_NEAR_RIGHT_FLAG   0x04

#define KOBUKI_DOCKING_IR_FAR_LEFT_FLAG     0x08
#define KOBUKI_DOCKING_IR_FAR_CENTER_FLAG   0x10
#define KOBUKI_DOCKING_IR_FAR_RIGHT_FLAG    0x20

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

/* Request extra */
typedef enum
{
    KOBUKI_REQUEST_HARDWARE_VERSION_ID = 0x01,
    KOBUKI_REQUEST_FIRMWARE_VERSION_ID = 0x02,
    KOBUKI_REQUEST_UUID = 0x08,
} kobuki_request_id;

/* Request extra */
typedef enum
{
    KOBUKI_GPIO0_HIGH_FLAG = 0x0001,
    KOBUKI_GPIO1_HIGH_FLAG = 0x0002,
    KOBUKI_GPIO2_HIGH_FLAG = 0x0004,
    KOBUKI_GPIO3_HIGH_FLAG = 0x0008,
    KOBUKI_ENABLE_3V3_FLAG = 0X010,
    KOBUKI_ENABLE_5V_FLAG = 0x0020,
    KOBUKI_ENABLE_12V_5A_FLAG = 0x0040,
    KOBUKI_ENABLE_12V_1_5_A_FLAG = 0x0080,
    KOBUKI_LED1_RED_FLAG = 0x0100,
    KOBUKI_LED1_GREEN_FLAG = 0x0200,
    KOBUKI_LED2_RED_FLAG = 0x0400,
    KOBUKI_LED2_GREEN_FLAG = 0x0800
} kobuki_gpio_id;


// Bytestream
typedef struct _kobuki_bytestream {
    uint8_t header_0;
    uint8_t header_1;
    uint8_t length;
    uint8_t* payloads;
    uint8_t checksum;
} __attribute__((packed)) kobuki_bytestream_t;

/* ---------------- */
/* Command  Packets */
/* ---------------- */

// Base Control
typedef struct _kobuki_base_control_payload {
    uint8_t header;
    uint8_t length;
    uint16_t speed;
    uint16_t radius;
} __attribute__((packed)) kobuki_base_control_payload_t;

// Sound
typedef struct _kobuki_sound_payload {
    uint8_t header;
    uint8_t length;
    uint16_t note;
    uint8_t duration;
} __attribute__((packed)) kobuki_sound_payload_t;

// Sound Sequence
typedef struct _kobuki_sound_sequence_payload {
    uint8_t header;
    uint8_t length;
    uint8_t number;
} __attribute__((packed)) kobuki_sound_sequence_payload_t;

// Request Extra
typedef struct _kobuki_request_extra_payload {
    uint8_t header;
    uint8_t length;
    uint16_t flags;
} __attribute__((packed)) kobuki_request_extra_payload_t;

// General Purpose Output
typedef struct _kobuki_general_output_payload {
    uint8_t header;
    uint8_t length;
    uint16_t flags;
} __attribute__((packed)) kobuki_general_output_payload_t;

// Set Controller Gain
typedef struct _kobuki_set_controller_gain_payload {
    uint8_t header;
    uint8_t length;
    uint8_t type;
    uint32_t p_gain;
    uint32_t i_gain;
    uint32_t d_gain;
} __attribute__((packed)) kobuki_set_controller_gain_payload_t;

// Get Controller Gain
typedef struct _kobuki_get_controller_gain_payload {
    uint8_t header;
    uint8_t length;
    uint8_t unused;
} __attribute__((packed)) kobuki_get_controller_gain_payload_t;

/* ---------------- */
/* Feedback Packets */
/* ---------------- */

// Basic Sensor Data
typedef struct _kobuki_basic_sensor_data_payload {
    uint8_t header;
    uint8_t length;
    uint16_t timestamp;
    uint8_t bumper_flag;
    uint8_t wheel_drop_flag;
    uint8_t cliff_flag;
    uint16_t left_encoder;
    uint16_t right_encoder;
    int8_t left_pwm;
    int8_t right_pwm;
    uint8_t button_flag;
    uint8_t charger_flag;
    uint8_t battery;
    uint8_t overcurrent_flag;
} __attribute__((packed)) kobuki_basic_sensor_data_payload_t;

// Docking IR
typedef struct _kobuki_docking_ir_payload {
    uint8_t header;
    uint8_t length;
    uint8_t right_signal;
    uint8_t central_signal;
    uint8_t left_signal;
} __attribute__((packed)) kobuki_docking_ir_payload_t;

// Inertial Sensor Data
typedef struct _kobuki_inertial_sensor_data_payload {
    uint8_t header;
    uint8_t length;
    int16_t angle;
    int16_t angle_rate;
    uint8_t unused_0;
    uint8_t unused_1;
    uint8_t unused_2;
} __attribute__((packed)) kobuki_inertial_sensor_data_payload_t;

// Cliff Sensor Data
typedef struct _kobuki_cliff_sensor_data_payload {
    uint8_t header;
    uint8_t length;
    uint8_t right_cliff_sensor;
    uint8_t central_cliff_sensor;
    uint8_t left_cliff_sensor;
} __attribute__((packed)) kobuki_cliff_sensor_data_payload_t;

// Current
typedef struct _kobuki_current_payload {
    uint8_t header;
    uint8_t length;
    uint8_t left_motor;
    uint8_t right_motor;
} __attribute__((packed)) kobuki_current_payload_t;

// Hardware Version
typedef struct _kobuki_hardware_version_payload {
    uint8_t header;
    uint8_t length;
    uint8_t pathch;
    uint8_t minor;
    uint8_t major;
    uint8_t unused;
} __attribute__((packed)) kobuki_hardware_version_payload_t;

// Firmware Version
typedef struct _kobuki_firmware_version_payload {
    uint8_t header;
    uint8_t length;
    uint8_t pathch;
    uint8_t minor;
    uint8_t major;
    uint8_t unused;
} __attribute__((packed)) kobuki_firmware_version_payload_t;

// Raw Data Of 3D Gyro
// We don't need raw data because angle and angular velocity can be retrieved from Inertial Sensor Data
//typedef struct _kobuki_3d_gyro_raw_data {
//    uint8_t x;
//    uint8_t y;
//    uint8_t z;
//} __attribute__((packed)) kobuki_3d_gyro_raw_data_t;
//
//typedef struct _kobuki_3d_gyro_raw_data_payload {
//    uint8_t header;
//    uint8_t length;
//    uint8_t frame_id;
//    uint8_t followed_data_length;
//    kobuki_3d_gyro_raw_data_t raw_data[3];
//} __attribute__((packed)) kobuki_3d_gyro_raw_data_payload_t;

// General Purpose Input
typedef struct _kobuki_general_purpose_input_payload {
    uint8_t header;
    uint8_t length;
    uint16_t digital_input;
    uint16_t analog_input_ch_0;
    uint16_t analog_input_ch_1;
    uint16_t analog_input_ch_2;
    uint16_t analog_input_ch_3;
    uint16_t unused_0;
    uint16_t unused_1;
    uint16_t unused_2;
} __attribute__((packed)) kobuki_general_purpose_input_payload_t;

// Unique Device IDentifier(UDID)
typedef struct _kobuki_uuid_payload {
    uint8_t header;
    uint8_t length;
    uint32_t uuid_0;
    uint32_t uuid_1;
    uint32_t uuid_2;
} __attribute__((packed)) kobuki_uuid_payload_t;

// Controller Info
typedef struct _kobuki_controller_info_payload {
    uint8_t header;
    uint8_t length;
    uint8_t type;
    uint32_t p_gain;
    uint32_t i_gain;
    uint32_t d_gain;
} __attribute__((packed)) kobuki_controller_info_payload_t;


int kobuki_protocol_send_payload(uint8_t* payload, uint8_t len);

void kobuki_play_sound_sequence(uint8_t number);
void kobuki_set_gpio(uint16_t flag);
void kobuki_request_extra(uint16_t flag);
void kobuki_set_controller_gain_(uint8_t type, uint32_t kp, uint32_t ki, uint32_t kd);
void kobuki_get_controller_gain_(void);

int8_t kobuki_protocol_loop(uint8_t* packet, uint8_t len);

#endif /* KOBUKI_PROTOCOL_H_ */
