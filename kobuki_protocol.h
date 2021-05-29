/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-28     hw630       the first version
 */
#ifndef APPLICATIONS_KOBUKI_PROTOCOL_H_
#define APPLICATIONS_KOBUKI_PROTOCOL_H_

#include <stdint.h>
#include "kobuki_serial.h"

#define KOBUKI_BYTE_STREAM_HEADER_0 0xAA
#define KOBUKI_BYTE_STREAM_HEADER_1 0x55

#define KOBUKI_BASE_CONTROL_HEADER 0x01
#define KOBUKI_BASE_CONTROL_LENGTH 0x04

#define KOBUKI_SOUND_HEADER 0x03
#define KOBUKI_SOUND_LENGTH 0x04

#define KOBUKI_SOUND_SEQUENCE_HEADER 0x04
#define KOBUKI_SOUND_SEQUENCE_LENGTH 0x01

#define KOBUKI_REQUEST_EXTRA_HEADER 0x09
#define KOBUKI_REQUEST_EXTRA_LENGTH 0x02

// Bytestream
typedef struct _kobuki_bytestream {
    uint8_t header_0;
    uint8_t header_1;
    uint8_t length;
    uint8_t* payloads;
    uint8_t checksum;
} __attribute__((packed)) kobuki_bytestream_t;

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


// Set Controller Gain


// Get Controller Gain


// Feedback Packets

// Basic Sensor Data

// Docking IR

// Inertial Sensor Data

// Cliff Sensor Data

// Current

// Hardware Version

// Firmware Version

// Raw Data Of 3D Gyro

// General Purpose Input

// Unique Device IDentifier(UDID)

// Controller Info


int kobuki_protocol_send_payload(uint8_t* payload, uint8_t len);

#endif /* APPLICATIONS_KOBUKI_PROTOCOL_H_ */
