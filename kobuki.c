/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-28     hw630       the first version
 */

#include <math.h>
#include "kobuki_serial.h"
#include "kobuki_protocol.h"

// b is bias or wheelbase, that indicates the length between the center of the wheels. Fixed at 230 mm.
static float wheelbase = 0.230;

void kobuki_init()
{
    kobuki_serial_init();
}

// cm / s
void kobuki_set_speed(double tv, double rv)
{
    kobuki_base_control_payload_t payload;
    payload.header = KOBUKI_BASE_CONTROL_HEADER;
    payload.length = KOBUKI_BASE_CONTROL_LENGTH;

    // convert to mm;
    tv *= 1000;
    double b2 = wheelbase * 500;

    if (fabs(tv) < 1)
    {
        // Pure rotation
        payload.radius = 1;
        payload.speed =  (int16_t) (rv * b2);
    }
    else if (fabs(rv) < 1e-3 )
    {
        // pure translation"
        payload.speed = (int16_t) tv;
        payload.radius = 0;
    }
    else {
        // translation and rotation
        float r = tv / rv;
        payload.radius = (int16_t) r;
        if (r > 1) {
          payload.speed = (int16_t) (tv * (r + b2)/ r);
        } else if (r < -1) {
          payload.speed = (int16_t) (tv * (r - b2)/ r);
        }
    }
    kobuki_protocol_send_payload( (uint8_t*) (&payload), sizeof(kobuki_base_control_payload_t));
}

// 1 / (f * a), where f is frequency of sound in Hz, and a is 0.00000275
void kobuki_play_sound(uint16_t note, uint8_t duration)
{
    kobuki_sound_payload_t payload;
    payload.header = KOBUKI_SOUND_HEADER;
    payload.length = KOBUKI_SOUND_LENGTH;
    payload.note = note;
    payload.duration = duration;
    kobuki_protocol_send_payload( (uint8_t*) (&payload), sizeof(kobuki_sound_payload_t));
}

void kobuki_play_sound_sequence(uint8_t number)
{
    kobuki_sound_sequence_payload_t payload;
    payload.header = KOBUKI_SOUND_SEQUENCE_HEADER;
    payload.length = KOBUKI_SOUND_SEQUENCE_LENGTH;
    payload.number = number;
    kobuki_protocol_send_payload( (uint8_t*) (&payload), sizeof(kobuki_sound_sequence_payload_t));
}

void kobuki_close()
{
    kobuki_serial_close();
}
