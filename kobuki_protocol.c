/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-28     hw630       the first version
 */

#include "kobuki_protocol.h"

#define DBG_SECTION_NAME  "kobuki_protocol"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

int kobuki_protocol_send_payload(uint8_t* payload, uint8_t len)
{
    uint8_t ret = 0;
    uint8_t cs = len;
    // Checksum
    for (int i = 0; i < len; i++)
    {
      cs ^= payload[i];
    }
    ret += kobuki_serial_write_char(KOBUKI_BYTE_STREAM_HEADER_0);
    ret += kobuki_serial_write_char(KOBUKI_BYTE_STREAM_HEADER_1);
    ret += kobuki_serial_write_char(len);
    ret += kobuki_serial_write(payload, len);
    ret += kobuki_serial_write_char(cs);

    return ret;
}

void kobuki_play_sound_sequence(uint8_t number)
{
    kobuki_sound_sequence_payload_t payload;
    payload.header = KOBUKI_SOUND_SEQUENCE_HEADER;
    payload.length = KOBUKI_SOUND_SEQUENCE_LENGTH;
    payload.number = number;
    kobuki_protocol_send_payload( (uint8_t*) (&payload), sizeof(kobuki_sound_sequence_payload_t));
}

void kobuki_request_extra(uint16_t flag)
{
    kobuki_request_extra_payload_t payload;
    payload.header = KOBUKI_REQUEST_EXTRA_HEADER;
    payload.length = KOBUKI_REQUEST_EXTRA_LENGTH;
    payload.flags = flag;
    kobuki_protocol_send_payload( (uint8_t*) (&payload), sizeof(kobuki_request_extra_payload_t));
}

void kobuki_set_gpio(uint16_t flag)
{
    kobuki_general_output_payload_t payload;
    payload.header = KOBUKI_GENERAL_OUTPUT_HEADER;
    payload.length = KOBUKI_GENERAL_OUTPUT_LENGTH;
    payload.flags = flag;
    kobuki_protocol_send_payload( (uint8_t*) (&payload), sizeof(kobuki_general_output_payload_t));
}

void kobuki_set_controller_gain_(uint8_t type, uint32_t kp, uint32_t ki, uint32_t kd)
{
    kobuki_set_controller_gain_payload_t payload;
    payload.header = KOBUKI_SET_CONTROLLER_GAIN_HEADER;
    payload.length = KOBUKI_SET_CONTROLLER_GAIN_LENGTH;
    payload.type = type;
    payload.p_gain = kp;
    payload.i_gain = ki;
    payload.d_gain = kd;
    kobuki_protocol_send_payload( (uint8_t*) (&payload), sizeof(kobuki_set_controller_gain_payload_t));
}

void kobuki_get_controller_gain_()
{
    kobuki_get_controller_gain_payload_t payload;
    payload.header = KOBUKI_GET_CONTROLLER_GAIN_HEADER;
    payload.length = KOBUKI_GET_CONTROLLER_GAIN_LENGTH;
    kobuki_protocol_send_payload( (uint8_t*) (&payload), sizeof(kobuki_get_controller_gain_payload_t));
}

int8_t kobuki_protocol_loop(uint8_t* packet, uint8_t max_len)
{
    char c;
    char cs = 0;    // check sum
    char len = 0;

    uint8_t is_packet_ready = 0;
    int tick = kobuki_get_tick();
    while(!is_packet_ready)
    {
        kobuki_serial_read(&c);
        if (c == 0xAA)
        {
            kobuki_serial_read(&c);
            if(c == 0x55)
            {
                is_packet_ready = 1;
            }
        }
        if( (kobuki_get_tick() - tick) > KOBUKI_SERIAL_TIMEOUT)
        {
            return -2;
        }
    }

    kobuki_serial_read(&len);
    if(len > max_len)
    {
        LOG_E("Buffer Overflow");
        return -1;
    }

    cs ^= len;
    int i;
    for (i = 0; i < len; ++i) {
        if(kobuki_serial_read(&packet[i]) == 0)
        {
            // Timeout
            return -2;
        }
        else 
        {
            cs ^= packet[i];
        }
    }
    char cs_;
    kobuki_serial_read(&cs_);
    if (cs ^= cs_)
    {
        LOG_E("Invalid Checksum");
        return 0;
    }

    return len;
}
