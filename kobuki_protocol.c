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

int kobuki_protocol_send_payload(uint8_t* payload, uint8_t len)
{
    uint8_t ret = 0;
    uint8_t cs = len;
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
