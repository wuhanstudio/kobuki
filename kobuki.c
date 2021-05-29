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
#include "kobuki.h"
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

void kobuki_play_sound_on()
{
    kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_ON);
}

void kobuki_play_sound_off()
{
    kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_OFF);
}

void kobuki_play_sound_recharge()
{
    kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_RECHARGE);
}

void kobuki_play_sound_button()
{
    kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_BUTTON);
}

void kobuki_play_sound_error()
{
    kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_ERROR);
}

void kobuki_play_sound_cleaning_start()
{
    kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_CLEANING_START);
}

void kobuki_play_sound_cleaning_end()
{
    kobuki_play_sound_sequence(KOBUKI_SOUND_SEQUENCE_CLEANING_END);
}

void kobuki_enalbe_3v3()
{
    kobuki_set_gpio(KOBUKI_ENABLE_3V3_FLAG);
}

void kobuki_enable_5v()
{
    kobuki_set_gpio(KOBUKI_ENABLE_5V_FLAG);
}

void kobuki_enable_12v_5a()
{
    kobuki_set_gpio(KOBUKI_ENABLE_12V_5A_FLAG);
}

void kobuki_enable_12v_1_5a()
{
    kobuki_set_gpio(KOBUKI_ENABLE_12V_1_5_A_FLAG);
}

void kobuki_set_led1_red()
{
    kobuki_set_gpio(KOBUKI_LED1_RED_FLAG);
}

void kobuki_set_led1_green()
{
    kobuki_set_gpio(KOBUKI_LED1_GREEN_FLAG);
}

void kobuki_set_led2_red()
{
    kobuki_set_gpio(KOBUKI_LED2_RED_FLAG);
}

void kobuki_set_led2_green()
{
    kobuki_set_gpio(KOBUKI_LED2_GREEN_FLAG);
}

void kobuki_set_controller_gain(uint32_t kp, uint32_t ki, uint32_t kd)
{
    kobuki_set_controller_gain_(1, kp, ki, kd);
}

void kobuki_get_controller_gain()
{
    kobuki_get_controller_gain_();
}


void kobuki_reset_controller_gain()
{
    kobuki_set_controller_gain_(0, 0, 0, 0);
}

void kobuki_get_hardware_version()
{
    kobuki_request_extra(KOBUKI_REQUEST_HARDWARE_VERSION_ID);
}

void kobuki_get_firmware_version()
{
    kobuki_request_extra(KOBUKI_REQUEST_FIRMWARE_VERSION_ID);
}

void kobuki_get_uuid()
{
    kobuki_request_extra(KOBUKI_REQUEST_UUID);
}

static void kobuki_parse_subpaylod(kobuki_t robot, uint8_t* subpayload, uint8_t len)
{
    switch(subpayload[0])
    {
    case KOBUKI_BASIC_SENSOR_DATA_HEADER:
        rt_kprintf("Received basic sensor \n");
        break;
    case KOBUKI_DOCKING_IR_HEADER:
        rt_kprintf("Received docking id \n");
        break;
    case KOBUKI_INERTIAL_SENSOR_DATA_HEADER:
        rt_kprintf("Received inertial sensor \n");
        break;
    case KOBUKI_CLIFF_SENSOR_DATA_HEADER:
        rt_kprintf("Received cliff sensor \n");
        break;
    case KOBUKI_CURRENT_HEADER:
        rt_kprintf("Received current \n");
        break;
    case KOBUKI_HARDWARE_VERSION_HEADER:
        rt_kprintf("Received hardware version \n");
        break;
    case KOBUKI_FIRMWARE_VERSION_HEADER:
        rt_kprintf("Received firmware version \n");
        break;
    case KOBUKI_3D_GYRO_RAW_DATA_HEADER:
        rt_kprintf("Received 3D gyro \n");
        break;
    case KOBUKI_GENERAL_PURPOSE_INPUT_HEADER:
        rt_kprintf("Received general output\n");
        break;
    case KOBUKI_UUID_HEADER:
        rt_kprintf("Received uuid \n");
        break;
    case KOBUKI_CONTROLLER_INFO_HEADER:
        rt_kprintf("Received controller sensor \n");
        break;
    default:
        rt_kprintf("unkown subpayload \n");
        break;
    }
}

void kobuki_loop(kobuki_t robot)
{
    int packet_len = kobuki_protocol_loop(robot->packet, KOBUKI_PACKET_BUFFER);
    if (packet_len < 0)
    {
        // buffer overflow
        return;
    }
    else if (packet_len == 0)
    {
        // invalid checksum
        return;
    }
    else
    {
        // received valid payloads
        rt_kprintf("received packet %d\n", packet_len);
        int i;
        for (i = 0; i < packet_len; i += robot->packet[i+1] + 2) {
            rt_kprintf("[%d] subpayload len %d\n", i, robot->packet[i+1]);
            kobuki_parse_subpaylod(robot, robot->packet + i, robot->packet[i+1]);
        }
    }
}

void kobuki_close()
{
    kobuki_serial_close();
}
