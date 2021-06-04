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

#define DBG_SECTION_NAME  "kobuki"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

 #define M_PI 3.14159265358979323846264338327950288

// b is bias or wheelbase, that indicates the length between the center of the wheels. Fixed at 230 mm.
static float wheelbase = 0.230;

static float left_ticks_per_m  = 1./11724.41658029856624751591;
static float right_ticks_per_m = 1./11724.41658029856624751591;

int kobuki_init(kobuki_t robot)
{
    rt_err_t result;
    result = rt_event_init(&(robot->event), "kobuki_event", RT_IPC_FLAG_FIFO);

    robot->connected = 0;
    robot->last_sync_tick = 0;

    robot->x = 0;
    robot->y = 0;
    robot->theta = 0;

    if (result != RT_EOK)
    {
        return -2;
    }

    return kobuki_serial_init();
}

// translation velocity and rotation velocity (cm / s)
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
        // pure rotation
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

static void kobuki_update_odometry(kobuki_t robot, uint16_t left_encoder, uint16_t right_encoder, uint16_t elapsed_time_ms)
{
    double dl= left_ticks_per_m * (left_encoder- robot->left_encoder);
    double dr= right_ticks_per_m * (right_encoder - robot->right_encoder);
    double dx = 0, dy = 0;
    double dtheta = (dr-dl) / wheelbase;

    if (dl != dr)
    {
      double R = 0.5 * (dr + dl) / dtheta;
      dx = R * sin(dtheta);
      dy = R * (1-cos(dtheta));
    } else
    {
      dx = dr;
    }
    double s = sin(robot->theta);
    double c = cos(robot->theta);
    double diff_x = c * dx - s * dy;
    //ROS_INFO_STREAM("Elapsed " << elapsed_time_ms << " distance " << diff_x);
    robot->v_x = diff_x / ((elapsed_time_ms) / 1000.0);
    robot->x += diff_x;
    robot->y += s * dx + c * dy;
    robot->theta += dtheta;
    robot->theta = fmod(robot->theta + 4 * M_PI, 2 * M_PI);
    if (robot->theta > M_PI)
    {
        robot->theta -= 2*M_PI;
    }
}

static void kobuki_parse_subpaylod(kobuki_t robot, uint8_t* subpayload, uint8_t len)
{
    int32_t elapsed_time = 0;
    switch(subpayload[0])
    {
    case KOBUKI_BASIC_SENSOR_DATA_HEADER:
        if(robot->last_sync_tick > 0)
        {
            if( ((kobuki_basic_sensor_data_payload_t*)subpayload)->timestamp > robot->timestamp ) {
                elapsed_time = ((kobuki_basic_sensor_data_payload_t*)subpayload)->timestamp - robot->timestamp;
            }
            else
            {
                elapsed_time = 65535 - robot->timestamp + ((kobuki_basic_sensor_data_payload_t*)subpayload)->timestamp;
            }
            kobuki_update_odometry(robot,
                    ((kobuki_basic_sensor_data_payload_t*)subpayload)->left_encoder,
                    ((kobuki_basic_sensor_data_payload_t*)subpayload)->right_encoder,
                    elapsed_time);
        }

        robot->timestamp        = ((kobuki_basic_sensor_data_payload_t*)subpayload)->timestamp;

        robot->bumper           = ((kobuki_basic_sensor_data_payload_t*)subpayload)->bumper_flag;
        robot->left_bumper      = ((kobuki_basic_sensor_data_payload_t*)subpayload)->bumper_flag & KOBUKI_LEFT_BUMPER_FLAG    ? 1 : 0;
        robot->central_bumper   = ((kobuki_basic_sensor_data_payload_t*)subpayload)->bumper_flag & KOBUKI_CENTRAL_BUMPER_FLAG ? 1 : 0;
        robot->right_bumper     = ((kobuki_basic_sensor_data_payload_t*)subpayload)->bumper_flag & KOBUKI_RIGHT_BUMPER_FLAG   ? 1 : 0;

        robot->wheel_drop       = ((kobuki_basic_sensor_data_payload_t*)subpayload)->wheel_drop_flag;
        robot->left_wheel_drop  = ((kobuki_basic_sensor_data_payload_t*)subpayload)->wheel_drop_flag & KOBUKI_LEFT_WHEEL_DROP_FLAG  ? 1 : 0;
        robot->right_wheel_drop = ((kobuki_basic_sensor_data_payload_t*)subpayload)->wheel_drop_flag & KOBUKI_RIGHT_WHEEL_DROP_FLAG ? 1 : 0;

        robot->cliff            = ((kobuki_basic_sensor_data_payload_t*)subpayload)->cliff_flag;
        robot->left_cliff       = ((kobuki_basic_sensor_data_payload_t*)subpayload)->cliff_flag & KOBUKI_LEFT_CLIFF_FLAG    ? 1 : 0;
        robot->central_cliff    = ((kobuki_basic_sensor_data_payload_t*)subpayload)->cliff_flag & KOBUKI_CENTRAL_CLIFF_FLAG ? 1 : 0;
        robot->right_cliff      = ((kobuki_basic_sensor_data_payload_t*)subpayload)->cliff_flag & KOBUKI_RIGHT_CLIFF_FLAG   ? 1 : 0;

        robot->left_encoder     = ((kobuki_basic_sensor_data_payload_t*)subpayload)->left_encoder;
        robot->right_encoder    = ((kobuki_basic_sensor_data_payload_t*)subpayload)->right_encoder;

        robot->left_pwm         = (int8_t)((kobuki_basic_sensor_data_payload_t*)subpayload)->left_pwm;
        robot->right_pwm        = (int8_t)((kobuki_basic_sensor_data_payload_t*)subpayload)->right_pwm;

        robot->button           = ((kobuki_basic_sensor_data_payload_t*)subpayload)->button_flag;
        robot->button_0         = ((kobuki_basic_sensor_data_payload_t*)subpayload)->button_flag & KOBUKI_BUTTON_0_FLAG ? 1 : 0;
        robot->button_1         = ((kobuki_basic_sensor_data_payload_t*)subpayload)->button_flag & KOBUKI_BUTTON_1_FLAG ? 1 : 0;
        robot->button_2         = ((kobuki_basic_sensor_data_payload_t*)subpayload)->button_flag & KOBUKI_BUTTON_2_FLAG ? 1 : 0;

        robot->charger                  = ((kobuki_basic_sensor_data_payload_t*)subpayload)->charger_flag;
        robot->charger_discharging      = ((kobuki_basic_sensor_data_payload_t*)subpayload)->charger_flag == KOBUKI_DISCHARGING_FLAG      ? 1 : 0;
        robot->charger_docking_charged  = ((kobuki_basic_sensor_data_payload_t*)subpayload)->charger_flag == KOBUKI_DOCKING_CHARGED_FLAG  ? 1 : 0;
        robot->charger_docking_charging = ((kobuki_basic_sensor_data_payload_t*)subpayload)->charger_flag == KOBUKI_DOCKING_CHARGING_FLAG ? 1 : 0;
        robot->charger_adapter_charged  = ((kobuki_basic_sensor_data_payload_t*)subpayload)->charger_flag == KOBUKI_ADAPTER_CHARGED_FLAG  ? 1 : 0;
        robot->charger_adapter_charging = ((kobuki_basic_sensor_data_payload_t*)subpayload)->charger_flag == KOBUKI_ADAPTER_CHARGING_FLAG ? 1 : 0;

        robot->wheel_overcurrent        = ((kobuki_basic_sensor_data_payload_t*)subpayload)->overcurrent_flag;
        robot->left_wheel_overcurrent   = ((kobuki_basic_sensor_data_payload_t*)subpayload)->overcurrent_flag & KOBUKI_LEFT_WHEEL_OVERCURRENT_FLAG  ? 1 : 0;
        robot->right_wheel_overcurrent  = ((kobuki_basic_sensor_data_payload_t*)subpayload)->overcurrent_flag & KOBUKI_RIGHT_WHEEL_OVERCURRENT_FLAG ? 1 : 0;

        break;

    case KOBUKI_DOCKING_IR_HEADER:
        robot->docking_ir_left          = ((kobuki_docking_ir_payload_t*)subpayload)->left_signal;
        robot->docking_ir_center        = ((kobuki_docking_ir_payload_t*)subpayload)->central_signal;
        robot->docking_ir_right         = ((kobuki_docking_ir_payload_t*)subpayload)->right_signal;

        robot->docking_ir_far_left      = ((kobuki_docking_ir_payload_t*)subpayload)->central_signal & KOBUKI_DOCKING_IR_FAR_LEFT_FLAG ? 1 : 0;
        robot->docking_ir_far_center    = ((kobuki_docking_ir_payload_t*)subpayload)->central_signal & KOBUKI_DOCKING_IR_FAR_CENTER_FLAG ? 1 : 0;
        robot->docking_ir_far_right     = ((kobuki_docking_ir_payload_t*)subpayload)->central_signal & KOBUKI_DOCKING_IR_FAR_RIGHT_FLAG ? 1 : 0;

        robot->docking_ir_near_left     = ((kobuki_docking_ir_payload_t*)subpayload)->central_signal & KOBUKI_DOCKING_IR_NEAR_LEFT_FLAG ? 1 : 0;
        robot->docking_ir_near_center   = ((kobuki_docking_ir_payload_t*)subpayload)->central_signal & KOBUKI_DOCKING_IR_NEAR_CENTER_FLAG ? 1 : 0;
        robot->docking_ir_near_right    = ((kobuki_docking_ir_payload_t*)subpayload)->central_signal & KOBUKI_DOCKING_IR_NEAR_RIGHT_FLAG ? 1 : 0;
        break;

    case KOBUKI_INERTIAL_SENSOR_DATA_HEADER:
        robot->inertial_angle           = ((kobuki_inertial_sensor_data_payload_t*)subpayload)->angle;
        robot->inertial_angle_rate      = ((kobuki_inertial_sensor_data_payload_t*)subpayload)->angle_rate;
        robot->v_theta                  = ( (double)(((kobuki_inertial_sensor_data_payload_t*)subpayload)->angle_rate) / 100.0) * (M_PI / 180.0);
        break;

    case KOBUKI_CLIFF_SENSOR_DATA_HEADER:
        robot->left_cliff_adc = ((kobuki_cliff_sensor_data_payload_t*)subpayload)->left_cliff_sensor;
        robot->central_cliff_adc = ((kobuki_cliff_sensor_data_payload_t*)subpayload)->central_cliff_sensor;
        robot->right_cliff_adc = ((kobuki_cliff_sensor_data_payload_t*)subpayload)->right_cliff_sensor;
        break;

    case KOBUKI_CURRENT_HEADER:
        robot->left_motor_current = ((kobuki_current_payload_t*)subpayload)->left_motor;
        robot->right_motor_current = ((kobuki_current_payload_t*)subpayload)->right_motor;
        break;

    case KOBUKI_HARDWARE_VERSION_HEADER:
        robot->harware_version_major = ((kobuki_hardware_version_payload_t*)subpayload)->major;
        robot->harware_version_minor = ((kobuki_hardware_version_payload_t*)subpayload)->minor;
        robot->harware_version_patch = ((kobuki_hardware_version_payload_t*)subpayload)->pathch;
        rt_event_send(&(robot->event), KOBUKI_RECV_HARDWARE_EVENT);
        break;

    case KOBUKI_FIRMWARE_VERSION_HEADER:
        robot->firmware_version_major = ((kobuki_firmware_version_payload_t*)subpayload)->major;
        robot->firmware_version_minor = ((kobuki_firmware_version_payload_t*)subpayload)->minor;
        robot->firmware_version_patch = ((kobuki_firmware_version_payload_t*)subpayload)->pathch;
        rt_event_send(&(robot->event), KOBUKI_RECV_FIRMWARE_EVENT);
        break;

    case KOBUKI_3D_GYRO_RAW_DATA_HEADER:
        // We don't need raw data because angle and angular velocity can be retrieved from Inertial Sensor Data
        break;

    case KOBUKI_GENERAL_PURPOSE_INPUT_HEADER:
        robot->analog_input_0 = ((kobuki_general_purpose_input_payload_t*)subpayload)->analog_input_ch_0;
        robot->analog_input_1 = ((kobuki_general_purpose_input_payload_t*)subpayload)->analog_input_ch_1;
        robot->analog_input_2 = ((kobuki_general_purpose_input_payload_t*)subpayload)->analog_input_ch_2;
        robot->analog_input_3 = ((kobuki_general_purpose_input_payload_t*)subpayload)->analog_input_ch_3;
        break;

    case KOBUKI_UUID_HEADER:
        robot->uuid_0 =((kobuki_uuid_payload_t*)subpayload)->uuid_0;
        robot->uuid_1 =((kobuki_uuid_payload_t*)subpayload)->uuid_1;
        robot->uuid_2 =((kobuki_uuid_payload_t*)subpayload)->uuid_2;
        rt_event_send(&(robot->event), KOBUKI_RECV_UUID_EVENT);
        break;

    case KOBUKI_CONTROLLER_INFO_HEADER:
        robot->kp = ((kobuki_controller_info_payload_t*)subpayload)->p_gain;
        robot->ki = ((kobuki_controller_info_payload_t*)subpayload)->i_gain;
        robot->kd = ((kobuki_controller_info_payload_t*)subpayload)->d_gain;
        rt_event_send(&(robot->event), KOBUKI_RECV_CONTROLLER_INFO_EVENT);
        break;

    default:
        LOG_E("Unknown Payload\n");
        break;
    }
}

void kobuki_loop(kobuki_t robot)
{
    if( (kobuki_get_tick() - robot->last_sync_tick) > KOBUKI_SYNC_TIMEOUT)
    {
        robot->connected = 0;
    }
    else
    {
        robot->connected = 1;
    }

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
        robot->last_sync_tick   = kobuki_get_tick();
        int i;
        for (i = 0; i < packet_len; i += robot->packet[i+1] + 2) {
            kobuki_parse_subpaylod(robot, robot->packet + i, robot->packet[i+1]);
        }
    }
}

void kobuki_close(kobuki_t robot)
{
    rt_event_detach(&(robot->event));
    kobuki_serial_close();
}
