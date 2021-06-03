## Kobuki Robot

Kobuki Robot serial communication driver.

<img src="docs/logo_kobuki.png" width="200">

By default, UART2 is used for communication with Kobuki on RT-Thread (RTOS). 

The driver is mostly platform independent. Porting to a new platform only requires implementation of basic serial operations in `kobuki_serial.c`: 

```
int  kobuki_serial_init();
char kobuki_serial_read();
int  kobuki_serial_write(uint8_t* dat, int len);
int  kobuki_serial_write_char(uint8_t ch);
void kobuki_serial_close();
int kobuki_get_tick();
```

### Supported APIs

#### Initialization

```
void kobuki_init();
```

```
void kobuki_close();
```

#### Set speed

```
void kobuki_set_speed(double tv, double rv);
```

#### Control power supply

```
void kobuki_enalbe_3v3();
```

```
void kobuki_enable_5v();
```

```
void kobuki_enable_12v_5a();
```

```
void kobuki_enable_12v_1_5a();
```

#### Control GPIO

```
void kobuki_set_led1_red();
```

```
void kobuki_set_led1_green();
```

```
void kobuki_set_led2_red();
```

```
void kobuki_set_led2_green();
```

#### PID Controller

```
void kobuki_set_controller_gain(uint32_t kp, uint32_t ki, uint32_t kd);
```

```
void kobuki_get_controller_gain();
```

```
void kobuki_reset_controller_gain();
```

#### Play sound

```
void kobuki_play_sound(uint16_t note, uint8_t duration);
```

```
void kobuki_play_sound_on();
```

```
void kobuki_play_sound_off();
```

```
void kobuki_play_sound_recharge();
```

```
void kobuki_play_sound_button();
```

```
void kobuki_play_sound_error();
```

```
void kobuki_play_sound_cleaning_start();
```

```
void kobuki_play_sound_cleaning_end();
```

#### Version

```
void kobuki_get_hardware_version();
```

```
void kobuki_get_firmware_version();
```

```
void kobuki_get_uuid();
```

#### Full list of robot status

```
struct kobuki {
    struct rt_event event;
    
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
} kobuki;
```

