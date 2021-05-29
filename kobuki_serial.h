/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-28     hw630       the first version
 */
#ifndef APPLICATIONS_KOBUKI_SERIAL_H_
#define APPLICATIONS_KOBUKI_SERIAL_H_

#include <rtthread.h>

#ifndef KOBUKI_SERIAL_NAME
    #define KOBUKI_SERIAL_NAME "uart2"
#endif

int  kobuki_serial_setup();
char kobuki_serial_read();
int  kobuki_serial_write(uint8_t* dat, int len);
int  kobuki_serial_write_char(uint8_t ch);

#endif /* APPLICATIONS_KOBUKI_SERIAL_H_ */
