/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-28     hw630       the first version
 */
#ifndef KOBUKI_SERIAL_H_
#define KOBUKI_SERIAL_H_

#include <rtthread.h>


#ifndef KOBUKI_SERIAL_NAME
    #define KOBUKI_SERIAL_NAME "uart2"
#endif

#ifndef KOBUKI_SERIAL_TIMEOUT
    #define KOBUKI_SERIAL_TIMEOUT 500
#endif

int  kobuki_serial_init(void);
char kobuki_serial_read(void);
int  kobuki_serial_write(uint8_t* dat, int len);
int  kobuki_serial_write_char(uint8_t ch);
void kobuki_serial_close(void);

int kobuki_get_tick(void);

#endif /* KOBUKI_SERIAL_H_ */
