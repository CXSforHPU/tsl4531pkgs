/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-16     JellyYe      First implementation
 */

#ifndef __TSL4531_H
#define __TSL4531_H

#include <rtdevice.h>
#include <sensor.h>

struct tsl4531_device
{
    struct rt_i2c_bus_device *i2c;
    rt_mutex_t lock;
};
typedef struct tsl4531_device *tsl4531_device_t;

#define TSL4531_ADDR        0x29
#define TSL4531_CMD_FIELD   0x80

/* Command */
#define TSL4531_CONTROL     0x00
#define TSL4531_CONFIG      0x01
#define TSL4531_DATALOW     0x04
#define TSL4531_DATAHIGH    0x05
#define TSL4531_ID          0x0A

/* Control mode */
#define TSL4531_CONTROL_POWERDOWN        0x00   /* Power-off */
#define TSL4531_CONTROL_SINGLEPOWERDOWN  0x02   /* Run an ADC cycle and return to Power-off */
#define TSL4531_CONTROL_NMALOperation    0x03   /* Normal Operation */

/* Set acquisition cycle (power saving mode) 1.
 * When PSAVESKIP = 0, the typical total cycle time is Tint + (60/MULTIPLIER) ms. 
 * When PSAVESKIP = 1, the typical total cycle time is Tint 
 */
#define TSL4531_CONFIG_1x  0x00  /* Tint = 400 ms */
#define TSL4531_CONFIG_2x  0x01  /* Tint = 200 ms */
#define TSL4531_CONFIG_4x  0x02  /* Tint = 100 ms */

int rt_hw_tsl4531_init(const char *name, struct rt_sensor_config *cfg);

#endif


