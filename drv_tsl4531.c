/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-16     JellyYe      First implementation
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>
#include <drv_tsl4531.h>

#define SENSOR_LUX_RANGE_MAX 4000
#define SENSOR_LUX_RANGE_MIN 0

/* Following is the driver part of sensor hardware equipment */

/* Write Sensor Register */
static rt_err_t write_reg(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t *data)
{
    rt_uint8_t buf[2];
    struct rt_i2c_msg msgs;

    buf[0] = TSL4531_CMD_FIELD | reg;
    buf[1] = data[0];

    msgs.addr = TSL4531_ADDR;  
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = 2;

    /* Transfer data by calling I2C device interface */
    if (rt_i2c_transfer(bus, &msgs,  1) == 1)
    { 
        return RT_EOK; 
    }
    else
    {
        return -RT_ERROR;
    }
}

/* Read Data form Sensor Register */
static rt_err_t read_regs(struct rt_i2c_bus_device *bus, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msgs;

    msgs.addr = TSL4531_ADDR;
    msgs.flags = RT_I2C_RD;
    msgs.buf = buf;
    msgs.len = len;

    /* Transfer data by calling I2C device interface */
    if (rt_i2c_transfer(bus, &msgs,  1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

/* Sensor Device Initialization */
static rt_err_t sensor_tsl4531_init(tsl4531_device_t dev)
{ 
    rt_uint8_t temp[1] = {0};
    temp[0] = TSL4531_CONTROL_NMALOperation;
    write_reg(dev->i2c , TSL4531_CONTROL, temp);    
        
    rt_thread_delay(rt_tick_from_millisecond(500)); 
        
    temp[0] = TSL4531_CONFIG_1x;
    write_reg(dev->i2c, TSL4531_CONFIG, temp); /* go into calibration */
    
   return RT_EOK;
}

static float tls4531_read_lux(tsl4531_device_t dev)
{
    static rt_uint8_t temp[2];
    static float lux = -50.0;  /* The data is error with missing measurement */
    rt_err_t result;

    RT_ASSERT(dev);

    rt_uint8_t cmd[1] = {0};
    write_reg(dev->i2c, TSL4531_DATAHIGH, cmd);       /* sample data cmd */
    rt_thread_mdelay(40);
    read_regs(dev->i2c, 1, &temp[0]);                 /* get data */
    
    rt_thread_delay(rt_tick_from_millisecond(50)); 
    
    write_reg(dev->i2c, TSL4531_DATALOW, cmd);        /* sample data cmd */
    rt_thread_mdelay(40);
    read_regs(dev->i2c, 1, &temp[1]);                 /* get data */
    
    /*sensor temperature converse to reality */
    lux = ((temp[0]  << 8) + temp[1]);

    return lux;
}

/* Allocate and initialize the heap of devices */
 static tsl4531_device_t tsl4531_init(const char *i2c_bus_name)
{
    tsl4531_device_t dev;

    RT_ASSERT(i2c_bus_name);

    dev = rt_calloc(1, sizeof(struct tsl4531_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for tsl4531 device on '%s' ", i2c_bus_name);
        return RT_NULL;
    }

    dev->i2c = rt_i2c_bus_device_find(i2c_bus_name);
    if (dev->i2c == RT_NULL)
    {
        LOG_E("Can't find tsl4531 device on '%s' ", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    sensor_tsl4531_init(dev);

    return dev;
}

/* The following is the sensor device driver */
static struct tsl4531_device *lux_dev;

static rt_err_t _sensor_tsl4531_init(struct rt_sensor_intf *intf)
{
    lux_dev = tsl4531_init(intf->dev_name);

    if (lux_dev == RT_NULL)
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

void tsl4531_deinit(tsl4531_device_t dev)
{
    RT_ASSERT(dev);
    rt_free(dev);
}

static rt_size_t _tsl4531_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    rt_int32_t luxvalue;
    
    if (sensor->info.type == RT_SENSOR_CLASS_LIGHT)
    {
        luxvalue =  tls4531_read_lux(lux_dev);
        data->data.light = (rt_int32_t)luxvalue;
        data->timestamp = rt_sensor_get_ts();
    }    
  return 1;
}

static rt_ssize_t tsl4531_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _tsl4531_polling_get_data(sensor, buf);
    }
    else
    {
        return 0;
    }
}

static rt_err_t tsl4531_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    tsl4531_fetch_data,
    tsl4531_control
};

int rt_hw_tsl4531_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_lux = RT_NULL;
    
     /* Registered Illumination Sensor Devcie */
    sensor_lux = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor_lux == RT_NULL)
        return -1;

    sensor_lux->info.type       = RT_SENSOR_CLASS_LIGHT;
    sensor_lux->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
    sensor_lux->info.model      = "tsl4531";
    sensor_lux->info.unit       = RT_SENSOR_UNIT_LUX;
    sensor_lux->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor_lux->info.range_max  = SENSOR_LUX_RANGE_MAX;
    sensor_lux->info.range_min  = SENSOR_LUX_RANGE_MIN;
    sensor_lux->info.period_min = 6;

    rt_memcpy(&sensor_lux->config, cfg, sizeof(struct rt_sensor_config));
    sensor_lux->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor_lux, name, RT_DEVICE_FLAG_RDONLY, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        goto __exit;
    }

    _sensor_tsl4531_init(&cfg->intf);
    return RT_EOK;
    
    __exit:
    if (sensor_lux)
        rt_free(sensor_lux);
    if (lux_dev)
        tsl4531_deinit(lux_dev);
    return -RT_ERROR;     
}


