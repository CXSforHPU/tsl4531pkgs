#ifndef __TSL4531_H
#define __TSL4531_H
#include <rtdevice.h>
#include <sensor.h>
struct tsl4531_device
{
    struct rt_i2c_bus_device *i2c;

#ifdef AHT10_USING_SOFT_FILTER
    filter_data_t temp_filter;
    filter_data_t humi_filter;

    rt_thread_t thread;
    rt_uint32_t period; //sample period
#endif /* AHT10_USING_SOFT_FILTER */

    rt_mutex_t lock;
};

typedef struct tsl4531_device *tsl4531_device_t;

#define TSL4531_ADDR 0x29
#define TSL4531_CMD_FIELD   0x80
//命令寄存器地址
#define TSL4531_CONTROL 	0x00
#define TSL4531_CONFIG 		0x01
#define TSL4531_DATALOW 	0x04
#define TSL4531_DATAHIGH 	0x05
#define TSL4531_ID 	      0x0A

//控制寄存器的控制模式
#define TSL4531_CONTROL_POWERDOWN        0x00//掉電
#define TSL4531_CONTROL_SINGLEPOWERDOWN  0x02 //运行一个ADC周期并返回到掉电
#define TSL4531_CONTROL_NMALOperation    0x03//Normal Operation

//採集週期設置(省電模式)1. When PSAVESKIP = 0, the typical total cycle time is Tint + (60/MULTIPLIER) ms. When PSAVESKIP = 1, the typical total cycle time is Tint
#define TSL4531_CONFIG_1x  0x00//Tint = 400 ms
#define TSL4531_CONFIG_2x  0x01//Tint = 200 ms
#define TSL4531_CONFIG_4x  0x02//Tint = 100 ms
static rt_err_t sensor_tsl4531_init(tsl4531_device_t dev);
tsl4531_device_t tsl4531_init(const char *i2c_bus_name);
int rt_hw_tsl4531_init(const char *name, struct rt_sensor_config *cfg);

#endif


