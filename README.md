# TSL4531

## 简介
TSL4531 软件包提供了使用光照传感器 `TSL4531` 基本功能。并且本软件包新的版本已经对接到了 Sensor 框架，通过 Sensor 框架，开发者可以快速的将此传感器驱动起来。

输入电压`2.3v - 3.3v` 量程、精度如下表所示

| 功能 | 量程 | 精度 |
| ---- | ---- | ---- |
| LUX值 | `3 lux-220k lux` |`±1‘|


## 支持情况

| 包含设备 | 温度 | 湿度 |
| ---- | ---- | ---- |
| **通信接口** |          |        |
| IIC      | √        | √      |
| **工作模式**     |          |        |
| 轮询             | √        | √      |
| 中断             |          |        |
| FIFO             |          |        |

## 使用说明

### 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- IIC 驱动：tsl4531 设备使用 IIC 进行数据通讯，需要系统 IIC 驱动支持

### 获取软件包

使用 aht10 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages  --->
  peripheral libraries and drivers  --->
    sensors drivers  --->
            tsl4531: digital humidity and temperature sensor tsl4531 driver library. 
                         [ ]   Enable average filter by software         
                               Version (latest)  --->
```



**Version**：软件包版本选择，默认选择最新版本。

### 使用软件包

tsl4531 软件包初始化函数如下所示：

```
    rt_hw_tsl4531_init("tsl4531", &cfg);
```

该函数需要由用户调用，函数主要完成的功能有，

- 设备配置和初始化（根据传入的配置信息配置接口设备）；
- 注册相应的传感器设备，完成 tsl4531 设备的注册；

#### 初始化示例

```c
#ifdef BSP_USING_TSL4531
#include "dr_tsl4531.h"

#define TSL4531_I2C_BUS  "i2c3"

int rt_hw_tsl4531_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name  = TSL4531_I2C_BUS;
    cfg.intf.user_data = (void *)TSL4531_ADDR;

    rt_hw_tsl4531_init("tsl4531", &cfg);

    return RT_EOK;
}
INIT_ENV_EXPORT(rt_hw_tsl4531_port);
#endif
```

## 注意事项

注意设备地址要正确。

## 联系人信息

维护人:984490128@qq.com

- 维护：
- 主页：
