# mlx-ft6336-drivers
Use FT6336 touch IC on ESP-IDF without INT# or RST#.

## 功能说明

参考了GD提供的FT6336驱动和Arduino的FT6336驱动，进行了独立整合；特点如下：

- 用硬件 I2C 通信，简化了代码；
- 移除中断依赖，支持轮询方式检测触摸；
- 支持 RST 和 INT 引脚的可选配置；
- 增加屏幕旋转和坐标映射功能；

常规配置，同时使用SDA、SCL、RST#和INT#引脚，这些引脚都配置为上拉模式，其中RST#配置为（推挽）输出；

最简配置，仅使用SDA和SCL引脚，推荐将RST#连接到MCU的RST引脚，INT#悬空/10k电阻上拉；

极简配置，仅使用SDA和SCL引脚，将RST#和INT#均通过10k电阻上拉；

## 示例代码

见example，适配GoodDisplay的2.13寸触摸屏，在屏幕设置了加/减号区域，可在串口输出触摸点数和触摸坐标，增加/减少计数；

