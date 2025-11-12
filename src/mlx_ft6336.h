/*****************************************************************************
* | File      	:   mlx_ft6336.h
* | Author      :   MLX
* | Function    :   FT6336 Touch Functions
* | Info        :   参考了GoodDisplay官网的FT6336驱动和Arduino通用的FT6336驱动，进行了独立整合；
* |             :   用硬件 I2C 通信，简化了代码；
* |             :   移除中断依赖，支持轮询方式检测触摸；
* |             :   支持 RST 和 INT 引脚的可选配置；
* |             :   增加屏幕旋转和坐标映射功能；
*----------------
* |	This version:   V1.0
* | Date        :   2025-11-13
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#ifndef MLX_FT6336_H
#define MLX_FT6336_H

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define FT6336_ADDR                 0x38        // I2C地址（7位）
#define FT6336_ADDR_8BIT            (FT6336_ADDR << 1)  // 8位地址

// 旋转方向定义
#define FT6336_ROTATION_NORMAL      0
#define FT6336_ROTATION_RIGHT       1
#define FT6336_ROTATION_INVERTED    2
#define FT6336_ROTATION_LEFT        3

#define FT6336_ROTATION_0           (FT6336_ROTATION_NORMAL)
#define FT6336_ROTATION_90          (FT6336_ROTATION_LEFT)
#define FT6336_ROTATION_180         (FT6336_ROTATION_INVERTED)
#define FT6336_ROTATION_270         (FT6336_ROTATION_RIGHT)

// 寄存器定义
#define FT6336_REG_DEVICE_MODE      0x00
#define FT6336_REG_TD_STATUS        0x02
#define FT6336_REG_TOUCH1           0x03
#define FT6336_REG_TOUCH2           0x09
#define FT6336_REG_TH_GROUP         0x80
#define FT6336_REG_PERIOD_ACTIVE    0x88
#define FT6336_REG_FIRMWARE_ID      0xA6
#define FT6336_REG_CHIP_ID          0xA8

// 触摸点结构体
typedef struct {
    uint8_t id;         // 触摸点ID
    uint16_t x;         // X坐标
    uint16_t y;         // Y坐标
    bool valid;         // 有效标志
} ft6336_touch_point_t;

// FT6336设备结构体
typedef struct {
    i2c_port_t i2c_port;        // I2C端口号
    uint8_t addr;               // I2C地址（7位）
    gpio_num_t rst_pin;         // 复位引脚，-1表示不使用
    gpio_num_t int_pin;         // 中断引脚，-1表示不使用
    uint16_t screen_width;      // 屏幕宽度
    uint16_t screen_height;     // 屏幕高度
    uint8_t rotation;           // 旋转方向
    ft6336_touch_point_t points[2];  // 触摸点数据
    uint8_t touch_count;        // 触摸点数量
} ft6336_dev_t;

/**
 * @brief 初始化FT6336触摸屏
 * 
 * @param dev FT6336设备结构体指针
 * @param i2c_port I2C端口号
 * @param sda_pin SDA引脚
 * @param scl_pin SCL引脚
 * @param rst_pin 复位引脚，-1表示不使用
 * @param int_pin 中断引脚，-1表示不使用
 * @param width 屏幕宽度
 * @param height 屏幕高度
 * @return esp_err_t 成功返回ESP_OK，否则返回错误码
 */
esp_err_t ft6336_init(ft6336_dev_t *dev, i2c_port_t i2c_port, 
                     gpio_num_t sda_pin, gpio_num_t scl_pin,
                     gpio_num_t rst_pin, gpio_num_t int_pin,
                     uint16_t width, uint16_t height);

/**
 * @brief 设置屏幕旋转方向
 * 
 * @param dev FT6336设备结构体指针
 * @param rotation 旋转方向，取值为FT6336_ROTATION_*
 */
void ft6336_set_rotation(ft6336_dev_t *dev, uint8_t rotation);

/**
 * @brief 读取触摸数据
 * 
 * @param dev FT6336设备结构体指针
 * @return esp_err_t 成功返回ESP_OK，否则返回错误码
 */
esp_err_t ft6336_read_touch(ft6336_dev_t *dev);

/**
 * @brief 检查是否有触摸
 * 
 * @param dev FT6336设备结构体指针
 * @return bool 有触摸返回true，否则返回false
 */
bool ft6336_is_touched(ft6336_dev_t *dev);

/**
 * @brief 复位触摸屏
 * 
 * @param dev FT6336设备结构体指针
 * @return esp_err_t 成功返回ESP_OK，否则返回错误码
 */
esp_err_t ft6336_reset(ft6336_dev_t *dev);

/**
 * @brief 写入寄存器
 * 
 * @param dev FT6336设备结构体指针
 * @param reg 寄存器地址
 * @param data 要写入的数据
 * @return esp_err_t 成功返回ESP_OK，否则返回错误码
 */
esp_err_t ft6336_write_reg(ft6336_dev_t *dev, uint8_t reg, uint8_t data);

/**
 * @brief 读取寄存器
 * 
 * @param dev FT6336设备结构体指针
 * @param reg 寄存器地址
 * @param data 读取到的数据
 * @return esp_err_t 成功返回ESP_OK，否则返回错误码
 */
esp_err_t ft6336_read_reg(ft6336_dev_t *dev, uint8_t reg, uint8_t *data);

#endif // MLX_FT6336_H