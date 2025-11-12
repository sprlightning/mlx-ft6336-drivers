/*****************************************************************************
* | File      	:   mlx_ft6336.cpp
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
#include "mlx_ft6336.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string.h" // memset()

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_SCL_IO 32
#define I2C_MASTER_SDA_IO 33

static const char *TAG = "FT6336";

/**
 * @brief 初始化硬件I2C的通用代码（主函数中已包含统一的硬件I2C初始化代码，这里实际上不使用，仅供参考）
 */
static esp_err_t i2c_master_init_ft6336(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
    };
 
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
 
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief 初始化FT6336触摸屏
 */
esp_err_t ft6336_init(ft6336_dev_t *dev, i2c_port_t i2c_port, 
                     gpio_num_t sda_pin, gpio_num_t scl_pin,
                     gpio_num_t rst_pin, gpio_num_t int_pin,
                     uint16_t width, uint16_t height) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 初始化设备结构体
    dev->i2c_port = i2c_port;
    dev->addr = FT6336_ADDR;
    dev->rst_pin = rst_pin;
    dev->int_pin = int_pin;
    dev->screen_width = width;
    dev->screen_height = height;
    dev->rotation = FT6336_ROTATION_NORMAL;
    dev->touch_count = 0;
    memset(dev->points, 0, sizeof(dev->points));

    // 配置RST引脚
    if (dev->rst_pin != GPIO_NUM_NC) {
        gpio_config_t gpio_conf = {
            .pin_bit_mask = (1ULL << dev->rst_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&gpio_conf);
    }

    // 配置INT引脚（如果使用）
    if (dev->int_pin != GPIO_NUM_NC) {
        gpio_config_t gpio_conf = {
            .pin_bit_mask = (1ULL << dev->int_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&gpio_conf);
    }

    // 复位触摸屏
    esp_err_t ret = ft6336_reset(dev);
    if (ret != ESP_OK) {
        return ret;
    }

    // 检查芯片ID
    uint8_t chip_id;
    ret = ft6336_read_reg(dev, FT6336_REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }

    if (chip_id != 0x11) {
        vTaskDelay(pdMS_TO_TICKS(5)); // 等待5ms，确保芯片已复位
        // 再读一次
        ret = ft6336_read_reg(dev, FT6336_REG_CHIP_ID, &chip_id);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read chip ID");
            return ret;
        }
        if (chip_id != 0x11) ESP_LOGW(TAG, "Invalid chip ID: 0x%02X", chip_id);
    }

    // 初始化触摸屏配置
    ft6336_write_reg(dev, FT6336_REG_DEVICE_MODE, 0x00);      // 正常操作模式
    ft6336_write_reg(dev, FT6336_REG_TH_GROUP, 0x16);         // 触摸阈值，越小越灵敏
    ft6336_write_reg(dev, FT6336_REG_PERIOD_ACTIVE, 0x0E);    // 激活周期

    ESP_LOGI(TAG, "FT6336 initialized successfully");
    return ESP_OK;
}

/**
 * @brief 设置屏幕旋转方向
 */
void ft6336_set_rotation(ft6336_dev_t *dev, uint8_t rotation) {
    if (dev == NULL) return;
    if (rotation >= 4) rotation = 0;
    dev->rotation = rotation;
}

/**
 * @brief 读取触摸数据
 */
esp_err_t ft6336_read_touch(ft6336_dev_t *dev) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 读取触摸状态
    uint8_t status;
    esp_err_t ret = ft6336_read_reg(dev, FT6336_REG_TD_STATUS, &status);
    if (ret != ESP_OK) {
        return ret;
    }

    dev->touch_count = status & 0x0F;
    if (dev->touch_count > 2) {
        dev->touch_count = 0;
    }

    // 清除之前的触摸点数据
    for (int i = 0; i < 2; i++) {
        dev->points[i].valid = false;
    }

    // 读取触摸点数据
    for (int i = 0; i < dev->touch_count; i++) {
        uint8_t data[4];
        uint8_t reg = (i == 0) ? FT6336_REG_TOUCH1 : FT6336_REG_TOUCH2;
        
        // 读取4字节触摸数据
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, FT6336_ADDR_8BIT | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set register address");
            return ret;
        }

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, FT6336_ADDR_8BIT | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, 4, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read touch data");
            return ret;
        }

        // 解析触摸数据
        uint16_t x = ((data[0] & 0x0F) << 8) | data[1];
        uint16_t y = ((data[2] & 0x0F) << 8) | data[3];
        uint8_t id = (data[2] >> 4) & 0x0F;
        
        // 检查触摸事件是否有效
        uint8_t event = (data[0] >> 6) & 0x03;
        if (event == 0x00 || event == 0x02) { // 按下或接触
            // 根据旋转方向调整坐标
            uint16_t temp;
            switch (dev->rotation) {
                case FT6336_ROTATION_NORMAL:
                    // 正常方向，不调整
                    break;
                case FT6336_ROTATION_RIGHT:
                    temp = x;
                    x = y;
                    y = dev->screen_width - temp;
                    break;
                case FT6336_ROTATION_INVERTED:
                    x = dev->screen_width - x;
                    y = dev->screen_height - y;
                    break;
                case FT6336_ROTATION_LEFT:
                    temp = x;
                    x = dev->screen_height - y;
                    y = temp;
                    break;
            }

            // 存储触摸点数据
            dev->points[i].id = id;
            dev->points[i].x = x;
            dev->points[i].y = y;
            dev->points[i].valid = true;
        }
    }

    return ESP_OK;
}

/**
 * @brief 检查是否有触摸
 */
bool ft6336_is_touched(ft6336_dev_t *dev) {
    if (dev == NULL) return false;

    // 如果使用了INT引脚，先检查INT引脚状态
    if (dev->int_pin != GPIO_NUM_NC) {
        if (gpio_get_level(dev->int_pin) != 0) {
            return false;
        }
    }

    // 读取触摸状态寄存器确认
    uint8_t status;
    if (ft6336_read_reg(dev, FT6336_REG_TD_STATUS, &status) != ESP_OK) {
        return false;
    }

    return (status & 0x0F) > 0;
}

/**
 * @brief 复位触摸屏
 */
esp_err_t ft6336_reset(ft6336_dev_t *dev) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 如果没有复位引脚，直接返回成功
    if (dev->rst_pin == GPIO_NUM_NC) {
        vTaskDelay(pdMS_TO_TICKS(100));
        return ESP_OK;
    }

    // 执行复位序列
    gpio_set_level(dev->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(dev->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

/**
 * @brief 写入寄存器
 */
esp_err_t ft6336_write_reg(ft6336_dev_t *dev, uint8_t reg, uint8_t data) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, FT6336_ADDR_8BIT | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * @brief 读取寄存器
 */
esp_err_t ft6336_read_reg(ft6336_dev_t *dev, uint8_t reg, uint8_t *data) {
    if (dev == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 发送寄存器地址
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, FT6336_ADDR_8BIT | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }

    // 读取数据
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, FT6336_ADDR_8BIT | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}
