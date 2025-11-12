/*****************************************************************************
* | File      	:   ft6336_get_touch_count.cpp
* | Author      :   MLX
* | Function    :   FT6336 Touch Functions demo
* | Info        :   适配GoodDisplay的2.13寸触摸屏，在屏幕设置了加/减号区域，可在串口输出触摸点数和触摸坐标，增加/减少计数；
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "example-main";

// I2C引脚定义
// I2C定义
#define I2C_NUM         (I2C_NUM_0)
#define I2C_SCL         (GPIO_NUM_32) // A4, T9
#define I2C_SDA         (GPIO_NUM_33) // A5, T8
#define I2C_FREQ        (400000)

// 触摸屏FT6336定义（使用I2C）
#define TP_INT         GPIO_NUM_NC  // 不使用中断引脚，采用轮询
#define TP_RST         GPIO_NUM_NC  // 不使用复位引脚
ft6336_dev_t ft6336_dev;

// 屏幕尺寸定义
#define EPD_WIDTH       (128)
#define EPD_HEIGHT      (250)

/**
 * @brief 初始化硬件I2C的通用代码
 */
static esp_err_t i2c_master_init_ft6336(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_FREQ,
        },
    };
 
    esp_err_t err = i2c_param_config(I2C_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
 
    return i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0);
}

void ft6336_task(void *pvParameters) {
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "触摸屏任务启动");

    uint8_t num_count = 0;
    bool num_updated = false;
    
    while(1) {
        esp_task_wdt_reset();
        
        // 检查是否有触摸
        if (ft6336_is_touched(&ft6336_dev)) {
            // 读取触摸数据
            if (ft6336_read_touch(&ft6336_dev) == ESP_OK) {
                // 处理触摸点数据
                for (int i = 0; i < ft6336_dev.touch_count; i++) {
                    if (ft6336_dev.points[i].valid) {
                        uint16_t x = ft6336_dev.points[i].x;
                        uint16_t y = ft6336_dev.points[i].y;
                        
                        ESP_LOGI(TAG, "触摸点 %d: x=%d, y=%d", 
                                 ft6336_dev.points[i].id, x, y);
                        
                        // 左侧减号按钮区域
                        if (x > 0 && x < 60 && y > 80 && y < 110) {
                            // 减少计数
                            num_count --;
                            ESP_LOGI(TAG, "触摸减号，计数变为: %d", num_count);
                            
                            // 标记计数需要更新
                            num_updated = true;
                        }
                        // 右侧加号按钮区域
                        else if (x > 190 && x < 250 && y > 80 && y < 110) {
                            // 增加计数
                            num_count ++;
                            ESP_LOGI(TAG, "触摸加号，计数变为: %d", num_count);
                            
                            // 标记计数需要更新
                            num_updated = true;
                        }
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));  // 20ms检测一次
    }
    
    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);
}

extern "C" void app_main(void) {
    // 初始化硬件I2C总线
    i2c_master_init_ft6336();

    // 初始化触摸屏
    esp_err_t ret = ft6336_init(&ft6336_dev, I2C_NUM, I2C_SDA, I2C_SCL,
                            TP_RST, TP_INT, EPD_WIDTH, EPD_HEIGHT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FT6336初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置屏幕旋转方向（根据实际情况调整）
    ft6336_set_rotation(&ft6336_dev, FT6336_ROTATION_LEFT);

    xTaskCreatePinnedToCore(ft6336_task, "ft6336_task", 4096, NULL, 5, NULL, 0); // 创建触摸屏任务
}