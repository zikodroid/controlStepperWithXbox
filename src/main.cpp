/*
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "XboxController.h"

static const char *TAG = "STEPPER_CONTROL";

// --- توصيلات العتاد ---
#define STEP_PIN          GPIO_NUM_1
#define DIR_PIN           GPIO_NUM_2
#define ENABLE_PIN        GPIO_NUM_3
#define I2C_SDA_PIN       GPIO_NUM_4
#define I2C_SCL_PIN       GPIO_NUM_5

#define I2C_MASTER_NUM    I2C_NUM_0
#define AS5600_ADDR       0x36
#define AS5600_RAW_ANGLE  0x0C

// --- إعدادات LEDC ---
#define LEDC_TIMER        LEDC_TIMER_0
#define LEDC_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL      LEDC_CHANNEL_0
#define LEDC_DUTY_RES     LEDC_TIMER_10_BIT

#define INVERT_DIRECTION  false 

// --- ثوابت معايرة العصا اليسرى الحقيقية (Unsigned 16-bit) ---
#define STICK_CENTER_X    33150
#define DEADZONE          2000
#define MAX_DELTA_X       32000.0f  // أقصى إزاحة متوقعة عن السنتر

XboxController xbox;
volatile uint16_t joy_left_x = STICK_CENTER_X; // البداية عند نقطة السكون

float current_measured_angle = 0.0f;
float current_measured_rpm = 0.0f;

esp_err_t init_i2c(void) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_PIN;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

float read_as5600_angle(void) {
    uint8_t reg = AS5600_RAW_ANGLE;
    uint8_t data[2] = {0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        uint16_t raw_val = ((uint16_t)data[0] << 8) | data[1];
        raw_val &= 0x0FFF;
        return (raw_val * 360.0f) / 4096.0f;
    }
    return -1.0f;
}

void init_stepper(void) {
    gpio_reset_pin(DIR_PIN);
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(ENABLE_PIN);
    gpio_set_direction(ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ENABLE_PIN, 0); // تفعيل ثابت للمحرك (Holding Torque)

    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = 1000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {
        .gpio_num       = STEP_PIN,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ch_conf);
}

// دالة التحكم بمنطق دواسة البنزين المحدثة بناءً على القيم الحقيقية للمحور (0 .. 65535)
void update_stepper(uint16_t raw_x) {
    static uint32_t last_freq = 0;
    
    // 1. حساب الانحراف عن نقطة السكون الحقيقية (33150)
    int32_t delta = (int32_t)raw_x - STICK_CENTER_X;
    int32_t abs_delta = abs(delta);

    // 2. تطبيق نطاق السكون (Deadzone) عند منتصف الحركة
    if (abs_delta <= DEADZONE) {
        if (last_freq != 0) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            last_freq = 0;
        }
        return;
    }

    // 3. تحديد الاتجاه (يمين > 33150 = CW، يسار < 33150 = CCW)
    bool dir = (delta > 0);
    if (INVERT_DIRECTION) dir = !dir;
    gpio_set_level(DIR_PIN, dir ? 1 : 0);

    // 4. حساب نسبة دواسة البنزين (من 0.0 عند حافة السكون إلى 1.0 عند أقصى شدة)
    float throttle = (float)(abs_delta - DEADZONE) / (MAX_DELTA_X - DEADZONE);
    if (throttle > 1.0f) throttle = 1.0f;
    if (throttle < 0.0f) throttle = 0.0f;

    // 5. حساب التردد المطلوب
    uint32_t min_freq = 800;   // بداية الحركة البطمية
    uint32_t max_freq = 5000;  // أقصى سرعة
    uint32_t target_freq = min_freq + (uint32_t)(throttle * (max_freq - min_freq));

    // 6. التحديث المباشر للمحرك عبر LEDC
    if (abs((int32_t)target_freq - (int32_t)last_freq) > 20) {
        ledc_set_freq(LEDC_MODE, LEDC_TIMER, target_freq);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 512); // 50% Duty Cycle
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        last_freq = target_freq;
    }
}

void encoder_task(void *pvParameters) {
    static float prev_angle = 0.0f;
    static int64_t prev_time = 0;

    prev_angle = read_as5600_angle();
    prev_time = esp_timer_get_time();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));

        float new_angle = read_as5600_angle();
        int64_t new_time = esp_timer_get_time();

        if (new_angle >= 0.0f) {
            float delta_angle = new_angle - prev_angle;
            float delta_time_sec = (new_time - prev_time) / 1000000.0f;

            if (delta_angle > 180.0f) {
                delta_angle -= 360.0f;
            } else if (delta_angle < -180.0f) {
                delta_angle += 360.0f;
            }

            if (delta_time_sec > 0) {
                current_measured_rpm = (delta_angle / 360.0f) / (delta_time_sec / 60.0f);
            }

            current_measured_angle = new_angle;
            prev_angle = new_angle;
            prev_time = new_time;

            printf("Position: %6.2f Deg | Actual Speed: %6.1f RPM\n", 
                   current_measured_angle, current_measured_rpm);
        }
    }
}

void onControllerData(const XboxData &data) {
    joy_left_x = data.left_x;
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "=============================================");
    ESP_LOGI(TAG, "    FIRMWARE UPDATED: REAL_AXIS_THROTTLE    ");
    ESP_LOGI(TAG, "=============================================");

    ESP_ERROR_CHECK(init_i2c());
    init_stepper();

    xbox.onData(onControllerData);
    xbox.begin();

    xTaskCreate(encoder_task, "encoder_task", 4096, NULL, 5, NULL);

    while (1) {
        update_stepper(joy_left_x);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
    */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "XboxController.h"

static const char *TAG = "NEMA17_SMOOTH_CONTROL";

// Hardware connections
#define STEP_PIN          GPIO_NUM_1
#define DIR_PIN           GPIO_NUM_2
#define ENABLE_PIN        GPIO_NUM_3
#define I2C_SDA_PIN       GPIO_NUM_4
#define I2C_SCL_PIN       GPIO_NUM_5

#define I2C_MASTER_NUM    I2C_NUM_0
#define AS5600_ADDR       0x36
#define AS5600_RAW_ANGLE  0x0C

// Stick calibration constants (Unsigned 16-bit)
#define STICK_CENTER_X    33150
#define DEADZONE          2500
#define MAX_DELTA_X       31000.0f

// Adjusted microsecond delay range for NEMA 17 stability
#define MIN_DELAY_US      800   // Max speed (~1250 steps/sec - safe for NEMA 17)
#define MAX_DELAY_US      4000  // Min speed (~250 steps/sec - smooth start)

#define INVERT_DIRECTION  false 

XboxController xbox;
volatile uint16_t joy_left_x = STICK_CENTER_X;

volatile uint32_t g_step_delay_us = 0; // 0 means motor is stopped
volatile bool g_direction = true;       // true: CW, false: CCW

float current_measured_angle = 0.0f;
float current_measured_rpm = 0.0f;

esp_err_t init_i2c(void) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_PIN;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

float read_as5600_angle(void) {
    uint8_t reg = AS5600_RAW_ANGLE;
    uint8_t data[2] = {0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        uint16_t raw_val = ((uint16_t)data[0] << 8) | data[1];
        raw_val &= 0x0FFF;
        return (raw_val * 360.0f) / 4096.0f;
    }
    return -1.0f;
}

void init_stepper_gpio(void) {
    gpio_reset_pin(STEP_PIN);
    gpio_set_direction(STEP_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(DIR_PIN);
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(ENABLE_PIN);
    gpio_set_direction(ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ENABLE_PIN, 0); // Active Low driver enable
}

// Precise time-based stepper task using esp_timer
void stepper_task(void *pvParameters) {
    int64_t last_step_time = esp_timer_get_time();

    while (1) {
        uint32_t delay = g_step_delay_us;

        if (delay > 0) {
            int64_t now = esp_timer_get_time();

            if (now - last_step_time >= delay) {
                last_step_time = now;

                gpio_set_level(DIR_PIN, g_direction ? 1 : 0);
                
                // STEP pulse HIGH
                gpio_set_level(STEP_PIN, 1);
                esp_rom_delay_us(5); // 5us pulse width for A4988
                
                // STEP pulse LOW
                gpio_set_level(STEP_PIN, 0);
            }

            // Yield execution briefly to keep CPU healthy and prevent watchdog reset
            esp_rom_delay_us(20);
        } else {
            // Idle state when motor is stopped
            vTaskDelay(pdMS_TO_TICKS(10));
            last_step_time = esp_timer_get_time();
        }
    }
}

void update_stepper_logic(uint16_t raw_x) {
    int32_t delta = (int32_t)raw_x - STICK_CENTER_X;
    int32_t abs_delta = abs(delta);

    // 1. Deadzone handling
    if (abs_delta <= DEADZONE) {
        g_step_delay_us = 0; // Stop pulse generation
        return;
    }

    // 2. Direction selection
    bool dir = (delta > 0);
    if (INVERT_DIRECTION) dir = !dir;
    g_direction = dir;

    // 3. Throttle ratio calculation (0.0 to 1.0)
    float throttle = (float)(abs_delta - DEADZONE) / (MAX_DELTA_X - DEADZONE);
    if (throttle > 1.0f) throttle = 1.0f;
    if (throttle < 0.0f) throttle = 0.0f;

    // 4. Map throttle to microsecond delay (Higher throttle = shorter delay)
    g_step_delay_us = MAX_DELAY_US - (uint32_t)(throttle * (MAX_DELAY_US - MIN_DELAY_US));
}

void encoder_task(void *pvParameters) {
    static float prev_angle = 0.0f;
    static int64_t prev_time = 0;

    prev_angle = read_as5600_angle();
    prev_time = esp_timer_get_time();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));

        float new_angle = read_as5600_angle();
        int64_t new_time = esp_timer_get_time();

        if (new_angle >= 0.0f) {
            float delta_angle = new_angle - prev_angle;
            float delta_time_sec = (new_time - prev_time) / 1000000.0f;

            if (delta_angle > 180.0f) {
                delta_angle -= 360.0f;
            } else if (delta_angle < -180.0f) {
                delta_angle += 360.0f;
            }

            if (delta_time_sec > 0) {
                current_measured_rpm = (delta_angle / 360.0f) / (delta_time_sec / 60.0f);
            }

            current_measured_angle = new_angle;
            prev_angle = new_angle;
            prev_time = new_time;

            printf("Position: %6.2f Deg | Speed: %6.1f RPM | Delay: %4lu us\n", 
                   current_measured_angle, current_measured_rpm, g_step_delay_us);
        }
    }
}

void onControllerData(const XboxData &data) {
    joy_left_x = data.left_x;
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "=============================================");
    ESP_LOGI(TAG, "   NEMA 17 SMOOTH STEPPER CONTROL READY      ");
    ESP_LOGI(TAG, "=============================================");

    ESP_ERROR_CHECK(init_i2c());
    init_stepper_gpio();

    xbox.onData(onControllerData);
    xbox.begin();

    // Create tasks with sufficient stack
    xTaskCreatePinnedToCore(stepper_task, "stepper_task", 4096, NULL, 10, NULL, 1);
    xTaskCreate(encoder_task, "encoder_task", 4096, NULL, 5, NULL);

    while (1) {
        update_stepper_logic(joy_left_x);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}