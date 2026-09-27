#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "XboxController.h"

static const char *TAG = "ROTARY_BASE_SYSTEM";

// Hardware connections
#define STEP_PIN         GPIO_NUM_1
#define DIR_PIN          GPIO_NUM_2
#define ENABLE_PIN       GPIO_NUM_3
#define I2C_SDA_PIN      GPIO_NUM_4
#define I2C_SCL_PIN      GPIO_NUM_5

#define I2C_MASTER_NUM   I2C_NUM_0
#define AS5600_ADDR      0x36
#define AS5600_RAW_ANGLE 0x0C
#define SSD1306_ADDR     0x3C

#define MAX_SPEED_DELAY_US 900  // Delay for maximum safe speed

// Step targets
#define STEPS_1_DEG_OUT      25L  // 25 revs
#define STEPS_22_5_DEG_OUT   1125L // 563 revs

XboxController xbox;
SemaphoreHandle_t i2c_mutex = NULL;

// Stepper control variables
volatile int64_t g_remaining_steps = 0;
volatile bool g_stepper_dir = true;

// Shared sensor calibration data
volatile float g_zero_offset = 0.0f;
volatile float g_calibrated_angle = 0.0f;

// Basic 5x7 ASCII Font for SSD1306
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x04, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x7F, 0x20, 0x18, 0x20, 0x7F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}  // Z
};

static uint8_t oled_buffer[1024];

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

void ssd1306_send_cmd(uint8_t cmd) {
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (SSD1306_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(handle, 0x00, true); // Control byte: Command
    i2c_master_write_byte(handle, cmd, true);
    i2c_master_stop(handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(handle);
}

void init_ssd1306(void) {
    ssd1306_send_cmd(0xAE); // Display OFF
    ssd1306_send_cmd(0xD5); // Set Display Clock Divide Ratio
    ssd1306_send_cmd(0x80);
    ssd1306_send_cmd(0xA8); // Set Multiplex Ratio
    ssd1306_send_cmd(0x3F); // 1/64 duty
    ssd1306_send_cmd(0xD3); // Set Display Offset
    ssd1306_send_cmd(0x00);
    ssd1306_send_cmd(0x40); // Set Start Line
    ssd1306_send_cmd(0x8D); // Charge Pump
    ssd1306_send_cmd(0x14); // Enable Charge Pump
    ssd1306_send_cmd(0x20); // Memory Addressing Mode
    ssd1306_send_cmd(0x00); // Horizontal mode
    ssd1306_send_cmd(0xA1); // Segment Re-map
    ssd1306_send_cmd(0xC8); // COM Output Scan Direction
    ssd1306_send_cmd(0xDA); // COM Pins Hardware Configuration
    ssd1306_send_cmd(0x12);
    ssd1306_send_cmd(0x81); // Set Contrast
    ssd1306_send_cmd(0xCF);
    ssd1306_send_cmd(0xD9); // Set Pre-charge Period
    ssd1306_send_cmd(0xF1);
    ssd1306_send_cmd(0xDB); // Set VCOMH Deselect Level
    ssd1306_send_cmd(0x40);
    ssd1306_send_cmd(0xA4); // Entire Display ON resume
    ssd1306_send_cmd(0xA6); // Normal Display
    ssd1306_send_cmd(0xAF); // Display ON
}

void oled_update(void) {
    ssd1306_send_cmd(0x21); // Column Address
    ssd1306_send_cmd(0);
    ssd1306_send_cmd(127);
    ssd1306_send_cmd(0x22); // Page Address
    ssd1306_send_cmd(0);
    ssd1306_send_cmd(7);

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (SSD1306_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(handle, 0x40, true); // Data stream
    i2c_master_write(handle, oled_buffer, sizeof(oled_buffer), true);
    i2c_master_stop(handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, handle, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(handle);
}

void oled_draw_char(int x, int page, char c) {
    if (c < 32 || c > 90) return;
    int index = c - 32;
    for (int i = 0; i < 5; i++) {
        oled_buffer[page * 128 + x + i] = font5x7[index][i];
    }
    oled_buffer[page * 128 + x + 5] = 0x00; // spacing
}

void oled_draw_string(int x, int page, const char *str) {
    while (*str && x < 122) {
        oled_draw_char(x, page, *str);
        x += 6;
        str++;
    }
}

float read_as5600_raw(void) {
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

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(20));
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
    gpio_set_level(ENABLE_PIN, 0); // Driver Enable (Active Low)
}

// Dedicated Core 1 task for step pulse generation
void stepper_task(void *pvParameters) {
    while (1) {
        if (g_remaining_steps > 0) {
            gpio_set_level(DIR_PIN, g_stepper_dir ? 1 : 0);

            // Generate Step Pulse
            gpio_set_level(STEP_PIN, 1);
            esp_rom_delay_us(5);
            gpio_set_level(STEP_PIN, 0);
            esp_rom_delay_us(MAX_SPEED_DELAY_US - 5);

            g_remaining_steps--;

            // Periodic watchdog yield every 100 steps
            if ((g_remaining_steps % 100) == 0) {
                vTaskDelay(1);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// Display & AS5600 Reader Task running on Core 0
void display_encoder_task(void *pvParameters) {
    char str_buf[20];

    while (1) {
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(30)) == pdTRUE) {
            float raw = read_as5600_raw();
            if (raw >= 0.0f) {
                float calibrated = raw - g_zero_offset;
                if (calibrated < 0.0f) calibrated += 360.0f;
                g_calibrated_angle = calibrated;
            }

            // Draw to OLED Buffer
            memset(oled_buffer, 0x00, sizeof(oled_buffer));
            oled_draw_string(10, 0, "ROTARY BASE STATUS");
            oled_draw_string(0, 1, "---------------------");

            snprintf(str_buf, sizeof(str_buf), "POS: %6.2f DEG", g_calibrated_angle);
            oled_draw_string(5, 3, str_buf);

            snprintf(str_buf, sizeof(str_buf), "STEPS: %7lld", (long long)g_remaining_steps);
            oled_draw_string(5, 5, str_buf);

            oled_draw_string(0, 7, "RATIO: 1:45 (WORM)");

            oled_update();
            xSemaphoreGive(i2c_mutex);
        }

        // Screen refresh target (~20 Hz for smooth real-time response)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void onControllerData(const XboxData &data) {
    static bool prev_btn_x = false;
    static bool prev_btn_y = false;
    static bool prev_btn_b = false;
    static bool prev_btn_a = false;
    static bool prev_lt = false;

    // 1. معايرة نقطة الصفر بالزناد الأيسر (LT)
    bool current_lt = (data.trigger_lt > 200);
    if (current_lt && !prev_lt) {
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            float raw = read_as5600_raw();
            if (raw >= 0.0f) {
                g_zero_offset = raw;
                ESP_LOGW(TAG, "New Zero Reference Calibrated: %.2f Deg", g_zero_offset);
            }
            xSemaphoreGive(i2c_mutex);
        }
    }
    prev_lt = current_lt;

    if (data.btn_X && !prev_btn_x) {
        g_stepper_dir = true;
        g_remaining_steps = STEPS_1_DEG_OUT;
    }
    else if (data.btn_Y && !prev_btn_y) {
        g_stepper_dir = false;
        g_remaining_steps = STEPS_1_DEG_OUT;
    }
    else if (data.btn_B && !prev_btn_b) {
        g_stepper_dir = true;
        g_remaining_steps = STEPS_22_5_DEG_OUT;
    }
    else if (data.btn_A && !prev_btn_a) {
        g_stepper_dir = false;
        g_remaining_steps = STEPS_22_5_DEG_OUT;
    }

    prev_btn_x = data.btn_X;
    prev_btn_y = data.btn_Y;
    prev_btn_b = data.btn_B;
    prev_btn_a = data.btn_A;
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "=============================================");
    ESP_LOGI(TAG, "    ROTARY WORM DRIVE & OLED SYSTEM INIT     ");
    ESP_LOGI(TAG, "=============================================");

    i2c_mutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(init_i2c());
    init_ssd1306();
    init_stepper_gpio();

    xbox.onData(onControllerData);
    xbox.begin();

    // Stepper execution locked to Core 1
    xTaskCreatePinnedToCore(stepper_task, "stepper_task", 4096, NULL, 10, NULL, 1);

    // Sensor acquisition & display refresh on Core 0
    xTaskCreatePinnedToCore(display_encoder_task, "oled_encoder_task", 4096, NULL, 5, NULL, 0);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf("Output Angle: %6.2f Deg | Target Steps Left: %lld\n", 
               g_calibrated_angle, (long long)g_remaining_steps);
    }
}