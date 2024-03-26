#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "bme280.h"
#include "icm20948.h"

#include "driver/i2c_master.h"

#include "sdkconfig.h"

#include "freertos/task.h"


#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

#define I2C_MASTER_SCL_IO  22       
#define I2C_MASTER_SDA_IO  21       
#define I2C_MASTER_NUM     I2C_NUM_0 
#define I2C_MASTER_FREQ_HZ 33500    
#define I2C_MASTER_ACK 0 
#define I2C_MASTER_NACK 1

#define TAG_BME280 "BME280"
#define TAG_ICM20948 "ICM20948"

static const char *TAG = "test";

static const char *TAG1 = "gyro test";
static const char *TAG2 = "accel test";

/*GPS STUFF*/
#include "driver/uart.h"

#define TAG "ZOE-M8Q"
#define I2C_MASTER_TX_BUF_DISABLE   0    /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0    /*!< I2C master doesn't need buffer */

#define ZOE_M8Q_I2C_ADDRESS  0x42 

static esp_err_t gps_send_command(uint8_t command[], size_t command_len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ZOE_M8Q_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, command, command_len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t gps_read_data(uint8_t *data, size_t data_len) {
    if (data_len == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ZOE_M8Q_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    if (data_len > 1) {
        i2c_master_read(cmd, data, data_len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + data_len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void gps_task(void *pvParameters) {
    // Initialize UART for serial output
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t gps_data[256]; // Buffer for GPS data

    while (1) {
        memset(gps_data, 0, sizeof(gps_data)); // Clear the buffer before reading
        esp_err_t ret = gps_read_data(gps_data, sizeof(gps_data));
        if (ret == ESP_OK) {
            // Print to UART (Serial)
            uart_write_bytes(UART_NUM_0, (const char *)gps_data, strlen((const char *)gps_data));

            // Optionally, you can print to ESP log as well
            ESP_LOGI("GPS_TASK", "Received GPS data: %s", gps_data);
        } else {
            ESP_LOGE("GPS_TASK", "Failed to read data from GPS module");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for a second before the next read
    }
}
/*GPS STUFF*/

esp_err_t i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK)
        return ret;

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}
