//
// Created by trist on 2/26/2024.
//

#ifndef SENSORSUITE_ZOEM8Q_H
#define SENSORSUITE_ZOEM8Q_H
#include "esp_err.h"

//put macros here

#define ZOEM8Q_I2C_ADDRESS 0x42

// Register Definitions
#define REG_CHIP_ID 0x00 // Chip ID
#define REG_VERSION 0x01 // Version
#define REG_CONFIG_DATA 0x1A // Configuration Data
#define REG_LOW_BATTERY_THRESHOLD 0x01 // Low Battery Threshold
#define REG_BATTERY_VOLTAGE 0x09 // Battery Voltage
#define REG_STATUS 0x0A // Status Register
#define REG_LATITUDE_MSB 0x01 // Latitude MSB
#define REG_LATITUDE_CSB 0x02 // Latitude CSB
#define REG_LATITUDE_LSB 0x03 // Latitude LSB
#define REG_LONGITUDE_MSB 0x04 // Longitude MSB
#define REG_LONGITUDE_CSB 0x05 // Longitude CSB
#define REG_LONGITUDE_LSB 0x06 // Longitude LSB
#define REG_SPEED 0x07 // Speed
#define REG_COURSE 0x08 // Course
#define REG_DATE 0x05 // Date
#define REG_YEAR 0x07 // Year
#define REG_MONTH 0x08 // Month
#define REG_DAY 0x09 // Day
#define REG_HOUR 0x0A // Hour
#define REG_MINUTE 0x0B // Minute
#define REG_SECOND 0x0C // Second
#define REG_SATELLITES 0x1F // Satellites
#define REG_ALTITUDE_MSB 0x31 // Altitude MSB
#define REG_ALTITUDE_LSB 0x32 // Altitude LSB
#define REG_TIME_UTC 0x21 // Time UTC
#define REG_TIME_STATUS 0x22 // Time Status
#define REG_SATELLITES_IN_VIEW 0x30 // Satellites in View
#define REG_SATELLITES_IN_USE 0x38 // Satellites in Use
#define REG_FIX_STATUS 0x3D // Fix Status
#define REG_CONFIG_STATUS 0x3E // Config Status
#define REG_FW_VERSION 0x80 // FW Version
#define REG_DEVICE_STATUS 0x90 // Device Status
#define REG_DEVICE_ERROR 0x91 // Device Error
#define REG_TIME_TO_FIRST_FIX 0x92 // Time to First Fix
#define REG_CUSTOM_CONFIG 0x93 // Custom Config

#define I2C_MASTER_SCL_IO  22        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO  21        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM     I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 50000    /*!< I2C master clock frequency */

//actual types and function definitions

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    int satellites;
} gps_data_t;

esp_err_t gps_init(void);
esp_err_t i2c_bus_init(void);
esp_err_t gps_read_data(gps_data_t* data);

#endif //SENSORSUITE_ZOEM8Q_H
