//
// Created by trist on 2/26/2024.
//

#ifndef SENSORSUITE_ZOEM8Q_H
#define SENSORSUITE_ZOEM8Q_H

//put macros here

#define ZOEM8Q_I2C_ADDRESS 0x42
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
