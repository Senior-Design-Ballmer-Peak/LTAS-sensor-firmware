#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "bme280.h"
#include "icm20948.h"
#include "rf69.h"


#define I2C_MASTER_SCL_IO  22        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO  21        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM     I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 33500    /*!< I2C master clock frequency */
#define I2C_MASTER_ACK 0 /**/
#define I2C_MASTER_NACK 1 /**/

#define PIN_NUM_MOSI 23 /**/
#define PIN_NUM_MISO 19 /**/
#define PIN_NUM_CLK 18 /**/
#define SPI_DMA_CHAN_NONE 0 /**/


#define TAG_BME280 "BME280" /**/
#define TAG_ICM20948 "ICM20948" /**/
#define TAG_RF69 "RF69" /**/

static const char *TAG = "test";

static const char *TAG1 = "gyro test";
static const char *TAG2 = "accel test";
static icm20948_handle_t icm20948 = NULL;


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

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;

	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = FAIL;
	}
	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

void BME280_delay_msek(u32 msek)
{
	vTaskDelay(msek/portTICK_PERIOD_MS);
}

void task_bme280_normal_mode(void *ignore)
{
	struct bme280_t bme280 = {
		.bus_write = BME280_I2C_bus_write,
		.bus_read = BME280_I2C_bus_read,
		.dev_addr = BME280_I2C_ADDRESS2,
		.delay_msec = BME280_delay_msek
	};

	s32 com_rslt;
	s32 v_uncomp_pressure_s32;
	s32 v_uncomp_temperature_s32;
	s32 v_uncomp_humidity_s32;

	com_rslt = bme280_init(&bme280);

	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
	if (com_rslt == SUCCESS) {
		while(true) {
			vTaskDelay(40/portTICK_PERIOD_MS);

			com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
				&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

			if (com_rslt == SUCCESS) {
				ESP_LOGI(TAG_BME280, "%.2f degC / %.3f hPa / %.3f %%",
					bme280_compensate_temperature_double(v_uncomp_temperature_s32),
					bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100, // Pa -> hPa
					bme280_compensate_humidity_double(v_uncomp_humidity_s32));
			} else {
				ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
			}
		}
	} else {
		ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
	}

	vTaskDelete(NULL);
}



static esp_err_t
icm20948_configure(icm20948_acce_fs_t acce_fs, icm20948_gyro_fs_t gyro_fs)
{
	esp_err_t ret;

	/*
	 * One might need to change ICM20948_I2C_ADDRESS to ICM20948_I2C_ADDRESS_1
	 * if address pin pulled low (to GND)
	 */
	icm20948 = icm20948_create(I2C_MASTER_NUM, ICM20948_I2C_ADDRESS);
	if (icm20948 == NULL) {
		ESP_LOGE(TAG, "ICM20948 create returned NULL!");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG, "ICM20948 creation successfull!");

	ret = icm20948_reset(icm20948);
	if (ret != ESP_OK){
		ESP_LOGE(TAG, "reset failed");
		return ret;
	}
		

	vTaskDelay(10 / portTICK_PERIOD_MS);

	ret = icm20948_wake_up(icm20948);
	if (ret != ESP_OK){
		ESP_LOGE(TAG, "wake up failed");
		return ret;
	}
		

	ret = icm20948_set_bank(icm20948, 0);
	if (ret != ESP_OK){
		ESP_LOGE(TAG, "set bank failed");
		return ret;
	}
		

	uint8_t device_id;
	ret = icm20948_get_deviceid(icm20948, &device_id);
	if (ret != ESP_OK){
		ESP_LOGE(TAG, "get deviceid failed");
		return ret;
	}
		
	ESP_LOGI(TAG, "0x%02X", device_id);
	if (device_id != ICM20948_WHO_AM_I_VAL){
		ESP_LOGE(TAG, "incorrect device_id");
		return ESP_FAIL;
	}
		

	ret = icm20948_set_gyro_fs(icm20948, gyro_fs);
	if (ret != ESP_OK){
		ESP_LOGE(TAG, "set gyro failed");
		return ESP_FAIL;
	}
		

	ret = icm20948_set_acce_fs(icm20948, acce_fs);
	if (ret != ESP_OK){
		ESP_LOGE(TAG, "set accel failed");
		return ESP_FAIL;
	}
		

	return ret;
}

void
icm_read_task(void *args)
{
	esp_err_t ret = icm20948_configure(ACCE_FS_2G, GYRO_FS_250DPS);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "ICM configuration failure");
		vTaskDelete(NULL);
	}
	ESP_LOGI(TAG, "ICM20948 configuration successfull!");

	icm20948_acce_value_t acce;
	icm20948_gyro_value_t gyro;
	for (int i = 0; i < 1000; ++i) {
		ret = icm20948_get_acce(icm20948, &acce);
		if (ret == ESP_OK)
			ESP_LOGI(TAG2, "ax: %lf ay: %lf az: %lf", acce.acce_x, acce.acce_y, acce.acce_z);
		ret = icm20948_get_gyro(icm20948, &gyro);
		if (ret == ESP_OK)
			ESP_LOGI(TAG1, "gx: %lf gy: %lf gz: %lf", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}



esp_err_t init_spi_bus(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI, // Specify your pin numbers here
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    // Initialize the SPI bus
    return spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CHAN_NONE);
}

void tx_task(void *pvParameter)
{
	ESP_LOGI(pcTaskGetName(0), "Start");
	int packetnum = 0;	// packet counter, we increment per xmission
	while(1) {

		char radiopacket[64] = "Hello World #";
		sprintf(radiopacket, "Hello World #%d", packetnum++);
		ESP_LOGI(pcTaskGetName(0), "Sending %s", radiopacket);
  
		// Send a message!
		send((uint8_t *)radiopacket, strlen(radiopacket));
		waitPacketSent();

		// Now wait for a reply
		uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
		uint8_t len = sizeof(buf);

		if (waitAvailableTimeout(500))	{
			// Should be a reply message for us now   
			if (recv(buf, &len)) {
				ESP_LOGI(pcTaskGetName(0), "Got a reply: %s", (char*)buf);
			} else {
				ESP_LOGE(pcTaskGetName(0), "Receive failed");
			}
		} else {
			ESP_LOGE(pcTaskGetName(0), "No reply, is another RFM69 listening?");
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	} // end while

	// never reach here
	vTaskDelete( NULL );
}

void app_main(void)
{
    ESP_LOGI(TAG, "App main started");

    // Initialize I2C bus
    esp_err_t i2c = i2c_bus_init();
    if (i2c != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus initialization failed: %s", esp_err_to_name(i2c));
        return;
    }
    ESP_LOGI(TAG, "I2C bus initialization successful");

    // Initialize RFM69 radio
    if (!init()) {
        ESP_LOGE(TAG, "RFM69 radio initialization failed");
        return;
    }
    ESP_LOGI(TAG, "RFM69 radio initialization successful");

    // Set RFM69 radio frequency
    float freq = 915.0;
    ESP_LOGI(TAG, "Setting frequency to %.1fMHz", freq);
    if (!setFrequency(freq)) {
        ESP_LOGE(TAG, "Failed to set frequency");
        return;
    }
    ESP_LOGI(TAG, "Frequency set successfully");

    // Create tasks with optimized priorities
    xTaskCreate(&tx_task, "tx_task", 2048, NULL, 5, NULL);
    //xTaskCreate(icm_read_task, "icm_read_task", 2048, NULL, 4, NULL);
    //xTaskCreate(&task_bme280_normal_mode, "bme280_normal_mode", 2048, NULL, 3, NULL);
}