/*
 *  Project:    MPU6050 to websocket bump sensor and recorder
 *  Date:       30-1-2019
 *  Version:    0.1
 *  Author:     RJM
 *
 */


 #include <stdio.h>
 #include <string.h>
 #include <stdlib.h>

 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"

 #include "esp_log.h"
 #include "esp_system.h"
 #include "esp_err.h"

 #include "driver/i2c.h"


 static const char *TAG = "main";

 #define I2C_EXAMPLE_MASTER_SCL_IO           2                /*!< gpio number for I2C master clock */
 #define I2C_EXAMPLE_MASTER_SDA_IO           14               /*!< gpio number for I2C master data  */
 #define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
 #define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
 #define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

 #define MPU6050_SENSOR_ADDR                 0x68             /*!< slave address for MPU6050 sensor */
 #define MPU6050_CMD_START                   0x41             /*!< Command to set measure mode */
 #define MPU6050_WHO_AM_I                    0x75             /*!< Command to read WHO_AM_I reg */
 #define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
 #define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
 #define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
 #define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
 #define ACK_VAL                             0x0              /*!< I2C ack value */
 #define NACK_VAL                            0x1              /*!< I2C nack value */
 #define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

 /**
  * Define the mpu6050 register address:
  */
 #define SMPLRT_DIV      0x19
 #define CONFIG          0x1A
 #define GYRO_CONFIG     0x1B
 #define ACCEL_CONFIG    0x1C
 #define ACCEL_XOUT_H    0x3B
 #define ACCEL_XOUT_L    0x3C
 #define ACCEL_YOUT_H    0x3D
 #define ACCEL_YOUT_L    0x3E
 #define ACCEL_ZOUT_H    0x3F
 #define ACCEL_ZOUT_L    0x40
 #define TEMP_OUT_H      0x41
 #define TEMP_OUT_L      0x42
 #define GYRO_XOUT_H     0x43
 #define GYRO_XOUT_L     0x44
 #define GYRO_YOUT_H     0x45
 #define GYRO_YOUT_L     0x46
 #define GYRO_ZOUT_H     0x47
 #define GYRO_ZOUT_L     0x48
 #define PWR_MGMT_1      0x6B
 #define WHO_AM_I        0x75

/*
 * function prototypes
 */

static esp_err_t i2c_config();
static esp_err_t mpu_config();
static esp_err_t i2c_write_to_slave(i2c_port_t port, uint8_t device, uint8_t reg_addr, uint8_t *data, size_t data_len);
static esp_err_t i2c_read_from_slave(i2c_port_t port, uint8_t device, uint8_t reg_add, uint8_t *buffer, uint8_t len);
static void mpu_sample(void *args);


 /*
  * Struct for MPU data
  */
 typedef struct  {
 	/* Private */
 	uint8_t Address;         /*!< I2C address of device. */
 	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
 	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
 	/* Public */
 	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
 	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
 	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
 	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
 	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
 	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
 	float   Temperature;       /*!< Temperature in degrees */
 	//I2C_HandleTypeDef* I2Cx;
 } MPU6050;

MPU6050 mpu;


static esp_err_t mpu_config(){

    uint8_t command;

    command = 0x00;     // turn the device on
    ESP_ERROR_CHECK(i2c_write_to_slave(I2C_NUM_0, MPU6050_SENSOR_ADDR, PWR_MGMT_1, &command, 1));
    command = 0x00;     // set samplerate to highest
    ESP_ERROR_CHECK(i2c_write_to_slave(I2C_NUM_0, MPU6050_SENSOR_ADDR, SMPLRT_DIV, &command, 1));
    command = 0x04;     // set lowpass filter to 20Hz
    ESP_ERROR_CHECK(i2c_write_to_slave(I2C_NUM_0, MPU6050_SENSOR_ADDR, CONFIG, &command, 1));
    command = 0x10;     // set gyro range to 1000 d/s-1
    ESP_ERROR_CHECK(i2c_write_to_slave(I2C_NUM_0, MPU6050_SENSOR_ADDR, GYRO_CONFIG, &command, 1));
    command = 0x8;      // set accel range to 4g/s-1
    ESP_ERROR_CHECK(i2c_write_to_slave(I2C_NUM_0, MPU6050_SENSOR_ADDR, ACCEL_CONFIG, &command, 1));

    return ESP_OK;
}

static esp_err_t i2c_config(){

     i2c_config_t config;

     config.mode = I2C_MODE_MASTER;
     config.sda_io_num = GPIO_NUM_14;
     config.scl_io_num = GPIO_NUM_2;
     config.sda_pullup_en = 0;
     config.scl_pullup_en = 0;
     ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, config.mode));
     ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &config));

     return ESP_OK;

 }

static esp_err_t i2c_write_to_slave(i2c_port_t port, uint8_t device, uint8_t reg_addr, uint8_t *data, size_t data_len) {
    // write data to the slave

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, device << 1| I2C_MASTER_WRITE, ACK_CHECK_EN); // write device and register address to bus
    i2c_master_write_byte(handle, reg_addr, ACK_CHECK_EN);
    if(data_len > 0) { i2c_master_write(handle, data, data_len, ACK_CHECK_EN); } // if there is data to write, send it now
    i2c_master_stop(handle);
    esp_err_t ret = i2c_master_cmd_begin(port, handle, 1000 / portTICK_RATE_MS); // send commands
    i2c_cmd_link_delete(handle); // delete handle, free resources

    return ret;
}

static esp_err_t i2c_read_from_slave(i2c_port_t port, uint8_t device, uint8_t reg_addr, uint8_t *buffer, uint8_t len){

    esp_err_t ret;
    uint8_t dummy=0;  // dummy data
    i2c_write_to_slave(port, device, reg_addr, &dummy, 0); // write register address to the slave

    i2c_cmd_handle_t handle = i2c_cmd_link_create();  // start read handle
    i2c_master_start(handle);
    i2c_master_write_byte(handle, device << 1|I2C_MASTER_READ, ACK_CHECK_EN); // write READ command to device
    i2c_master_read(handle, buffer, len, LAST_NACK_VAL);              // read into buffer
    i2c_master_stop(handle);      // send stop bit
    ret = i2c_master_cmd_begin(port, handle, 1000 / portTICK_RATE_MS);  //  send commands

    i2c_cmd_link_delete(handle);    // free resources

    return ret;
}


static void mpu_sample(void *args){

    uint8_t sensor_data[14];

    while (1) {

        memset(sensor_data, 0, 14);
        esp_err_t ret = i2c_read_from_slave(I2C_NUM_0, MPU6050_SENSOR_ADDR, ACCEL_XOUT_H, sensor_data, 14);

        if (ret == ESP_OK) {
        mpu.Accelerometer_X = (uint16_t) sensor_data[0] << 8| sensor_data[1];
        mpu.Accelerometer_Y = (uint16_t) sensor_data[2] << 8| sensor_data[3];
        mpu.Accelerometer_Z = (uint16_t) sensor_data[4] << 8| sensor_data[5];
        mpu.Temperature = 36.53 + ((double)(int16_t)((sensor_data[6] << 8) | sensor_data[7]) / 340);
        mpu.Gyroscope_X = (uint16_t) sensor_data[8] << 8| sensor_data[9];
        mpu.Gyroscope_Y = (uint16_t) sensor_data[10] << 8| sensor_data[11];
        mpu.Gyroscope_Z = (uint16_t) sensor_data[12] << 8| sensor_data[13];
        ESP_LOGI(TAG, "Accel X: %d\tAccel Y: %d\tAccel Z: %d\tGyros X: %d\tGyros Y: %d\tGyros Z: %d", mpu.Accelerometer_X, mpu.Accelerometer_Y, mpu.Accelerometer_Z, mpu.Gyroscope_X,mpu.Gyroscope_Y, mpu.Gyroscope_Z);

        } else {
            ESP_LOGE(TAG, "Error in reading from MPU\n");
        }

        vTaskDelay(100 / portTICK_RATE_MS);
    }

    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}


void app_main(void){

    i2c_config();
    for(int i=0; i<50000; i++) { ; }
    mpu_config();

    xTaskCreate(mpu_sample, "sample", 5012, NULL, 2, NULL);
}

