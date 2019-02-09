/*
 *  Project:    MPU6050 to websocket bump sensor and recorder
 *  Date:       30-1-2019
 *  Version:    0.1
 *  Author:     RJM
 *
 */

/* inc standard libraries */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
/* inc freeRTOS libraries */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
/* inc ESP specific libraries */
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
/* inc esp lwip libraries */
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
/* inc esp peripheral libraries */
#include "nvs_flash.h"
#include "driver/i2c.h"

/******* Private defines ***********/
/* Wifi/server config */
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASS
#define SERVER_ADDR CONFIG_SERVER_ADDR
#define SERVER_PORT CONFIG_SERVER_PORT
/* i2c config */
#define I2C_MASTER_SCL_IO           2                /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           14               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define WRITE_BIT                   I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                    I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS               0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                     0x0              /*!< I2C ack value */
#define NACK_VAL                    0x1              /*!< I2C nack value */
#define LAST_NACK_VAL               0x2              /*!< I2C last_nack value */

/* MPU6050 addresses*/
#define MPU6050_SENSOR_ADDR     0x68
#define MPU6050_CMD_START       0x41
#define MPU6050_WHO_AM_I        0x75
#define SMPLRT_DIV              0x19
#define CONFIG                  0x1A
#define GYRO_CONFIG             0x1B
#define ACCEL_CONFIG            0x1C
#define ACCEL_XOUT_H            0x3B
#define ACCEL_XOUT_L            0x3C
#define ACCEL_YOUT_H            0x3D
#define ACCEL_YOUT_L            0x3E
#define ACCEL_ZOUT_H            0x3F
#define ACCEL_ZOUT_L            0x40
#define TEMP_OUT_H              0x41
#define TEMP_OUT_L              0x42
#define GYRO_XOUT_H             0x43
#define GYRO_XOUT_L             0x44
#define GYRO_YOUT_H             0x45
#define GYRO_YOUT_L             0x46
#define GYRO_ZOUT_H             0x47
#define GYRO_ZOUT_L             0x48
#define PWR_MGMT_1              0x6B
#define WHO_AM_I                0x75


#define DATA_LEN (sizeof(uint16_t)*7 + sizeof(double))

/*
 * function prototypes
 */


static esp_err_t i2c_config();  /* begin & config the i2c peripheral */
static esp_err_t mpu_config();  /* begin & config the MPU6050 device*/
static void wifi_config();      /* begin & config the WIFI connection */
static void tcp_config();       /* begin & configure TCP connection -== DEPRACATED=- */
static void check_tcp();       /* check / re-establish tcp connection */
static void connect_to_server();/* begin connection to the server */
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event);  /* event handler for connection events */
static esp_err_t i2c_write_to_slave(i2c_port_t port, uint8_t device, uint8_t reg_addr, uint8_t *data, size_t data_len);
static esp_err_t i2c_read_from_slave(i2c_port_t port, uint8_t device, uint8_t reg_add, uint8_t *buffer, uint8_t len);
static void mpu_sample(void *args); /* Task: get values from MPU6050 */
static void transmit_sample(void *args); /* Task: send samples to server over tcp */
//static uint16_t delta(MPU6050 data);


 /* MPU6050 data holder */
 typedef struct  {
 	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
 	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
 	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
 	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
 	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
 	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
 	double Temperature;       /*!< Temperature in degrees */
 } MPU6050;

/* data transfer struct + ID */
typedef struct {
    MPU6050 data;
    uint16_t id;
    double timestamp;
    double delta_t;
} MPU_DATA;

/* Variable declaration */

static const char *MAIN_TAG = "main";
static const char *EVENT_TAG = "[EVENT]";
static const char *TCP_TAG = "[TCP_SETUP]";
static const char *DEBUG = "@@@@@";
static const char *HEARTBEAT = "HB";


static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
int sockfd;
uint8_t CONNECTED_STATUS = 0;
uint16_t id = 0;
QueueHandle_t qHandle;
uint32_t time_now, time_last=0;


/* Functions */



static void tcp_config(){
    uint32_t event_bits = CONNECTED_BIT;
    // wait for bit(s) to be set in event group //
    xEventGroupWaitBits(wifi_event_group, event_bits, false, true, portMAX_DELAY);
    ESP_LOGI(EVENT_TAG, "GOT IP");
    //now open a socket on the device and connect to the server //
    connect_to_server();

}

/* using the damn esp32 sdk again...
static void timer_config(){

    timer_config_t timer;
    timer.alarm_en = 0;
    timer.counter_en = 1;
    timer.counter_dir = TIMER_COUNT_UP;
    timer.divider = 128;

    timer_init(TIMER_GROUP_0, TIMER_0, );

}*/

static void check_tcp(){

    int tcp_check = send(sockfd, HEARTBEAT, strlen(HEARTBEAT), 0);
    if(tcp_check < 0) {
        close(sockfd);
        CONNECTED_STATUS = 0;
        ESP_LOGI(TCP_TAG, "Restarting TCP socket.\n");
        connect_to_server();
    }
}


static void connect_to_server(){

    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    struct sockaddr_in target;
    target.sin_addr.s_addr = inet_addr(SERVER_ADDR);
    target.sin_family = AF_INET;
    target.sin_port = htons(SERVER_PORT);

    /* create socket file descriptor */
    sockfd = socket(addr_family, SOCK_STREAM, ip_protocol);
    if(sockfd < 0) {
        ESP_LOGI(TCP_TAG, "Unable to create socket" ); }
    else {
        ESP_LOGI(TCP_TAG, "Socket created!\n"); }

    /* connect to the server */
    int err = connect(sockfd, (struct sockaddr *)&target, sizeof(target));
    if(err != 0) {
        ESP_LOGI(TCP_TAG, "Error in connecting to socket...\n"); }
    else {
        ESP_LOGI(TCP_TAG, "Connected to socket\n");
        CONNECTED_STATUS = 1; }

}


static void wifi_config(){

        tcpip_adapter_init();
        /* event group to track wifi events */
        wifi_event_group = xEventGroupCreate();
        ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL));
        /* initialise wifi as wifi default */
        wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_LOGI(EVENT_TAG, "Connecting to AP...");
        /* Wifi config details */
        wifi_config_t wifi_config= {
                .sta = { .ssid = WIFI_SSID,
                         .password = WIFI_PASS, },
        };
        /* config and start wifi */
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA,  &wifi_config));
        ESP_LOGI(EVENT_TAG, "Starting Wifi interface");
        ESP_ERROR_CHECK(esp_wifi_start());
}


static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    /* callback to switch handle wifi events */
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(EVENT_TAG, "Station started");
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        ESP_LOGI(EVENT_TAG, "Got IP");
        tcp_config();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        ESP_LOGI(EVENT_TAG, "Disonnected");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        CONNECTED_STATUS = 0;
        break;
    default:
        break;
    }
    return ESP_OK;
}

static esp_err_t mpu_config(){
    /* configure the MPU6050 settings, see below for details */
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
    /* configure I2C as master */

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
    /* Write data to slave over I2C */

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    /* build transaction chain */
    i2c_master_start(handle);
    i2c_master_write_byte(handle, device << 1| I2C_MASTER_WRITE, ACK_CHECK_EN); // write device and register address to bus
    i2c_master_write_byte(handle, reg_addr, ACK_CHECK_EN);
    if(data_len > 0) { i2c_master_write(handle, data, data_len, ACK_CHECK_EN); } // if there is data to write, send it now
    i2c_master_stop(handle);
    /* begin the transaction */
    esp_err_t ret = i2c_master_cmd_begin(port, handle, 1000 / portTICK_RATE_MS); // send commands
    i2c_cmd_link_delete(handle);

    return ret;
}

static esp_err_t i2c_read_from_slave(i2c_port_t port, uint8_t device, uint8_t reg_addr, uint8_t *buffer, uint8_t len){
    /* Read data from slave over I2C */

    esp_err_t ret;
    uint8_t dummy=0;
    /* write register address to the slave */
    i2c_write_to_slave(port, device, reg_addr, &dummy, 0);
    /* build read transaction */
    i2c_cmd_handle_t handle = i2c_cmd_link_create();  // start read handle
    i2c_master_start(handle);
    i2c_master_write_byte(handle, device << 1|I2C_MASTER_READ, ACK_CHECK_EN); // write READ command to device
    i2c_master_read(handle, buffer, len, LAST_NACK_VAL);              // read into buffer
    i2c_master_stop(handle);      // send stop bit
    /* begin transaction */
    ret = i2c_master_cmd_begin(port, handle, 1000 / portTICK_RATE_MS);  //  send commands
    i2c_cmd_link_delete(handle);

    return ret;
}


static void mpu_sample(void *args){
    /* read data from accelerometer */
    uint8_t sensor_data[14];
    MPU_DATA data;
    MPU6050 mpu;

    while (1) {
        /* loop forever */
        if(uxQueueMessagesWaiting(qHandle)) { vTaskDelay(100/portTICK_PERIOD_MS); }
        else{
            memset(sensor_data, 0, 14);
            esp_err_t ret = i2c_read_from_slave(I2C_NUM_0, MPU6050_SENSOR_ADDR, ACCEL_XOUT_H, sensor_data, 14);
            if (ret == ESP_OK) {
            /* increment id number */
            id++;
            /* fill data struct with data */
            mpu.Accelerometer_X = (uint16_t) sensor_data[0] << 8| sensor_data[1];
            mpu.Accelerometer_Y = (uint16_t) sensor_data[2] << 8| sensor_data[3];
            mpu.Accelerometer_Z = (uint16_t) sensor_data[4] << 8| sensor_data[5];
            mpu.Temperature = 36.53 + ((double)(int16_t)((sensor_data[6] << 8) | sensor_data[7]) / 340);  // check this... double or float????
            mpu.Gyroscope_X = (uint16_t) sensor_data[8] << 8| sensor_data[9];
            mpu.Gyroscope_Y = (uint16_t) sensor_data[10] << 8| sensor_data[11];
            mpu.Gyroscope_Z = (uint16_t) sensor_data[12] << 8| sensor_data[13];
            /* log data to console */
            ESP_LOGI(MAIN_TAG, "Accel X: %d\tAccel Y: %d\tAccel Z: %d\tGyros X: %d\tGyros Y: %d\tGyros Z: %d", mpu.Accelerometer_X, mpu.Accelerometer_Y, mpu.Accelerometer_Z, mpu.Gyroscope_X,mpu.Gyroscope_Y, mpu.Gyroscope_Z);
            } else {
                ESP_LOGI(MAIN_TAG, "Error in reading from MPU\n");
            }
            /*  build data to send to queue - maybe redundant to do both mpu and mpu_data */
            time_now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            data.data = mpu;
            data.id = id;
            /* make a timestamp and delta t for change over time */
            data.timestamp = time_now;
            data.delta_t = time_now - time_last;
            time_last = time_now;
            /* send data to queue - block until last data is sent */
            xQueueSendToBack(qHandle, (void *)&data, portMAX_DELAY);
            }
    }

    i2c_driver_delete(I2C_MASTER_NUM);
}


static void transmit_sample(void *args){
    /* send data from struct over TCP to server */
    void *payload;
    char rx_buffer[56];
    MPU_DATA data;

    while(1){
        /* loop forever */
        /* wait on data from queue */
        if(!(uxQueueMessagesWaiting(qHandle)) || !CONNECTED_STATUS){
             vTaskDelay(100/portTICK_PERIOD_MS); }
        else{
            xQueueReceive(qHandle, (void *)&data, portMAX_DELAY);
            payload = &data;

            /* send data over socket */
            int sent = send(sockfd, payload, DATA_LEN, 0);
            if(sent < 0) {
                ESP_LOGI(TCP_TAG, "Error in sending data\n");
                check_tcp();
            }
            else if(sent > DATA_LEN){
                ESP_LOGI(TCP_TAG, "Sent more than expected!\n");
            }
                else {
                    ESP_LOGI(TCP_TAG, "Packet sent.\n"); }
            }
        /*
        int recvd = recv(sockfd, rx_buffer, sizeof(rx_buffer)-1, 0);
        if(recvd < 0) { ESP_LOGI(TCP_TAG, "Error in     recieving data\n"); }
        else{
            rx_buffer[recvd] = 0;
            ESP_LOGI(TCP_TAG, "Received %d bytes: [%s]", recvd, rx_buffer);
        }
        */

    }

}


void app_main(void){


    i2c_config();
    mpu_config();
    wifi_config();

    /* create a queue for the data */
    qHandle = xQueueCreate(1, sizeof(MPU_DATA));
    if(qHandle != NULL){
        /* create tasks to run */
        xTaskCreate(transmit_sample, "transmit", 5012, NULL, 2, NULL);
        xTaskCreate(mpu_sample, "sample", 5012, NULL, 2, NULL);
    }
    else{
        ESP_LOGI(MAIN_TAG, "Error in establishing the queue.");
        while(1){
            /* here to catch errors */
        }
     }


}
