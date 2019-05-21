/*
 *  Project:    MPU6050 to tcp/websocket bump sensor and recorder
 *  Date:       30-1-2019
 *  Version:    0.4
 *  Author:     RJM
 *  Platform:   ESP8266_RTOS_SDK
 *  IDE:        Atom inc. PlatformIO
 *  Compiler:   xtensa-lx106-elf

    // TODO:         Implement functions to take averages/change over time, use LED pwm to fade led based on bumps,
                        Implement the advanced httpd which includes websocket support
 *
 *   //               Implement a webpage using websockets, display data as real-time graphs.
    // v0.1 Notes: Established working mpu6050, read/write i2c, wifi config & tcp connections
    //                   Succesfully streamed data to server running a crappy python recv. program

 *
 *  // v0.2 Notes: Although the tcp data stream was easy enough to implement, It would take silly amounts of time to implement a full http server
 *  //              so decided to use the lwip implementation of httpd. Could have used the lwip for the tcp stream too, but didn't. Maybe fix in future
 *  //              probably not though.
 *
 *  /// v0.3 notes: Plodding towards functionality. Problems implementing web server. Fixed by copying LWIP's httpd files to local folder`
 *  ///             Also problems with the PWM drivers in the core ESP8266_RTOS_SDK. Fixed by changing declaration of pwm_init() in pwm.c and pwm.h
 *  ///             --> Started a github issue describing problem and fix. Don't know how to request a merge yet. Interesting first.
 *
 *  /// v0.4 notes: After much struggle the web page is active (although not currently functional...). REPEATED problems with the makefsdata utility.
 *  ///             Needs to build fs elsewhere, then import the fsdata.c to the folder (but not main), along with lots of other lwip httpd stuff
 *  ///             Also, must add missing 'flags' variables to the pages. However, is now working. Now to find out how to implement websockets correctly.
 *  ///             Also, submitted an issue on github, was resolved and added to the chain :)
 *  ///
 *  ///             ok, got the httpd with websockets version compiled and working - now to check examples and implement some WS action
 *                  awesome, websocket sending data to the client. Only 1 directional for now.
 *                  NOO TODO:   + send a complete JSON style string of accelerometer data to the client
 *                              + graph it
 *                              + add filter functions (look into Kalman filters)
 *                              + research drop/knock interrupts on mpu6050
 *
 *
 */

/* inc standard libraries */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include <inttypes.h>
/* inc freeRTOS libraries */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
/* inc ESP specific libraries */
#include "esp_event_loop.h"
#include "driver/pwm.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
// test //
#include "test.h"

/* inc esp lwip libraries */
#include "lwip/init.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
/* lwip httpd files included locally */
//#include "fsdata.c"
//#include "fsdata.h"
#include "httpd.h"
#include "httpd_opts.h"

//#include "lwip/apps/httpd_opts.h"
/* inc esp peripheral libraries */
#include "nvs_flash.h"
#include "driver/i2c.h"

/******* Defines ***********/

#define MODE                        2         // mode 1 = tcp server, mode 2 = webserver

/* Wifi/server config */
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASS
#define SERVER_ADDR CONFIG_SERVER_ADDR
#define SERVER_PORT CONFIG_SERVER_PORT
/* i2c config */
#define I2C_MASTER_SCL_IO           4                /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           5               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define WRITE_BIT                   I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                    I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS               0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                     0x0              /*!< I2C ack value */
#define NACK_VAL                    0x1              /*!< I2C nack value */
#define LAST_NACK_VAL               0x2              /*!< I2C last_nack value */

#define LED_PIN                    GPIO_Pin_0
#define THRESHOLD                  100

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


#define DATA_LEN ((sizeof(uint16_t)*7) + sizeof(float) + (sizeof(uint32_t) * 2))

/* MPU6050 data holder */
typedef struct  {
   int16_t Accelerometer_X; /*!< Accelerometer value X axis */
   int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
   int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
   int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
   int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
   int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
   float Temperature;       /*!< Temperature in degrees */
} MPU6050;

/* data transfer struct + ID */
typedef struct {
   MPU6050 data;
   uint16_t id;
   uint32_t timestamp;
   uint32_t delta_t;
} MPU_DATA;

typedef struct{
   int16_t deltaAX;
   int16_t deltaAY;
   int16_t deltaAZ;
   int16_t deltaGX;
   int16_t deltaGY;
   int16_t deltaGZ;
   float deltaTemp;
   uint32_t deltaTime;
} MPU_DELTA;

typedef struct{
    int16_t test_data;
    int16_t time;

} MPU_WS;

/*
 * function prototypes
 */

static void start_httpd();
static void http_config();
static esp_err_t pwm_config();  /* begin led pwm_init*/
static esp_err_t i2c_config();  /* begin & config the i2c peripheral */
static esp_err_t mpu_config();  /* begin & config the MPU6050 device*/
static void wifi_config();      /* begin & config the WIFI connection */
static void tcp_config();       /* begin & configure TCP connection -== DEPRACATED=- */
static void check_tcp();       /* check / re-establish tcp connection */
static void connect_to_server();/* begin connection to the server */
static void ws_open_callback(struct tcp_pcb *tcp, const char *uri);
static void ws_recv_cb(struct tcp_pcb *tcp, uint8_t data, uint16_t data_len, uint8_t mode);
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event);  /* event handler for connection events */
static esp_err_t i2c_write_to_slave(i2c_port_t port, uint8_t device, uint8_t reg_addr, uint8_t *data, size_t data_len);
static esp_err_t i2c_read_from_slave(i2c_port_t port, uint8_t device, uint8_t reg_add, uint8_t *buffer, uint8_t len);
static void mpu_sample(void *args); /* Task: get values from MPU6050 */
static void transmit_sample(void *args); /* Task: send samples to server over tcp */
static void manage_led(void *args);
static void websocket_serve(void *args);
static void append_to_results(MPU_DATA *data);
static void get_delta(void *args);
//static uint16_t delta(MPU6050 data);





/************ Global Variables ********************/

static const char *MAIN_TAG = "main";
static const char *EVENT_TAG = "[EVENT]";
static const char *TCP_TAG = "[TCP_SETUP]";
static const char *DEBUG = "@@@@@";
static const char *HEARTBEAT = "[HEART]";
static const char *WS_TAG = "[WEBSOCKET]";


static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
int sockfd_0, sockfd_1;
uint8_t CONNECTED_STATUS = 0;
uint8_t REQUEST = 1;
uint16_t id = 0;
QueueHandle_t qHandle;
uint32_t time_now, time_last=0;
int r_index=0;
MPU6050 *current_result;
MPU_DATA *results[10];



/********************** Interrupts & Events ******************************/

// wifi events handles connection/disconnection events
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event){
    /* callback to switch handle wifi events */
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(EVENT_TAG, "Station started");
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        ESP_LOGI(EVENT_TAG, "Got IP");
        CONNECTED_STATUS = 1;
        ESP_LOGI(EVENT_TAG, "connected = %d", CONNECTED_STATUS);
        if(MODE == 1){ ESP_LOGI(TCP_TAG, "TCP->SERVER mode selected.\n"); tcp_config();   }
        else if(MODE == 2){ ESP_LOGI(TCP_TAG, "WEBSERVER mode selected.\n"); http_config();  }
        else { ESP_LOGI(TCP_TAG, "No mode selected.\n");}
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

/********************** Config & Startup *********************************/

// config & init the wifi connection
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

// config & init the TCP connection to server
static void tcp_config(){
    uint32_t event_bits = CONNECTED_BIT;
    // wait for bit(s) to be set in event group //
    xEventGroupWaitBits(wifi_event_group, event_bits, false, true, portMAX_DELAY);
    ESP_LOGI(EVENT_TAG, "GOT IP");
    //now open a socket on the device and connect to the server //
    connect_to_server();

}

// config & init the MPU6050 accelerometer
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

// configure the i2c driver
static esp_err_t i2c_config(){
    /* configure I2C as master */

     i2c_config_t config;
     config.mode = I2C_MODE_MASTER;
     config.sda_io_num = I2C_MASTER_SDA_IO;
     config.scl_io_num = I2C_MASTER_SCL_IO;
     config.sda_pullup_en = 0;
     config.scl_pullup_en = 0;
     ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, config.mode));
     ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &config));

     return ESP_OK;

 }

// configure the pwm driver for leds
static esp_err_t pwm_config(){
     // ok an actual control over the led pwm...
     // 5kHz, duty=0, pwm__channel=1, pin_num = 0
     uint32_t pin = GPIO_NUM_0;
     uint32_t duty = 0;
     //uint16_t phase[1]={ 0 };
     uint8_t channels[1]={ 1 };
     if(pwm_init(1000, &duty, 1, &pin) != ESP_OK) { ESP_LOGI(MAIN_TAG, "Error in intialising pwm\n"); }
     if(pwm_set_phase(0, 0) != ESP_OK) { ESP_LOGI(MAIN_TAG, "Error in setting phase\n"); }
     //if(pwm_set_channel_invert(0) != ESP_OK) { ESP_LOGI(MAIN_TAG, "Error in setting INVERSIONe\n"); }
     if(pwm_start() != ESP_OK) { ESP_LOGI(MAIN_TAG, "Error in starting pwm\n"); }
     return ESP_OK;
 }


/********************** I2C core functions ******************************/

/// writes data to slave
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

// reads data from slave
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


/********************* Application Tasks *******************************/

// samples from the MCU and logs data/sends to send q
static void mpu_sample(void *args){
    /* read data from accelerometer */
    uint8_t sensor_data[14];
    MPU_DATA data;
    MPU6050 mpu;

    while (1) {
        /* loop forever */

        memset(sensor_data, 0, 14);
        esp_err_t ret = i2c_read_from_slave(I2C_NUM_0, MPU6050_SENSOR_ADDR, ACCEL_XOUT_H, sensor_data, 14);
        if (ret == ESP_OK) {
        /* increment id number */
        id++;
        /* fill data struct with data */
        mpu.Accelerometer_X = (int16_t) sensor_data[0] << 8| sensor_data[1];
        mpu.Accelerometer_Y = (int16_t) sensor_data[2] << 8| sensor_data[3];
        mpu.Accelerometer_Z = (int16_t) sensor_data[4] << 8| sensor_data[5];
        mpu.Temperature = 36.53 + ((double)(int16_t)((sensor_data[6] << 8) | sensor_data[7]) / 340);  // check this... double or float????
        mpu.Gyroscope_X = (int16_t) sensor_data[8] << 8| sensor_data[9];
        mpu.Gyroscope_Y = (int16_t) sensor_data[10] << 8| sensor_data[11];
        mpu.Gyroscope_Z = (int16_t) sensor_data[12] << 8| sensor_data[13];

        /* log data to console */
        //ESP_LOGI(MAIN_TAG, "Accel X: %d\tAccel Y: %d\tAccel Z: %d\tGyros X: %d\tGyros Y: %d\tGyros Z: %d", mpu.Accelerometer_X, mpu.Accelerometer_Y, mpu.Accelerometer_Z, mpu.Gyroscope_X,mpu.Gyroscope_Y, mpu.Gyroscope_Z);
        /* problems with printing floats - remember some specific flags from stm32, could be the same?
        //ESP_LOGI(MAIN_TAG, "Temp: %f\n", mpu.Temperature);
        //ESP_LOGI(MAIN_TAG, "size of float: %d\t size of double: %d", sizeof(float), sizeof(double));
        } else {
            ESP_LOGI(MAIN_TAG, "Error in reading from MPU\n");
        }
        /*  build data to send to queue - maybe redundant to do both mpu and mpu_data */


        time_now = xTaskGetTickCount() / portTICK_PERIOD_MS;
        data.data = mpu;
        data.id = id;
        /* make a timestamp and delta t for change over time */
        data.timestamp = time_now;
        data.delta_t = time_now - time_last;
        time_last = time_now;
        //append_to_results(&data);
        /* send data to queue - block until last data is sent */
        if(uxQueueMessagesWaiting(qHandle)) { vTaskDelay(50/portTICK_PERIOD_MS); }
        else{   /* if mode==webserver, just send the mpu data, else if tcp stream, send Data struct w. timestamp */
            if(MODE == 1) { xQueueSendToBack(qHandle, (void *)&data, portMAX_DELAY); }
            else { xQueueSendToBack(qHandle, (void *)&mpu, portMAX_DELAY); }
            }
        }
    }
    i2c_driver_delete(I2C_MASTER_NUM);
}

// adjusts the LED brightness
static void manage_led(void *args){

    //MPU_DELTA delta;
    //memcpy((void *)&delta, args, sizeof(MPU_DELTA));

    uint32_t max_duty=5000, duty;
    bool direction = 1;
    uint8_t channel =1;

    while(1){
        if(CONNECTED_STATUS == 0){
        // while connecting, do fairly quick fade-in/out
            //ESP_LOGI(MAIN_TAG, "[pre] Setting LED duty: %u", duty);
            if(pwm_get_duty(0, &duty) != ESP_OK) { ESP_LOGI(MAIN_TAG, "Error in getting duty...\n"); }
            if(duty > max_duty){ duty = max_duty; }
            if(direction && duty < max_duty){ duty+=200; if(pwm_set_duty(0, duty) != ESP_OK) { ESP_LOGI(MAIN_TAG, "Error in setting duty...\n"); } pwm_start(); }
            else if(!(direction) && duty > 0){ duty-=200; if(pwm_set_duty(0, duty) != ESP_OK) { ESP_LOGI(MAIN_TAG, "Error in setting duty...\n"); } pwm_start(); }
            else { direction=!direction; }
        }
        else {
            //ESP_LOGI(MAIN_TAG, "Setting LED duty: %zu", duty);
            if(pwm_get_duty(0, &duty) != ESP_OK) { ESP_LOGI(MAIN_TAG, "Error in getting duty...\n"); }
            if(duty > max_duty){ duty = max_duty; }
            if(direction && duty < max_duty){ duty++; if(pwm_set_duty(0, duty) != ESP_OK) { ESP_LOGI(MAIN_TAG, "Error in setting duty...\n"); } pwm_start(); }
            else if(!(direction) && duty > 0){ duty--; if(pwm_set_duty(0, duty) != ESP_OK) { ESP_LOGI(MAIN_TAG, "Error in setting duty...\n"); } pwm_start(); }
            else { direction=!direction; }
        }
        vTaskDelay(100/portTICK_RATE_MS);
    }
}

// transmits collected data over tcp to server
static void transmit_sample(void *args){
    /* send data from struct over TCP to server */
    void *payload;
    char rx_buffer[56];
    MPU_DATA data;
    MPU_WS wsdata;

    while(1){
        /* loop forever */
        /* wait on data from queue */
        if(!(uxQueueMessagesWaiting(qHandle)) || !CONNECTED_STATUS){
             vTaskDelay(30/portTICK_PERIOD_MS); }
        else{
            xQueueReceive(qHandle, (void *)&data, portMAX_DELAY);
            payload = &data; }
        int sent = send(sockfd_0, payload, DATA_LEN, 0);
        if(sent < 0) {
            ESP_LOGI(TCP_TAG, "Error in sending data\n");
            check_tcp(); }
        else if(sent > DATA_LEN){
            ESP_LOGI(TCP_TAG, "Sent more than expected!\n"); }
        else {
            ESP_LOGI(TCP_TAG, "Packet sent.\n"); }
    }

}

/********************* Utility Functions *******************************/

// appends newest results to list
static void append_to_results(MPU_DATA *data){

    if(r_index < 10){
        //bzero(results[r_index], sizeof(MPU_DATA));
        results[r_index] = data;
        r_index++;
    }
    else{
        r_index = 0;
        //bzero(results[r_index], sizeof(MPU_DATA));
        results[r_index] = data;
    }
}


// finds change in measurements over time
static void get_delta(void *args){

    MPU_DELTA delta;
    while(1){
        bzero(&delta, sizeof(MPU_DELTA));

        if(id < 10){ vTaskDelay(2000/portTICK_RATE_MS); }
        if(r_index > 0){
            delta.deltaAX = (int16_t)results[r_index-1]->data.Accelerometer_X - (int16_t)results[r_index]->data.Accelerometer_X;
            delta.deltaAZ = (int16_t)results[r_index-1]->data.Accelerometer_Z - (int16_t)results[r_index]->data.Accelerometer_Z;
            delta.deltaAY = (int16_t)results[r_index-1]->data.Accelerometer_Y - (int16_t)results[r_index]->data.Accelerometer_Y;
            delta.deltaGX = (int16_t)results[r_index-1]->data.Gyroscope_X - (int16_t)results[r_index]->data.Gyroscope_X;
            delta.deltaGY = (int16_t)results[r_index-1]->data.Gyroscope_Y - (int16_t)results[r_index]->data.Gyroscope_Y;
            delta.deltaGZ = (int16_t)results[r_index-1]->data.Gyroscope_Z - (int16_t)results[r_index]->data.Gyroscope_Z;
            delta.deltaTemp = (float)results[r_index-1]->data.Temperature - (float)results[r_index]->data.Temperature;
            delta.deltaTime = (uint32_t)results[r_index-1]->delta_t;
        } else {
            // r_index = 0 so last reading is readings[10]
            delta.deltaAX = (int16_t)results[9]->data.Accelerometer_X - (int16_t)results[r_index]->data.Accelerometer_X;
            delta.deltaAZ = (int16_t)results[9]->data.Accelerometer_Z - (int16_t)results[r_index]->data.Accelerometer_Z;
            delta.deltaAY = (int16_t)results[9]->data.Accelerometer_Y - (int16_t)results[r_index]->data.Accelerometer_Y;
            delta.deltaGX = (int16_t)results[9]->data.Gyroscope_X - (int16_t)results[r_index]->data.Gyroscope_X;
            delta.deltaGY = (int16_t)results[9]->data.Gyroscope_Y - (int16_t)results[r_index]->data.Gyroscope_Y;
            delta.deltaGZ = (int16_t)results[9]->data.Gyroscope_Z - (int16_t)results[r_index]->data.Gyroscope_Z;
            delta.deltaTemp = (float)results[9]->data.Temperature - (float)results[r_index]->data.Temperature;
            delta.deltaTime = (uint32_t)results[9]->delta_t;
        }
        //ESP_LOGI(MAIN_TAG, "Accel X: %d\tAccel Y: %d\tAccel Z: %d\tGyros X: %d\tGyros Y: %d\tGyros Z: %d", mpu.Accelerometer_X, mpu.Accelerometer_Y, mpu.Accelerometer_Z, mpu.Gyroscope_X,mpu.Gyroscope_Y, mpu.Gyroscope_Z);
        //ESP_LOGI(DEBUG, "dAX: %d dAY: %d dAZ: %d ", delta.deltaAX, delta.deltaAY, delta.deltaAZ);
        //ESP_LOGI(DEBUG, "dGX: %d dGY: %d dGZ: %d", delta.deltaGX, delta.deltaGY, delta.deltaGZ);
        //ESP_LOGI(DEBUG, "dT: %z", delta.deltaTime);

        vTaskDelay(2000/portTICK_RATE_MS);
        uint32_t a_delta = delta.deltaAX + delta.deltaAY + delta.deltaAZ;
        uint32_t g_delta = delta.deltaGX + delta.deltaGY + delta.deltaGZ;
        if(a_delta > THRESHOLD) {
            int intensity = 1000;
            manage_led((void *)&intensity); }

        vTaskDelay(500/portTICK_RATE_MS);
    }

}

// checks the tcp connection using heartbeat packet
static void check_tcp(){
        /* TCP heartbeat function */

    char rx_buffer[12];
    char *needle = "BEAT";

    int tcp_check = send(sockfd_0, HEARTBEAT, strlen(HEARTBEAT), 0);
    if(tcp_check < 0) {
        close(sockfd_0);
        CONNECTED_STATUS = 0;
        ESP_LOGI(TCP_TAG, "Restarting TCP socket.\n");
        connect_to_server();
    }
    int rec = recv(sockfd_0, rx_buffer, sizeof(rx_buffer), NULL);
    if(strstr(rx_buffer, needle) != NULL){
        ESP_LOGI(TCP_TAG, "Heartbeat succesful\n");
    }
}

// connects to the configured server
static void connect_to_server(){

    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    struct sockaddr_in target;
    target.sin_addr.s_addr = inet_addr(SERVER_ADDR);
    target.sin_family = AF_INET;
    target.sin_port = htons(SERVER_PORT);

    /* create socket file descriptor */
    sockfd_0 = socket(addr_family, SOCK_STREAM, ip_protocol);
    if(sockfd_0 < 0) {
        ESP_LOGI(TCP_TAG, "Unable to create socket for server connection" ); }
    else {
        ESP_LOGI(TCP_TAG, "Socket created!\n"); }

    /* connect to the server */
    int err = connect(sockfd_0, (struct sockaddr *)&target, sizeof(target));
    if(err != 0) {
        ESP_LOGI(TCP_TAG, "Error in connecting to socket...\n"); }
    else {
        ESP_LOGI(TCP_TAG, "Connected to socket\n");
         }

}

/***************************** WebServer **********************************/

static void http_config(){

    uint32_t event_bits = CONNECTED_BIT;
    // wait for bit(s) to be set in event group //
    xEventGroupWaitBits(wifi_event_group, event_bits, false, true, portMAX_DELAY);
    ESP_LOGI(EVENT_TAG, "GOT IP");
    //now open a socket on the device and connect to the server //
    //start_http_server();
    start_httpd();
}

// begin the lwip interface, registers websocket callbacks and init the http demon
static void start_httpd(){
    lwip_init();
    websocket_register_callbacks((tWsOpenHandler)ws_open_callback, (tWsHandler)ws_recv_cb);
    httpd_init();

}

/****************************** WebSocket *********************************/
/**
Attempt at websocket server
see (http://www.barth-dev.de/websockets-on-the-esp32/)
see (https://github.com/lujji/esp-httpd/blob/master/examples/http_server/http_server.c)
**/


static void websocket_serve(void *args){

    struct tcp_pcb *tcp = (struct tcp_pcb *)args;
    MPU6050 data;

    while(1){
        /* check ws is still active */
        if(tcp == NULL || tcp->state != ESTABLISHED){
            ESP_LOGI(WS_TAG, "Websocket link is closed :(");
            break;
        }
        /* read from queue if data available */
        if(!(uxQueueMessagesWaiting(qHandle)) || !CONNECTED_STATUS){
             vTaskDelay(50/portTICK_PERIOD_MS); }
        else{
            xQueueReceive(qHandle, (void *)&data, portMAX_DELAY);
            ESP_LOGI(WS_TAG, "Sending WS data...");
            int16_t dX;
            int16_t dY;
            int16_t dZ;
            int16_t dT;
            dT = (int16_t )((xTaskGetTickCount() / portTICK_PERIOD_MS));
            if(REQUEST == 1){
                dX = data.Accelerometer_X;
                dY = data.Accelerometer_Y;
                dZ = data.Accelerometer_Z;
            } else {
                dX = data.Gyroscope_X;
                dY = data.Gyroscope_Y;
                dZ = data.Gyroscope_Z;
            }
            char data[80];  // difficulty with size of data as len changes. Better just terminate after len below. 60 plenty of space
            int len = snprintf(data, sizeof(data), "{\"aX\":\"%d\",\"aY\":\"%d\",\"aZ\":\"%d\", \"t\":\"%d\"}", dX, dY, dZ, dT);
            data[len+1] = "\0";
            ESP_LOGI(WS_TAG, "Sending: %s", data);
            websocket_write(tcp, (unsigned char *)data, len, WS_TEXT_MODE);
            vTaskDelay(10);
            }
        }
        vTaskDelete(NULL);
}

static void ws_open_callback(struct tcp_pcb *tcp, const char *uri){
    // called when websocket is opened
    ESP_LOGI(WS_TAG, "Websocket opened ");
    xTaskCreate(&websocket_serve, "websocket_serve", 2056, (void *)tcp, 2, NULL);
    //if( ^^^ != ESP_OK) { ESP_LOGI(WEBSOCKET, "Error in setting up websocket"); }
}

static void ws_recv_cb(struct tcp_pcb *tcp, uint8_t data, uint16_t data_len, uint8_t mode){

    if(data == 0x41 && REQUEST != 1){ REQUEST = 1; ESP_LOGI(WS_TAG, " MODE: A"); }
    else if(data == 0x47 && REQUEST != 2) { REQUEST = 2; ESP_LOGI(WS_TAG, " MODE: G"); }
    else { ; }

}

/**************************** APP MAIN ************************************/

void app_main(void){

    //bzero((char *)&results, sizeof(MPU_DATA)*10);
    ESP_LOGI(MAIN_TAG, "Config pwm...");
    pwm_config();
    ESP_LOGI(MAIN_TAG, "Config i2c...");
    i2c_config();
    ESP_LOGI(MAIN_TAG, "Config mpu...");
    mpu_config();
    ESP_LOGI(MAIN_TAG, "Config wifi...");
    wifi_config();

    /* create a queue for the data */
    if(MODE == 1){ qHandle = xQueueCreate(1, sizeof(MPU_DATA)); }
    else if(MODE == 2) { qHandle = xQueueCreate(1, sizeof(MPU6050)); }


    if(qHandle != NULL){
        /* create tasks to run */
        if(MODE == 1){
            xTaskCreate(transmit_sample, "transmit", 5012, NULL, 2, NULL);
        }
        xTaskCreate(manage_led, "ledc", 5012, NULL, 2, NULL);
        //xTaskCreate(get_delta, "delta", 2048, NULL, 1, NULL);
        xTaskCreate(mpu_sample, "sample", 5012, NULL, 2, NULL);
    }
    else{
        ESP_LOGI(MAIN_TAG, "Error in establishing the queue.");
        while(1){
            /* here to catch errors */
        }
     }


}
