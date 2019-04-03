/*
*   ESP32 bike POV display
*   Version: 0.1
*   Date: 31/3/19
*   Platform: esp-idf
*   IDE: Platform-io
*
*
*/



/*
*   Psuedo
*   step 1: Init - initialise SPI bus, initialise BLE Gatt, initialise GPIO interrupt pins
*      step 2:  BLE advertise
*        step 3: recv from Client, load selected image, begin display
*
*
*/



 #include <stdio.h>
 #include <string.h>
 #include <stdlib.h>

 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "freertos/event_groups.h"


 #include "esp_log.h"
 #include "esp_system.h"
 #include "esp_err.h"
 #include "esp_intr_alloc.h"

 #include "driver/spi_master.h"
 #include "driver/spi_common.h"
 #include "driver/gpio.h"
 #include "esp_spi_flash.h"

#define SPI_IF 1                // which spi interface to use.
#define CONFIG_SPI_TX 11        // gpio pin for data
#define CONFIG_SPI_CLK 12       // gpio pin for clock
#define GPIO_INPUT_IO_0 GPIO_NUM_2  //rotation interrupt 1
#define GPIO_INPUT_IO_1 GPIO_NUM_3  //rotation interrupt 2
#define GPIO_INPUT_PIN_SEL ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))  // bitmask for pins

#define NUMLEDS 30                                      // total number of leds
#define MAX_SPI_XFER_SIZE ((NUMLEDS * 32) + 64)         // 32 bits per LED plus start & end frame
#define NUMFRAMES 360

volatile int count = 0;                                 // for tracking and debugging
gpio_config_t rot_intr;                                 // gpio config

uint32_t last_time=0;                                   // track rotation time between functions
uint32_t frame_t;

// create a queue for the speed functions
QueueHandle_t speed_queue;

struct image_exchange{
    void *put_to;
    void *get_from;
    int size;
};

struct image_exchange image_x;

static void IRAM_ATTR rotary_interrupt_handler(void *pin_triggered){
    // interrupt twice per cycle. Simply send back the current tick count.
    uint32_t delta = (xTaskGetTickCountFromISR() - last_time;
    uint32_t time = delta * portTICK_PERIOD_MS;   // time in ms  FIX!!!!!!!!!!!!!!
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken =  pdFALSE;
    uint32_t pin = (uint32_t) pin_triggered;
    // check if data unblocks a waiting task ...
    xQueueSendFromISR(speed_queue, &time, &xHigherPriorityTaskWoken);
    // yield directly to that task
    portYIELD_FROM_ISR();


}


static void transfer_image(void *args){
    // function to transfer image into DMAcapable memory for quicker transactions
    struct image_exchange xchange;
    xchange.;
    while(1){

        int a = 10;
        a= a / 5;
        vTaskDelay(2000);
    }

}

static void empty(void *args){
    // dummy function
    int a;
    while(1) {
        a = 10;
        a = a / 2;
        printf("Hi.\n");
        vTaskDelay(1000);

    }

}


static void get_speed(void *args){
    // calculate speed and frame time when speed data is available
    // delta = changein time i.e time for half a rotation.  frame_t updates to be 2delta / no_frames
    //
    uint32_t new_time, delta;

    while(1){

        if(uxQueueMessagesWaiting(speed_queue) == 0) { vTaskDelay(100); }
        else{
            new_time = xQueueReceive(speed_queue, &new_time, portMAX_DELAY);
            delta = last_time - new_time;
            frame_t = (2 * delta) / NUMFRAMES;
            last_time = new_time;
        }


    }

}

void app_main(void){

    speed_queue = xQueueCreate(1, sizeof(uint32_t));
    // gpio pin setups
    rot_intr.intr_type = GPIO_PIN_INTR_POSEDGE;
    rot_intr.pull_down_en = 1;
    rot_intr.pull_up_en = 0;
    rot_intr.mode = GPIO_MODE_INPUT;
    rot_intr.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    gpio_config(&rot_intr);
    // init interrupts - main app should be pinned to a core '''check this''''
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INPUT_IO_0, rotary_interrupt_handler, (void *) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, rotary_interrupt_handler, (void *) GPIO_INPUT_IO_1);

    //  set heap memory to store active image
    size_t dma_mem=heap_caps_get_free_size(MALLOC_CAP_DMA);
    printf("DMA_MEMORY: %zu\n", dma_mem);
    size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
    printf("Largest block size: %zu\n", largest);

    void *image_heap=heap_caps_malloc(32000, MALLOC_CAP_DMA);
    if(image_heap == NULL) { printf("Error in assigning image heap\n"); }
    else { printf("Image heap at %p\n", image_heap); }
    // clear the new memory
    bzero(image_heap, 32000);

    // examine SPI flash mem
    size_t flash_size = spi_flash_get_chip_size();
    printf("Size of SPI flash memory: %zu\n", flash_size);
    uint32_t free_data_pages = spi_flash_mmap_get_free_pages(SPI_FLASH_MMAP_DATA);
    uint32_t free_instr_pages = spi_flash_mmap_get_free_pages(SPI_FLASH_MMAP_DATA);
    printf("Free pages (data): %zu\nFree pages (instr): %zu\n", free_data_pages, free_instr_pages);



    // initialise SPI bus
    spi_bus_config_t bus_config;
    bus_config.miso_io_num = -1;
    bus_config.mosi_io_num = CONFIG_SPI_TX;
    bus_config.sclk_io_num = CONFIG_SPI_CLK;
    bus_config.max_transfer_sz = MAX_SPI_XFER_SIZE;
    bus_config.quadhd_io_num = -1;
    bus_config.quadwp_io_num = -1;
    spi_bus_initialize(SPI_IF, &bus_config, 1);   // spi interface (1 or 2), bus config & dma channel (1 or 2);

    // intialise the SPI device
    spi_device_interface_config_t dev_config;
    dev_config.mode = 0;
    dev_config.clock_speed_hz = 200000;
    dev_config.spics_io_num = -1;
    dev_config.queue_size = (NUMLEDS + 3);      // shouldn't ever exceed numleds + 2 for data to be queued
    spi_device_handle_t leds;
    spi_bus_add_device(SPI_IF, &dev_config, &leds);




    xTaskCreate(empty, "empty", 5012, NULL, 1, NULL);
    xTaskCreate(transfer_image, "transfer_image", 5012, &image_x, 3, NULL);
    xTaskCreate(get_speed, "get_speed", 2048, NULL, 3, NULL);
}
