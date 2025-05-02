#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/spi_slave.h"
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include <ctype.h>
#include <stdbool.h>

// ##############################################################################

#define PACKET_TYPE_START_WRITE 0xA0
#define PACKET_TYPE_DATA_WRITE  0xA1
#define PACKET_TYPE_MID_WRITE   0xA2
#define PACKET_TYPE_END_WRITE   0xA3

#define PACKET_TYPE_START_READ 0xB0 
#define PACKET_TYPE_DATA_READ  0xB1 
#define PACKET_TYPE_END_READ   0xB3 

#define PACKET_TYPE_START_COMPUTE 0xC0   
#define PACKET_TYPE_POLL_COMPUTE  0xC1   
#define PACKET_TYPE_WAIT_COMPUTE  0xC2   
#define PACKET_TYPE_END_COMPUTE   0xC3  

#define PACKET_TYPE_START_DATA_TO_BLADE 0xD0
#define PACKET_TYPE_DATA_TO_BLADE 0xD1
#define PACKET_TYPE_END_DATA_TO_BLADE 0xD2
#define PACKET_TYPE_GET_RESULT_FROM_BLADE 0xD3
#define PACKET_TYPE_CLEAR_BLADE 0xDF

#define PACKET_TYPE_DELETE 0xE0
#define PACKET_TYPE_DELETE_RET 0xE1

#define PACKET_TYPE_FUC   0xA4

typedef enum {
    FSM_WAIT_START_WRITE,
    FSM_WAIT_DATA_WRITE,
    FSM_WAIT_START_READ,  
    FSM_WAIT_DATA_READ,    
    FSM_WAIT_END_READ,     
    FSM_WAIT_START_COMPUTE,    
    FSM_WAIT_COMPUTE_PROCESS,  
    FSM_WAIT_END_COMPUTE,      
    FSM_COMPLETE,
    FSM_ERROR
} fsm_state_t;

typedef enum
{
    FSM_IDLE, 
    FSM_SEND_DATA_TO_BLADE,  // NEW
    FSM_GET_DATA_FROM_BLADE, // NEW
} compute_fsm_state_t; 

// ##############################################################################

#define LED_NUM 32 

// ##############################################################################

static const char *COMM_TAG = "SPI_SLAVE";

#define MOUNT_POINT "/sdcard"

#define SD_HOST   VSPI_HOST   

#define SD_NUM_MOSI     23
#define SD_NUM_MISO     19
#define SD_NUM_CLK     18
#define SD_NUM_CS       4

// ##############################################################################

#define TRANSFER_SIZE 2048

#define NETWORK_HOST    HSPI_HOST

#define NETWORK_MOSI  13
#define NETWORK_MISO  12
#define NETWORK_SCLK   14
#define NETWORK_CS    15

#define NETWORK_HANDSHAKE 33

#define COMPUTE_HS_0 36
#define COMPUTE_HS_1 39 
#define COMPUTE_HS_2 26
#define COMPUTE_HS_3 27 

#define COMPUTE_CS_0  5
#define COMPUTE_CS_1  21
#define COMPUTE_CS_2  22
#define COMPUTE_CS_3  25


static const int compute_handshake_pins[] = {
    COMPUTE_HS_0,
    COMPUTE_HS_1,
    COMPUTE_HS_2
    // COMPUTE_HS_3
};

static const int compute_cs_pins[] = {
    COMPUTE_CS_0,
    COMPUTE_CS_1,
    COMPUTE_CS_2
    // COMPUTE_CS_3
};

#define NUM_COMPUTE_BLADES  (sizeof(compute_cs_pins)/sizeof(compute_cs_pins[0]))

static spi_device_handle_t compute_handles[NUM_COMPUTE_BLADES];


#define TIMEOUT_MS   15000
#define CHECK_TIMEOUT(blade)                                                     \
    do {                                                                         \
        if ((xTaskGetTickCount() - *compute_start_tick) > pdMS_TO_TICKS(TIMEOUT_MS)) { \
        ESP_LOGE(COMM_TAG, "Handshake timeout for blade %d", blade);             \
        passed[blade] = false;                                                   \
        goto cleanup;                                                            \
        }                                                                          \
    } while (0)

#define RECORD_FAILED(blade)                             \
    do {                                                \
        if (!passed[blade]) {                             \
        if (write_failed(result_file, blade) < 0) {     \
            *outer_fsm_state = FSM_ERROR;                 \
            goto cleanup;                                 \
        }                                               \
        }                                                 \
    } while (0)

// ##############################################################################

