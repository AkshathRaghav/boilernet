#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "lodepng.h"
#include "esp_heap_caps.h"

extern "C" {
  #include "driver/gpio.h"
  #include "driver/spi_slave.h"
  #include "hal/cache_hal.h"    
}

// TFLM includes
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// Model header (const array lives in Flash/RODATA)
#include "model_data.h"

#define BLINK_GPIO GPIO_NUM_32


#define GPIO_MOSI    GPIO_NUM_13
#define GPIO_MISO    GPIO_NUM_12
#define GPIO_SCLK    GPIO_NUM_14
#define GPIO_CS      GPIO_NUM_15

#define GPIO_HANDSHAKE GPIO_NUM_2
#define TRANSFER_SIZE   2048
#define COMM_TAG  "BLADE_SPI"

#define PACKET_TYPE_START_DATA_TO_BLADE   0xD0
#define PACKET_TYPE_DATA_TO_BLADE         0xD1
#define PACKET_TYPE_END_DATA_TO_BLADE     0xD2
#define PACKET_TYPE_GET_RESULT_FROM_BLADE 0xD3
#define PACKET_TYPE_CLEAR_BLADE 0xDF

#define PACKET_TYPE_FUC   0xA4

typedef enum {
    BLADE_FSM_WAIT_START,
    BLADE_FSM_RECEIVE,
    BLADE_FSM_PROCESS,
    BLADE_FSM_ERROR
} blade_fsm_t;