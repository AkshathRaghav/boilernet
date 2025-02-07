#ifndef READ_UART_H
#define READ_UART_H

// #include "driver/uart.h"
#include "/root/esp/v5.4/esp-idf/components/esp_driver_uart/include/driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sd_test_io.h"

#define EXAMPLE_MAX_CHAR_SIZE    64
#define CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
#define PIN_NUM_MISO  6
#define PIN_NUM_MOSI  4
#define PIN_NUM_CLK   5
#define PIN_NUM_CS    1
#define CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED

void uart_init();
void uart_receive_task(void *arg);
void app_main(void);

#endif READ_UART_H
