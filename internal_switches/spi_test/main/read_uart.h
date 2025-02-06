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

void uart_init();
void uart_receive_task(void *arg);
void app_main(void);

#endif READ_UART_H
