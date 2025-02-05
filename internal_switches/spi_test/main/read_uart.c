#include "./read_uart.h"
#include "driver/gpio.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

// Define GPIO for LED
#define LED_GPIO 18        // LED connected to GPIO 18
#define UART_NUM UART_NUM_0 // Use UART0 over USB
#define UART_LOG UART_NUM_1 // Use UART0 over USB
#define BUF_SIZE 1024

void UartInit(void);
void led_init(void);

void UartInit(void) {
    const char *TAG = "UART";
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // UART configuration
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Apply UART configuration
    ESP_ERROR_CHECK(uart_param_config(UART_LOG, &uart_config));

    // Set UART pins (keeping default USB-UART)
    ESP_ERROR_CHECK(uart_set_pin(UART_LOG, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_LOG, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
}

// Initialize LED GPIO
void led_init() {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

void send_response(const char *response) {
    uart_write_bytes(UART_NUM, response, strlen(response));
}

void app_main() {
    // Initialize UART and LED
    UartInit();
    led_init();

    uint8_t data[BUF_SIZE];

    while (1) {
        // Read data from UART
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';  // Null-terminate for string processing
            // ESP_LOGI("UART", "Received %d bytes: %s", len, data);
            // ESP_LOGI("UART", "Received %d bytes", len);

            // Turn on LED when data is received
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(500));  // Keep LED on for 500ms
            gpio_set_level(LED_GPIO, 0);

            // Basic verification (e.g., checking if it contains "ESP32")
            if (strstr((char *)data, "ESP32") != NULL) {
                send_response("ACK\n");  // Send acknowledgment
            } else {
                send_response("NACK\n");  // Send negative acknowledgment
            }
        }

        // Small delay to prevent excessive CPU usage
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}