#include "./read_uart.h"
#include "driver/gpio.h"

#define LED_GPIO 18        // LED connected to GPIO 17
#define UART_NUM UART_NUM_0 // USB UART
#define BUF_SIZE 1024


void led_init() {
    esp_rom_gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

void app_main() {
    uint8_t data[100];
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data), 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            gpio_set_level(LED_GPIO, 1);
        }
    }
}