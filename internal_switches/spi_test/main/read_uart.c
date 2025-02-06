#include "./read_uart.h"

#define UART_NUM UART_NUM_0  // USB UART
#define BUF_SIZE 1024
#define START_CMD "_START_"
#define END_CMD "_END_"


void UartInit(void) {
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
}

void send_response(const char *response) {
    uart_write_bytes(UART_NUM, response, strlen(response));
}

void led_init(int num) {
    gpio_reset_pin(num);
    gpio_set_direction(num, GPIO_MODE_OUTPUT);
}


void keep_high(int num) {
    gpio_set_level(num, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

void keep_low(int num) {
    gpio_set_level(num, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
}


void toggle_led(int num) {
    keep_high(num);
    keep_low(num);
}

void app_main() {
    led_init(18);
    led_init(19);
    led_init(20);

    UartInit();

    uint8_t data[BUF_SIZE];
    int receiving = 0;

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';  

            if (strcmp((char *)data, START_CMD) == 0) {
                receiving = 1;
                toggle_led(18);
                keep_high(19);
                send_response("ACK\0");
            } 
            else if (strcmp((char *)data, END_CMD) == 0 && receiving) {
                keep_low(19);
                toggle_led(20);
                send_response("ECK\0");
                receiving = 0;
            }
            else if (receiving) {
                send_response("CCK\0");
            }
            else {
                receiving = 0;
                toggle_led(20);
                send_response("NCK\0");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}