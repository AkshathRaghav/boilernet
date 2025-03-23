#include "./read_uart.h"

#define MOUNT_POINT "/sdcard"
#define UART_NUM UART_NUM_0  // USB UART
#define BUF_SIZE 1024
#define START_WRITE_CMD "_START_WRITE_"
#define END_WRITE_CMD "_END_WRITE_"
#define START_GET_CMD "_START_GET_"
#define END_GET_CMD "_END_GET_"
#define CONFIRM_GET_CMD "_CONFIRM_"

void led_init(int num) {
    gpio_reset_pin(num);
    gpio_set_direction(num, GPIO_MODE_INPUT_OUTPUT);
}


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

int SDCardInit(void) {
    ESP_LOGI("UARTTEST", "FIRST");

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    ESP_LOGI("UARTTEST", "SECOND");

    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    // host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ESP_LOGI("UARTTEST", "THIRD");

    int ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI("UARTTEST", "FOURTH");

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    ESP_LOGI("UARTTEST", "FIFTH");

    if (ret != ESP_OK) {
        return ret; 
    }

    return ESP_OK;
}

void send_response(const char *response) {
    uart_write_bytes(UART_NUM, response, strlen(response));
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

void size_t_to_string(size_t value, char *buffer, size_t buffer_size) {
    snprintf(buffer, buffer_size, "%zu", value);
}

void app_main() {
    UartInit();
    ESP_LOGI("UARTTEST", "Post  UART");

    int ret = SDCardInit();
    if (ret != ESP_OK) {
        keep_high(9);
        return;
    } 
    ESP_LOGI("UARTTEST", "Post SPI");

        
    toggle_led(9); // Confirm all setup 

    uint8_t data[BUF_SIZE];
    int receiving = 0;
    int sending = 0;
    FILE *f = NULL;
    const char *file_path = MOUNT_POINT "/test.bin";
    uint8_t buffer[64];
    char size_buffer[20];
    size_t read_size;
    size_t total_sent = 0;


    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        data[len] = '\0';  

        if (len > 0) {

            // Handle file writing
            if (strcmp((char *)data, START_WRITE_CMD) == 0) {
                receiving = 1;
                toggle_led(18);
                keep_high(19);

                f = fopen(file_path, "w");
                if (f == NULL) {
                    send_response("FCK\0");
                    keep_high(9);
                    receiving = 0;
                    continue;
                } 
                send_response("ACK\0");
            } 
            else if (strcmp((char *)data, END_WRITE_CMD) == 0 && receiving) {
                fclose(f);
                keep_low(19);
                toggle_led(20);
                send_response("ECK\0");
                receiving = 0;
            }
            else if (receiving) {
                if (f) {
                    fwrite(data, 1, len, f);
                    fflush(f); 
                    send_response("CCK\0");
                } else {
                    send_response("FFK\0");
                    keep_high(9);
                }
            }
            // Handle file reading
            else if (strcmp((char *)data, START_GET_CMD) == 0) {
                total_sent = 0; 
                sending = 1;
                f = fopen(file_path, "rb");
                if (f == NULL) {
                    send_response("FCK\0");
                    keep_high(9);
                    sending = 0;
                    continue;
                } 
                send_response("ACK\0");

                // toggle_led(18);
                keep_high(19);


                while ((read_size = fread(buffer, 1, sizeof(buffer), f)) > 0) {
                    uart_write_bytes(UART_NUM, (const char*)buffer, read_size);
                    vTaskDelay(pdMS_TO_TICKS(10));  

                    total_sent += read_size;

                    int ack_len = uart_read_bytes(UART_NUM, data, 3, 100 / portTICK_PERIOD_MS);
                    data[ack_len] = '\0';
                    if (strcmp((char *)data, "CCK") != 0) {
                        fclose(f);
                        sending = 0;
                        break;
                    } 
                }

                fclose(f);
                sending = 1;

                send_response("_END_GET_\0");
            } 
            else if (sending && strncmp((char *)data, CONFIRM_GET_CMD, strlen(CONFIRM_GET_CMD)) == 0) {
                snprintf(size_buffer, sizeof(size_buffer), "%zu", total_sent); 
                send_response(size_buffer);
                keep_low(19);
                sending = 0; 
            }
            else {
                receiving = 0;
                sending = 0;
                toggle_led(20);
                // send_response((char *)data);
                send_response("NCK\0");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
