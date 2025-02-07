#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

// Shared SPI bus pins.
#define GPIO_MOSI           23
#define GPIO_MISO           19
#define GPIO_SCLK           18

// Chip Select pins for two slaves.
#define GPIO_CS1            17
#define GPIO_CS2            5

// Push buttons to select the slave (active low).
#define GPIO_BUTTON1        33
#define GPIO_BUTTON2        26

#define MASTER_MSG          "BUTTON_PRESSED"
#define SLAVE_ACK_STR       "ACK_BUTTON_PRESSED"

// Maximum number of retries for sending a message.
#define MAX_RETRIES         5

#ifdef CONFIG_IDF_TARGET_ESP32
#define SENDER_HOST HSPI_HOST
#else
#define SENDER_HOST SPI2_HOST
#endif

static const char *TAG = "Master";

void app_main(void)
{
    esp_err_t ret;
    spi_device_handle_t slave1_handle;
    spi_device_handle_t slave2_handle;

    // Configure the SPI bus pins.
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    // Configure SPI device settings for Slave 1.
    spi_device_interface_config_t devcfg1 = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 5000000,
        .duty_cycle_pos = 128,  // 50% duty cycle
        .mode = 0,
        .spics_io_num = GPIO_CS1,
        .cs_ena_posttrans = 3,  // Keep CS low a few extra cycles after transfer
        .queue_size = 3
    };

    // Configure SPI device settings for Slave 2.
    spi_device_interface_config_t devcfg2 = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 5000000,
        .duty_cycle_pos = 128,
        .mode = 0,
        .spics_io_num = GPIO_CS2,
        .cs_ena_posttrans = 3,
        .queue_size = 3
    };

    // Configure the push button GPIOs.
    gpio_config_t btn_conf = {
        .intr_type = GPIO_INTR_DISABLE,  // Polling the buttons
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (BIT64(GPIO_BUTTON1) | BIT64(GPIO_BUTTON2)),
        .pull_up_en = GPIO_PULLUP_ENABLE, // Active low buttons
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&btn_conf);

    // Initialize the SPI bus.
    ret = spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    // Add both slave devices to the SPI bus.
    ret = spi_bus_add_device(SENDER_HOST, &devcfg1, &slave1_handle);
    assert(ret == ESP_OK);
    ret = spi_bus_add_device(SENDER_HOST, &devcfg2, &slave2_handle);
    assert(ret == ESP_OK);

    // Buffers for SPI transactions.
    char sendbuf[128] = {0};
    char recvbuf[128] = {0};
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    ESP_LOGI(TAG, "Ready. Press button on GPIO %d for Slave 1 or on GPIO %d for Slave 2.\n", GPIO_BUTTON1, GPIO_BUTTON2);

    while (1) {
        spi_device_handle_t current_handle = NULL;
        // Check if button for Slave 1 is pressed.
        if (gpio_get_level(GPIO_BUTTON1) == 0) {
            // Simple debounce.
            vTaskDelay(pdMS_TO_TICKS(50));
            while(gpio_get_level(GPIO_BUTTON1) == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            ESP_LOGI(TAG, "Button 1 pressed, sending message to Slave 1...\n");
            current_handle = slave1_handle;
        }
        // Check if button for Slave 2 is pressed.
        else if (gpio_get_level(GPIO_BUTTON2) == 0) {
            // Simple debounce.
            vTaskDelay(pdMS_TO_TICKS(50));
            while(gpio_get_level(GPIO_BUTTON2) == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            ESP_LOGI(TAG, "Button 2 pressed, sending message to Slave 2...\n");
            current_handle = slave2_handle;
        }

        if (current_handle != NULL) {
            if (current_handle == slave1_handle) {
                ESP_LOGI(TAG, "current_handle is slave1\n");
            }
            if (current_handle == slave2_handle) {
                ESP_LOGI(TAG, "current_handle is slave2\n");
            }
            int retry_count = 0;
            bool ack_received = false;
            while (!ack_received && retry_count < MAX_RETRIES) {
                // Prepare the message.
                memset(sendbuf, 0, sizeof(sendbuf));
                strcpy(sendbuf, MASTER_MSG);
                memset(recvbuf, 0, sizeof(recvbuf));

                // Set up the SPI transaction.
                t.length = sizeof(sendbuf) * 8;
                t.tx_buffer = sendbuf;
                t.rx_buffer = recvbuf;

                ret = spi_device_transmit(current_handle, &t);
                if (ret != ESP_OK) {
                    ESP_LOGI(TAG, "SPI transaction failed (retry %d)!\n", retry_count);
                } else {
                    if (strstr(recvbuf, SLAVE_ACK_STR) != NULL) {
                        ack_received = true;
                        ESP_LOGI(TAG, "Received valid acknowledgement from slave: %s\n", recvbuf);
                    } else {
                        ESP_LOGI(TAG, "Invalid acknowledgement: \"%s\" (retry %d)\n", recvbuf, retry_count);
                    }
                }
                retry_count++;
                if (!ack_received) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            if (!ack_received) {
                ESP_LOGI(TAG, "Failed to receive valid acknowledgement after %d retries.\n", retry_count);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Polling interval.
    }

    // (Cleanup code – never reached in this example)
    spi_bus_remove_device(slave1_handle);
    spi_bus_remove_device(slave2_handle);
}
