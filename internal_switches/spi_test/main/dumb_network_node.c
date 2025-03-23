#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "SPI_MASTER"

// Use these pins to match the slave configuration.
#define PIN_NUM_MOSI  13
#define PIN_NUM_MISO  12
#define PIN_NUM_CLK   14
#define PIN_NUM_CS    15

// Transfer size: 128 bytes.
#define TRANSFER_SIZE 128

// Define our two commands.
#define CMD_VALID   0xA5  // If slave sees this, it should reply with 0x11.
#define CMD_INVALID 0xB0  // If slave sees any other value, it should reply with 0xFF.

// Expected responses.
#define RESP_VALID   0x11
#define RESP_INVALID 0xFF

// We'll use HSPI_HOST for this example.
#define SPI_HOST_TYPE HSPI_HOST

static spi_device_handle_t spi_master_handle = NULL;

esp_err_t spi_master_init(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TRANSFER_SIZE,
    };

    ret = spi_bus_initialize(SPI_HOST_TYPE, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5000000,  // 5 MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
    };

    ret = spi_bus_add_device(SPI_HOST_TYPE, &devcfg, &spi_master_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
    }
    return ret;
}

void app_main(void)
{
    esp_err_t ret = spi_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI master initialization failed");
        return;
    }
    ESP_LOGI(TAG, "SPI master initialized");

    // Create buffers of size TRANSFER_SIZE (128 bytes) for TX and RX.
    uint8_t tx_buf[TRANSFER_SIZE];
    uint8_t rx_buf[TRANSFER_SIZE];
    spi_transaction_t t = {0};
    int count = 0;
    uint8_t cmd_sent = 0;

    while (1) {
        // Clear TX and RX buffers.
        memset(tx_buf, 0, TRANSFER_SIZE);
        memset(rx_buf, 0, TRANSFER_SIZE);

        // Alternate between valid and invalid commands.
        if ((count & 1) == 0) {
            cmd_sent = CMD_VALID;
        } else {
            cmd_sent = CMD_INVALID;
        }
        // Place the command in the first byte of tx_buf.
        tx_buf[0] = cmd_sent;

        // --- Transaction 1: Send command ---
        memset(&t, 0, sizeof(t));
        t.length = TRANSFER_SIZE * 8;  // 128 bytes in bits.
        t.tx_buffer = tx_buf;
        t.rx_buffer = NULL;  // Ignore RX data during the command transmission.
        ret = spi_device_polling_transmit(spi_master_handle, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transmit (command) failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Sent command: 0x%02X", cmd_sent);
        }

        // Wait briefly for the slave to prepare its response.
        vTaskDelay(pdMS_TO_TICKS(50));

        // --- Transaction 2: Read response ---
        memset(tx_buf, 0, TRANSFER_SIZE);  // Send dummy data for the response transaction.
        memset(rx_buf, 0, TRANSFER_SIZE);
        memset(&t, 0, sizeof(t));
        t.length = TRANSFER_SIZE * 8;
        t.tx_buffer = tx_buf;
        t.rx_buffer = rx_buf;
        ret = spi_device_polling_transmit(spi_master_handle, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transmit (response) failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Received response: 0x%02X", rx_buf[0]);
            if (cmd_sent == CMD_VALID) {
                if (rx_buf[0] == RESP_VALID) {
                    ESP_LOGI(TAG, "Slave responded correctly for valid command.");
                } else {
                    ESP_LOGW(TAG, "Unexpected response for valid command.");
                }
            } else {
                if (rx_buf[0] == RESP_INVALID) {
                    ESP_LOGI(TAG, "Slave responded correctly for invalid command.");
                } else {
                    ESP_LOGW(TAG, "Unexpected response for invalid command.");
                }
            }
        }

        count++;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
