#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "SPI_SLAVE";

// Use HSPI_HOST for this example.
#define RCV_HOST      HSPI_HOST

// Pin assignments (must match your master):
#define GPIO_MOSI     13
#define GPIO_MISO     12
#define GPIO_SCLK     14
#define GPIO_CS       15

// Transfer size: 128 bytes per transaction.
#define TRANSFER_SIZE 1024

// Global variable to hold the last received command.
volatile uint8_t last_cmd = 0;

// Post-transaction callback: after a transaction, save the received byte.
void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    if (trans->rx_buffer) {
        last_cmd = *((uint8_t *)trans->rx_buffer);
    }
}

void app_main(void)
{
    esp_err_t ret;

    // Configure SPI bus.
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TRANSFER_SIZE,
    };

    // Configure SPI slave interface.
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 2,  // We'll queue two transactions in a row.
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = my_post_trans_cb,
    };

    // Enable pull-ups on SPI lines.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS,   GPIO_PULLUP_ONLY);

    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPI slave initialized");

    // Allocate DMA-capable buffers for two transactions.
    uint8_t *tx_buf_recv = spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0);
    uint8_t *rx_buf_recv = spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0);
    uint8_t *tx_buf_resp = spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0);
    uint8_t *rx_buf_resp = spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0);
    if (!tx_buf_recv || !rx_buf_recv || !tx_buf_resp || !rx_buf_resp) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffers");
        return;
    }

    // Initialize transaction descriptors.
    spi_slave_transaction_t t_recv = {0};
    spi_slave_transaction_t t_resp = {0};

    while (1) {
        // ----- Transaction 1: Receive Command from Master -----
        memset(rx_buf_recv, 0, TRANSFER_SIZE);
        memset(tx_buf_recv, 0, TRANSFER_SIZE);
        tx_buf_recv[0] = 0x00;  // Dummy value for TX
        t_recv.length    = TRANSFER_SIZE * 8;  // in bits
        t_recv.tx_buffer = tx_buf_recv;
        t_recv.rx_buffer = rx_buf_recv;

        ret = spi_slave_transmit(RCV_HOST, &t_recv, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Receive transaction failed: %s", esp_err_to_name(ret));
            continue;
        }
        ESP_LOGI(TAG, "Received command: 0x%02X", last_cmd);

        // ----- Transaction 2: Send Response to Master -----
        memset(tx_buf_resp, 0, TRANSFER_SIZE);
        if (last_cmd == 0xA5) {
            tx_buf_resp[0] = 0x11;
        } else {
            tx_buf_resp[0] = 0xFF;
        }
        memset(rx_buf_resp, 0, TRANSFER_SIZE);
        t_resp.length    = TRANSFER_SIZE * 8;  // in bits
        t_resp.tx_buffer = tx_buf_resp;
        t_resp.rx_buffer = rx_buf_resp;

        ret = spi_slave_transmit(RCV_HOST, &t_resp, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Response transaction failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Sent response: 0x%02X", tx_buf_resp[0]);
        }
        // Loop back for the next command.
    }
}
    