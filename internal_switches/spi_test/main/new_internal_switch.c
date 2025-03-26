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
#define RCV_HOST    HSPI_HOST

// SPI pins (must match master's configuration)
#define GPIO_MOSI     13
#define GPIO_MISO     12
#define GPIO_SCLK     14
#define GPIO_CS       15

// Handshake pin number (slave drives this)
#define GPIO_HANDSHAKE 4

// Transfer size: 128 bytes per transaction.
#define TRANSFER_SIZE 1024

// Post-setup callback: Called after the transaction is queued.
// Set the handshake pin high to indicate the slave is ready.
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 1);
}

// Post-transaction callback: Called after the transaction is completed.
// Set the handshake pin low to indicate the slave is busy (or that the transaction has ended).
void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 0);
}

void handshake_init_slave(void)
{
    // Configure handshake pin as output.
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
}

void app_main(void)
{
    esp_err_t ret;

    handshake_init_slave();

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
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb
    };

    // Enable pull-ups on SPI lines so that the signals are stable when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS,   GPIO_PULLUP_ONLY);

    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPI slave initialized");

    // Allocate DMA buffers for two transactions.
    uint8_t *rx_buf_recv = spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0);
    uint8_t *rx_buf_resp = spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0);
    if (!rx_buf_recv || !rx_buf_resp) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffers");
        return;
    }
    memset(rx_buf_resp, 0, TRANSFER_SIZE);
    
    int count = 0; 
    spi_slave_transaction_t t_recv = {0};

    while (1) {
        // ----- Transaction 1: Receive full buffer from Master -----
        memset(rx_buf_recv, 0, TRANSFER_SIZE);
        t_recv.length    = TRANSFER_SIZE * 8;
        t_recv.tx_buffer = rx_buf_resp;
        t_recv.rx_buffer = rx_buf_recv;

        ret = spi_slave_transmit(RCV_HOST, &t_recv, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Receive transaction failed: %s", esp_err_to_name(ret));
            continue;
        }
        ESP_LOGI(TAG, "%d:  Sent buffer; first byte: 0x%02X, 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X ..", count, rx_buf_resp[0], rx_buf_resp[1], rx_buf_resp[2], rx_buf_resp[3], rx_buf_resp[TRANSFER_SIZE - 2], rx_buf_resp[TRANSFER_SIZE - 1]);
        ESP_LOGI(TAG, "%d:  Received buffer; first byte: 0x%02X, 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X ..", count, rx_buf_recv[0], rx_buf_recv[1], rx_buf_recv[2], rx_buf_recv[3], rx_buf_recv[TRANSFER_SIZE - 2], rx_buf_recv[TRANSFER_SIZE - 1]);

        // ----- Transaction 2: Echo the received buffer back to Master -----
        memcpy(rx_buf_resp, rx_buf_recv, TRANSFER_SIZE);

        if (count == 1) { 
            count = 0; 
        } else {
            count = 1; 
        }
    }
}
