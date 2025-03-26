#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"


// ##############################################################################

static const char *COMM_TAG = "SPI_SLAVE";

#define RCV_HOST    HSPI_HOST

#define GPIO_MOSI     13
#define GPIO_MISO     12
#define GPIO_SCLK     14
#define GPIO_CS       15

// Handshake pin number (slave drives this)
#define GPIO_HANDSHAKE 4

#define TRANSFER_SIZE 2048

// Post-setup callback: Called after the transaction is queued.
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 1);
}

// Post-transaction callback: Called after the transaction is completed.
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

// ##############################################################################

static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83
};

uint8_t crc8(uint8_t *data, size_t length, uint8_t offset) {
    uint8_t crc = 0x00; // Initial value; change if your variant requires a different start.
    for (size_t i = offset; i < length - offset; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

// ##############################################################################

static esp_err_t spi_slave_init(void)
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
    return ret; 
}

static void spi_handler_task(void *pvParameters) { 
    uint8_t *rx_buf_parent = spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0);
    uint8_t *rx_buf = spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0);
    uint8_t *tx_buf = spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0);
    if (!rx_buf || !tx_buf) {
        ESP_LOGE(COMM_TAG, "Failed to allocate DMA buffers");
        return;
    }
    memset(tx_buf, 0, TRANSFER_SIZE);
    memset(rx_buf_parent, 0, TRANSFER_SIZE);
    
    int count = 0; 
    spi_slave_transaction_t t_recv = {0};
    t_recv.length    = TRANSFER_SIZE * 8;

    while (1) {
        memset(rx_buf, 0, TRANSFER_SIZE);
        t_recv.tx_buffer = tx_buf;
        t_recv.rx_buffer = rx_buf;

        esp_err_t ret = spi_slave_transmit(RCV_HOST, &t_recv, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(COMM_TAG, "Receive transaction failed: %s", esp_err_to_name(ret));
            continue;
        }
        ESP_LOGI(COMM_TAG, "%d:  Sent buffer; first byte: 0x%02X, 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X ..", count, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[TRANSFER_SIZE - 2], tx_buf[TRANSFER_SIZE - 1]);
        ESP_LOGI(COMM_TAG, "%d:  Received buffer; first byte: 0x%02X, 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X ..", count, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[TRANSFER_SIZE - 2], rx_buf[TRANSFER_SIZE - 1]);

        memcpy(rx_buf_parent, tx_buf, TRANSFER_SIZE);
        memcpy(tx_buf, rx_buf, TRANSFER_SIZE);
        tx_buf[2] = crc8(rx_buf, TRANSFER_SIZE, 3); 

        if (count == 1) { 
            count = 0; 
        } else {
            count = 1; 
        }
    }
}

// ##############################################################################

void app_main(void)
{

    esp_err_t ret = spi_slave_init(); 
    if (ret != ESP_OK)
    {
        ESP_LOGE(COMM_TAG, "SPI SLAVE INITIALIZATION FAILED!: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(COMM_TAG, "SPI SLAVE INITIALIZED!");

    xTaskCreate(spi_handler_task, "spi_handler", 4096, NULL, 5, NULL);
    ESP_LOGI(COMM_TAG, "SPI HANDLER INITIALIZED!");
}
