#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

// SPI Pin definitions
#define PIN_NUM_MOSI  13
#define PIN_NUM_MISO  12
#define PIN_NUM_CLK   14
#define PIN_NUM_CS    15

// LED Pin definitions (adjust these pins as needed)
#define LED_START     GPIO_NUM_18
#define LED_DAT       GPIO_NUM_19
#define LED_FIN       GPIO_NUM_21
#define LED_FUC       GPIO_NUM_22

// Packet type definitions (from SPI master)
#define PACKET_ACK    0x01  // Used for ACK (and also for START in our protocol)
#define PACKET_DAT    0x02  // Data packet
#define PACKET_FIN    0x03  // End of stream
#define PACKET_FUC    0x04  // Error condition

// Maximum packet size (1 header byte + up to 1024 bytes payload)
#define MAX_PACKET_SIZE  (1024 + 1)

// Global SPI slave buffers (these must be DMA-capable)
static uint8_t slave_tx_buffer[MAX_PACKET_SIZE];
static uint8_t slave_rx_buffer[MAX_PACKET_SIZE];

// Global variable to determine what header to send in the next transaction
// For DAT packets, we want to send an ACK back.
static uint8_t next_tx_header = 0xFF;  // Default value (no valid packet)

// Tag for logging
static const char *TAG = "SPI_SLAVE";

/********************************************************************
 * LED Initialization
 ********************************************************************/
static void led_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        // Bit mask for all LED pins
        .pin_bit_mask = ((uint64_t)1 << LED_START) |
                        ((uint64_t)1 << LED_DAT)   |
                        ((uint64_t)1 << LED_FIN)   |
                        ((uint64_t)1 << LED_FUC),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);

    // Initialize all LEDs to off (low)
    gpio_set_level(LED_START, 0);
    gpio_set_level(LED_DAT, 0);
    gpio_set_level(LED_FIN, 0);
    gpio_set_level(LED_FUC, 0);
}

/********************************************************************
 * SPI Slave Callbacks
 ********************************************************************/

// Called just before a transaction starts.
// Here we prepare the TX buffer with the header we wish to return.
void spi_slave_post_setup_callback(spi_slave_transaction_t *trans)
{
    // Set the first byte of the TX buffer to the next header value.
    slave_tx_buffer[0] = next_tx_header;
    ESP_LOGI(TAG, "Post-Setup: TX header set to 0x%02X", next_tx_header);
}

// Called after a transaction completes.
// Process the received packet (in slave_rx_buffer) and set LEDs accordingly.
// If a DAT packet is received, we set the next header to PACKET_ACK so the master gets an ACK.
void spi_slave_post_trans_callback(spi_slave_transaction_t *trans)
{
    uint8_t header = slave_rx_buffer[0];
    ESP_LOGI(TAG, "Post-Trans: Received header 0x%02X", header);

    switch (header) {
        case PACKET_ACK:
            ESP_LOGI(TAG, "Received START packet (interpreted as ACK)");
            gpio_set_level(LED_START, 1);
            // For START, no ACK reply is needed.
            next_tx_header = 0xFF;
            break;
        case PACKET_DAT:
            ESP_LOGI(TAG, "Received DATA packet (DAT)");
            gpio_set_level(LED_DAT, 1);
            // Prepare to send ACK for DAT packets.
            next_tx_header = PACKET_ACK;
            break;
        case PACKET_FIN:
            ESP_LOGI(TAG, "Received FIN packet");
            gpio_set_level(LED_FIN, 1);
            next_tx_header = 0xFF;
            break;
        case PACKET_FUC:
            ESP_LOGI(TAG, "Received FUC (error) packet");
            gpio_set_level(LED_FUC, 1);
            next_tx_header = 0xFF;
            break;
        default:
            ESP_LOGW(TAG, "Received unknown packet type: 0x%02X", header);
            next_tx_header = 0xFF;
            break;
    }
    // Optionally, you might add code to turn off the LEDs after a delay.
}

/********************************************************************
 * SPI Slave Receive Task
 ********************************************************************/

// This task continuously waits for an SPI transaction from the master.
void spi_slave_receive_task(void *arg)
{
    while (1) {
        spi_slave_transaction_t trans = {
            .length = MAX_PACKET_SIZE * 8,  // transaction length in bits
            .tx_buffer = slave_tx_buffer,
            .rx_buffer = slave_rx_buffer,
        };

        // Block until a transaction is initiated by the master.
        esp_err_t ret = spi_slave_transmit(VSPI_HOST, &trans, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI slave transmit error: %s", esp_err_to_name(ret));
        }
        // The post_trans callback is invoked automatically after each transaction.
    }
}

/********************************************************************
 * Application Main Entry Point
 ********************************************************************/
void app_main(void)
{
    ESP_LOGI(TAG, "Initializing SPI Slave Node...");

    // Initialize LED GPIOs
    led_init();

    // Configure SPI bus for slave mode
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_PACKET_SIZE,
    };

    // Configure SPI slave interface with callbacks
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = spi_slave_post_setup_callback,
        .post_trans_cb = spi_slave_post_trans_callback,
    };

    // Initialize SPI slave on VSPI_HOST using DMA channel 1
    esp_err_t ret = spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPI Slave Initialized");

    // Create the SPI slave receive task
    xTaskCreate(spi_slave_receive_task, "spi_slave_receive_task", 4096, NULL, 5, NULL);
}
