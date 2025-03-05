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

// Packet type definitions
#define PACKET_ACK    0x01  // ACK response
#define PACKET_DAT    0x02  // Data packet
#define PACKET_FIN    0x03  // End of stream
#define PACKET_FUC    0x04  // Error condition

// Maximum packet size (1 header byte + up to 1024 bytes payload)
#define MAX_PACKET_SIZE  (1024 + 1)

// Global SPI slave buffers (must be DMA-capable)
static uint8_t slave_tx_buffer[MAX_PACKET_SIZE];
static uint8_t slave_rx_buffer[MAX_PACKET_SIZE];

// Global variable to hold the header for the next transaction
// When a DAT packet is received, we prepare an ACK (PACKET_ACK) for the next transaction.
static uint8_t next_tx_header = 0xFF;  // Default: no valid header

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
// Here we load the TX buffer with the header (e.g. ACK) that should be sent in the next transaction.
void spi_slave_post_setup_callback(spi_slave_transaction_t *trans)
{
    slave_tx_buffer[0] = next_tx_header;
    ESP_LOGI(TAG, "Post-Setup: Prepared TX header 0x%02X", next_tx_header);
}

// Called after a transaction completes.
void spi_slave_post_trans_callback(spi_slave_transaction_t *trans)
{
    // Check how many bits were transferred.
    // If only 8 bits were transferred, assume this was the master reading our prepared ACK.
    if (trans->trans_len == 8) {
         ESP_LOGI(TAG, "Response transaction completed; sent ACK: 0x%02X", slave_tx_buffer[0]);
         return;  // Nothing further to process.
    }
    
    // Otherwise, process the received packet.
    uint8_t header = slave_rx_buffer[0];
    ESP_LOGI(TAG, "Data transaction received; header: 0x%02X", header);
    
    switch (header) {
        case PACKET_ACK:
            ESP_LOGI(TAG, "START packet detected (interpreted as ACK from master)");
            gpio_set_level(LED_START, 1);
            next_tx_header = 0xFF;  // No response needed
            break;
        case PACKET_DAT:
            ESP_LOGI(TAG, "DATA packet (DAT) received");
            gpio_set_level(LED_DAT, 1);
            // Prepare ACK for next transaction so the master gets confirmation.
            next_tx_header = PACKET_ACK;
            break;
        case PACKET_FIN:
            ESP_LOGI(TAG, "FIN packet received");
            gpio_set_level(LED_FIN, 1);
            next_tx_header = 0xFF;
            break;
        case PACKET_FUC:
            ESP_LOGI(TAG, "FUC (error) packet received");
            gpio_set_level(LED_FUC, 1);
            next_tx_header = 0xFF;
            break;
        default:
            ESP_LOGW(TAG, "Unknown packet type: 0x%02X", header);
            next_tx_header = 0xFF;
            break;
    }
    // (Optionally, you may wish to clear or reset LED states after some time.)
}

/********************************************************************
 * SPI Slave Receive Task
 ********************************************************************/
void spi_slave_receive_task(void *arg)
{
    while (1) {
        // We use the maximum packet size as the buffer length.
        spi_slave_transaction_t trans = {0};
        trans.length = MAX_PACKET_SIZE * 8;  // in bits; actual transaction length is determined by the master.
        trans.tx_buffer = slave_tx_buffer;
        trans.rx_buffer = slave_rx_buffer;
        
        // Wait (block) until the master initiates a transaction.
        esp_err_t ret = spi_slave_transmit(VSPI_HOST, &trans, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI slave transmit error: %s", esp_err_to_name(ret));
        }
        // The callbacks automatically process the transaction.
    }
}


esp_err_t spi_slave_init(void) {
    ESP_LOGI(TAG, "Initializing SPI Slave Node...");
    
    // Initialize LED GPIOs
    
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
}

/********************************************************************
 * Application Main Entry Point
 ********************************************************************/
void app_main(void)
{
    led_init();
    spi_slave_init();

    // Create the SPI slave receive task
    xTaskCreate(spi_slave_receive_task, "spi_slave_receive_task", 4096, NULL, 5, NULL);
}
