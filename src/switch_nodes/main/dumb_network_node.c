#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"  // for esp_random()
#include "esp_random.h"
#define TAG "SPI_MASTER"

// SPI pins (must match the slave configuration)
#define PIN_NUM_MOSI  13
#define PIN_NUM_MISO  12
#define PIN_NUM_CLK   14
#define PIN_NUM_CS    15

// Handshake pin number (slave drives this, master reads it)
#define GPIO_HANDSHAKE 4

// Transfer size: 2048 bytes.
#define TRANSFER_SIZE 2048

// Define two example command values (not used in the random payload).
#define CMD_VALID   0xA6  
#define CMD_INVALID 0xB1  

// Maximum retry attempts.
#define MAX_RETRY 5

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
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 5000000,  // 5 MHz
        .duty_cycle_pos = 128,      // 50% duty cycle
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .cs_ena_posttrans = 3,      // Keep CS low a few extra cycles after transfer
        .queue_size = 3
    };

    ret = spi_bus_add_device(SPI_HOST_TYPE, &devcfg, &spi_master_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
    }
    return ret;
}

void handshake_init_master(void)
{
    // Configure handshake pin as input with pull-down.
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
}

void wait_for_slave_ready(void)
{
    // Wait until the handshake pin goes high, indicating the slave is ready.
    while(gpio_get_level(GPIO_HANDSHAKE) == 0) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void app_main(void)
{
    esp_err_t ret = spi_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI master initialization failed");
        return;
    }
    handshake_init_master();
    ESP_LOGI(TAG, "SPI master initialized");

    uint8_t tx_buf[TRANSFER_SIZE];
    uint8_t rx_buf[TRANSFER_SIZE];
    spi_transaction_t t = {0};
    int count = 0;

    while (1) {
        // Fill the TX buffer with a random pattern.
        // Use esp_random() for low-overhead hardware random number generation.
        // This produces an "unknown" pattern each time.
        for (int i = 0; i < TRANSFER_SIZE; i++) {
            tx_buf[i] = (uint8_t) esp_random();
            rx_buf[i] = (uint8_t)0; 
        }

        int attempt;
        bool success = false;
        for (attempt = 0; attempt < MAX_RETRY; attempt++) {

            // Wait for the slave's handshake signal.
            wait_for_slave_ready();
            
            // --- Transaction 1: Send the full buffer ---
            memset(&t, 0, sizeof(t));
            t.length = TRANSFER_SIZE * 8;
            t.tx_buffer = tx_buf;
            t.rx_buffer = NULL;  // We're only sending data.
            ret = spi_device_polling_transmit(spi_master_handle, &t);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "SPI transmit (send phase) failed on attempt %d: %s", attempt+1, esp_err_to_name(ret));
                continue;
            }
            ESP_LOGI(TAG, "Attempt %d: Sent random buffer", attempt+1);

            // --- Transaction 2: Read echo from the slave ---
            memset(&t, 0, sizeof(t));
            t.length = TRANSFER_SIZE * 8;
            t.tx_buffer = tx_buf;  // Dummy TX data.
            t.rx_buffer = rx_buf;
            ret = spi_device_polling_transmit(spi_master_handle, &t);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "SPI transmit (echo read) failed on attempt %d: %s", attempt+1, esp_err_to_name(ret));
                continue;
            }
            
            // Compare received buffer with transmitted.
            int mismatches = 0;
            for (int i = 0; i < TRANSFER_SIZE; i++) {
                if (rx_buf[i] != tx_buf[i]) {
                    mismatches++;
                }
            }
            
            if (mismatches == 0) {
                ESP_LOGI(TAG, "\033[0;32mEcho successful on attempt %d: Received buffer matches transmitted buffer.\033[0m 0x%02X 0x%02X 0x%02X .. end 0x%02X 0x%02X ...", attempt+1, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[TRANSFER_SIZE-2], rx_buf[TRANSFER_SIZE-1]);
                success = true;
                break;
            } else {
                ESP_LOGW(TAG, "Attempt %d: Echo failed with %d mismatches.", attempt+1, mismatches);
                ESP_LOGI(TAG, "Returned values: 0x%02X 0x%02X 0x%02X ...", rx_buf[0], rx_buf[1], rx_buf[2]);
            }
            // Optional: small delay before retrying the same chunk.
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        if (!success) {
            ESP_LOGE(TAG, "Error: Maximum retry attempts reached. Echo still failed for this buffer.");
        }
        count++;
    }
}
