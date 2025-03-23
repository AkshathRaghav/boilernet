#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

// -----------------------------------------------------------------------------
// Pin definitions and constants
// -----------------------------------------------------------------------------
#define PIN_NUM_MOSI      13
#define PIN_NUM_MISO      12
#define PIN_NUM_CLK       15
#define PIN_NUM_CS        14
#define GPIO_LED          2

#define MSG_BUF_SIZE      2

// SPI command definitions
#define PACKET_START_WRITE 0xA5  // Command to indicate start writing
#define PACKET_DAT         0xA6  // Data packet
#define PACKET_MID         0xA7  // Middle packet
#define PACKET_END         0xA8  // Finish packet
#define PACKET_FUC         0xF9  // Error condition

#define SPI_HOST_TYPE     HSPI_HOST

static const char *TAG = "SPI_SLAVE";

// -----------------------------------------------------------------------------
// Slave FSM States
// -----------------------------------------------------------------------------
typedef enum {
    SLAVE_FSM_IDLE,
    SLAVE_FSM_WAIT_START_CONFIRM,
    SLAVE_FSM_WAIT_MID_CONFIRM,
    SLAVE_FSM_WAIT_END_CONFIRM,
    SLAVE_FSM_DATA_RECEIVE
} slave_fsm_state_t;

static volatile slave_fsm_state_t slave_state = SLAVE_FSM_IDLE;

// -----------------------------------------------------------------------------
// DMA-capable buffers (place in DRAM)
// -----------------------------------------------------------------------------
static DRAM_ATTR uint8_t tx_buffer[MSG_BUF_SIZE];
static DRAM_ATTR uint8_t rx_buffer[MSG_BUF_SIZE];

// -----------------------------------------------------------------------------
// Helper: Prepare the transmit buffer with a header value
// -----------------------------------------------------------------------------
static void prepare_tx_buffer(uint8_t header)
{
    memset(tx_buffer, 0, MSG_BUF_SIZE);
    tx_buffer[0] = header;
}

// -----------------------------------------------------------------------------
// Main application
// -----------------------------------------------------------------------------
void app_main(void)
{
    esp_err_t ret;

    // Configure SPI bus for the slave.
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MSG_BUF_SIZE,
    };

    // Configure SPI slave interface.
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL,
    };

    // Enable pull-ups on SPI bus lines.
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS,   GPIO_PULLUP_ONLY);

    // Configure LED for visual feedback.
    gpio_config_t led_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(GPIO_LED),
        .pull_up_en = 0,
        .pull_down_en = 0,
    };
    gpio_config(&led_conf);

    // Initialize SPI slave.
    ret = spi_slave_initialize(SPI_HOST_TYPE, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPI slave initialized.");

    // The next header value to transmit (e.g. for command confirmations).
    uint8_t next_tx_header = PACKET_START_WRITE;

    while (1) {
        // Prepare buffers for this transaction.
        prepare_tx_buffer(next_tx_header);
        memset(rx_buffer, 0, MSG_BUF_SIZE);

        spi_slave_transaction_t trans;
        memset(&trans, 0, sizeof(trans));
        trans.length    = MSG_BUF_SIZE * 8; // Total transfer size in bits.
        trans.tx_buffer = tx_buffer;
        trans.rx_buffer = rx_buffer;

        // Queue the SPI transaction.
        ret = spi_slave_queue_trans(SPI_HOST_TYPE, &trans, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to queue SPI transaction: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Wait for the transaction to complete.
        spi_slave_transaction_t *completed_trans = NULL;
        ret = spi_slave_get_trans_result(SPI_HOST_TYPE, &completed_trans, portMAX_DELAY);
        if (ret != ESP_OK || completed_trans != &trans) {
            ESP_LOGE(TAG, "Failed to get SPI transaction result: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Process the received command from the master.
        uint8_t received_cmd = rx_buffer[0];
        ESP_LOGI(TAG, "Transaction complete. Received command: 0x%02X", received_cmd);

        // Update the slave state machine based on the received command.
        switch (slave_state) {
            case SLAVE_FSM_IDLE:
                if (received_cmd == PACKET_START_WRITE) {
                    ESP_LOGI(TAG, "SLAVE: Received START command. Waiting for confirmation...");
                    slave_state    = SLAVE_FSM_WAIT_START_CONFIRM;
                    next_tx_header = PACKET_START_WRITE;  // Echo back for confirmation.
                } else if (received_cmd == PACKET_DAT) {
                    ESP_LOGI(TAG, "SLAVE: Received DATA packet.");
                    slave_state    = SLAVE_FSM_DATA_RECEIVE;
                    next_tx_header = 0xA5;  // No confirmation needed.
                } else if (received_cmd == PACKET_MID) {
                    ESP_LOGI(TAG, "SLAVE: Received MID command. Waiting for confirmation...");
                    slave_state    = SLAVE_FSM_WAIT_MID_CONFIRM;
                    next_tx_header = PACKET_MID;
                } else if (received_cmd == PACKET_END) {
                    ESP_LOGI(TAG, "SLAVE: Received END command. Waiting for confirmation...");
                    slave_state    = SLAVE_FSM_WAIT_END_CONFIRM;
                    next_tx_header = PACKET_END;
                } else {
                    ESP_LOGW(TAG, "SLAVE: Unknown command 0x%02X in IDLE state.", received_cmd);
                    next_tx_header = 0xA5;
                }
                break;

            case SLAVE_FSM_WAIT_START_CONFIRM:
                if (received_cmd == PACKET_START_WRITE) {
                    ESP_LOGI(TAG, "SLAVE: START command confirmed. Switching to data receive.");
                    slave_state    = SLAVE_FSM_DATA_RECEIVE;
                    next_tx_header = 0xA5;
                } else {
                    ESP_LOGW(TAG, "SLAVE: Unexpected cmd 0x%02X during START confirmation.", received_cmd);
                    next_tx_header = 0xA5;
                }
                break;

            case SLAVE_FSM_WAIT_MID_CONFIRM:
                if (received_cmd == PACKET_MID) {
                    ESP_LOGI(TAG, "SLAVE: MID command confirmed. Switching to data receive.");
                    slave_state    = SLAVE_FSM_DATA_RECEIVE;
                    next_tx_header = 0xA5;
                } else {
                    ESP_LOGW(TAG, "SLAVE: Unexpected cmd 0x%02X during MID confirmation.", received_cmd);
                    next_tx_header = 0xA5;
                }
                break;

            case SLAVE_FSM_WAIT_END_CONFIRM:
                if (received_cmd == PACKET_END) {
                    ESP_LOGI(TAG, "SLAVE: END command confirmed. Transfer complete.");
                    slave_state    = SLAVE_FSM_IDLE;
                    next_tx_header = 0xA5;
                } else {
                    ESP_LOGW(TAG, "SLAVE: Unexpected cmd 0x%02X during END confirmation.", received_cmd);
                    next_tx_header = 0xA5;
                }
                break;

            case SLAVE_FSM_DATA_RECEIVE:
                if (received_cmd == PACKET_DAT) {
                    ESP_LOGI(TAG, "SLAVE: DATA packet received in DATA state.");
                    // Here you can process the received data as needed.
                    next_tx_header = 0xA5;
                } else {
                    ESP_LOGW(TAG, "SLAVE: Unexpected cmd 0x%02X in DATA state. Echoing received cmd.", received_cmd);
                    next_tx_header = received_cmd;
                }
                break;

            default:
                next_tx_header = 0xDD;
                break;
        }

        // Provide visual feedback via LED blink.
        gpio_set_level(GPIO_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(GPIO_LED, 0);
    }
}
