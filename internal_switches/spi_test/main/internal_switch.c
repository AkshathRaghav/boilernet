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

// ######################################################################
// Pin definitions and constants – DO NOT CHANGE
#define PIN_NUM_MOSI  13
#define PIN_NUM_MISO  12
#define PIN_NUM_CLK   14
#define PIN_NUM_CS    15
#define GPIO_LED      2

#define MSG_BUF_SIZE  1

#define PACKET_START_WRITE 0xA5  // SPI command to indicate start writing
#define PACKET_DAT         0xA6  // SPI data packet (same as before)
#define PACKET_MID         0xA7  // SPI middle packet
#define PACKET_END         0xA8  // SPI finish packet
#define PACKET_FUC         0xF9  // SPI error condition

#define SPI_HOST_TYPE HSPI_HOST
// ######################################################################

static const char *TAG = "SLAVE";

// Slave FSM states.
typedef enum {
    SLAVE_FSM_IDLE,
    SLAVE_FSM_WAIT_START_CONFIRM,
    SLAVE_FSM_WAIT_MID_CONFIRM,
    SLAVE_FSM_WAIT_END_CONFIRM,
    SLAVE_FSM_DATA_RECEIVE
} slave_fsm_state_t;
static volatile slave_fsm_state_t slave_state = SLAVE_FSM_IDLE;

static uint8_t tx_buffer[MSG_BUF_SIZE];
static uint8_t rx_buffer[MSG_BUF_SIZE];

static void prepare_tx_buffer(uint8_t header)
{
    memset(tx_buffer, 0, MSG_BUF_SIZE);
    tx_buffer[0] = header;
}

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
        .max_transfer_sz = MSG_BUF_SIZE  // Ensure the buffer is large enough
    };

    // Configure SPI slave interface.
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL
    };

    // Enable pull-ups on the SPI bus lines.
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS,   GPIO_PULLUP_ONLY);

    // Configure the LED GPIO as an output.
    gpio_config_t led_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(GPIO_LED),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&led_conf);

    // Initialize the SPI slave.
    ret = spi_slave_initialize(SPI_HOST_TYPE, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPI slave initialized.");

    uint8_t next_tx_header = PACKET_START_WRITE;
    // uint8_t *tx_buffer = heap_caps_malloc(MSG_BUF_SIZE, MALLOC_CAP_DMA);
    // uint8_t *rx_buffer = heap_caps_malloc(MSG_BUF_SIZE, MALLOC_CAP_DMA);
    // if (!tx_buffer || !rx_buffer) {
    //     ESP_LOGE(TAG, "Failed to allocate DMA buffers");
    //     if (tx_buffer) free(tx_buffer);
    //     if (rx_buffer) free(rx_buffer);
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }

    // spi_slave_transaction_t trans;
    // memset(&trans, 0, sizeof(trans));

    while (1) {
        prepare_tx_buffer(next_tx_header);
        memset(rx_buffer, 0, MSG_BUF_SIZE);

        spi_slave_transaction_t trans;
        memset(&trans, 0, sizeof(trans));
        trans.length    = MSG_BUF_SIZE * 8; // in bits
        trans.tx_buffer = tx_buffer;
        trans.rx_buffer = rx_buffer;

        ret = spi_slave_queue_trans(SPI_HOST_TYPE, &trans, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to queue SPI transaction: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        spi_slave_transaction_t *completed_trans = NULL;
        ret = spi_slave_get_trans_result(SPI_HOST_TYPE, &completed_trans, portMAX_DELAY);
        if (ret != ESP_OK || completed_trans != &trans) {
            ESP_LOGE(TAG, "Failed to get SPI transaction result: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        uint8_t received_cmd = rx_buffer[0];
        ESP_LOGI(TAG, "Transaction complete. Received command: 0x%02X", received_cmd);

        switch (slave_state) {
            case SLAVE_FSM_IDLE:
                if (received_cmd == PACKET_START_WRITE) {
                    ESP_LOGI(TAG, "SLAVE: Received START command. Waiting for confirmation...");
                    slave_state      = SLAVE_FSM_WAIT_START_CONFIRM;
                    next_tx_header   = PACKET_START_WRITE;  // Echo next time
                }
                else if (received_cmd == PACKET_DAT) {
                    ESP_LOGI(TAG, "SLAVE: Received DATA packet.");
                    slave_state      = SLAVE_FSM_DATA_RECEIVE;
                    next_tx_header   = 0xDD; // No special confirmation needed
                }
                else if (received_cmd == PACKET_MID) {
                    ESP_LOGI(TAG, "SLAVE: Received MID command. Waiting for confirmation...");
                    slave_state      = SLAVE_FSM_WAIT_MID_CONFIRM;
                    next_tx_header   = PACKET_MID;
                }
                else if (received_cmd == PACKET_END) {
                    ESP_LOGI(TAG, "SLAVE: Received END command. Waiting for confirmation...");
                    slave_state      = SLAVE_FSM_WAIT_END_CONFIRM;
                    next_tx_header   = PACKET_END;
                }
                else {
                    ESP_LOGW(TAG, "SLAVE: Unknown command 0x%02X in IDLE state.", received_cmd);
                    next_tx_header   = 0xDD;
                }
                break;

            case SLAVE_FSM_WAIT_START_CONFIRM:
                if (received_cmd == PACKET_START_WRITE) {
                    ESP_LOGI(TAG, "SLAVE: START command confirmed. Switching to data receive.");
                    slave_state      = SLAVE_FSM_DATA_RECEIVE;
                    next_tx_header   = 0xDD;
                } else {
                    ESP_LOGW(TAG, "SLAVE: Unexpected cmd 0x%02X while WAIT_START_CONFIRM.", received_cmd);
                    next_tx_header   = 0xDD;
                }
                break;

            case SLAVE_FSM_WAIT_MID_CONFIRM:
                if (received_cmd == PACKET_MID) {
                    ESP_LOGI(TAG, "SLAVE: MID command confirmed. Switching to data receive.");
                    slave_state      = SLAVE_FSM_DATA_RECEIVE;
                    next_tx_header   = 0xDD;
                } else {
                    ESP_LOGW(TAG, "SLAVE: Unexpected cmd 0x%02X while WAIT_MID_CONFIRM.", received_cmd);
                    next_tx_header   = 0xDD;
                }
                break;

            case SLAVE_FSM_WAIT_END_CONFIRM:
                if (received_cmd == PACKET_END) {
                    ESP_LOGI(TAG, "SLAVE: END command confirmed. Transfer complete.");
                    slave_state      = SLAVE_FSM_IDLE;
                    next_tx_header   = 0xDD;
                } else {
                    ESP_LOGW(TAG, "SLAVE: Unexpected cmd 0x%02X while WAIT_END_CONFIRM.", received_cmd);
                    next_tx_header   = 0xDD;
                }
                break;

            case SLAVE_FSM_DATA_RECEIVE:
                if (received_cmd == PACKET_DAT) {
                    ESP_LOGI(TAG, "SLAVE: DATA packet received in DATA state.");
                    // Optionally log or process the entire data payload.
                    // e.g. ESP_LOG_BUFFER_HEX("DATA", rx_buffer, MSG_BUF_SIZE);
                    next_tx_header   = 0xDD;
                } else {
                    ESP_LOGW(TAG, "SLAVE: Unexpected cmd 0x%02X in DATA state, echoing it.", received_cmd);
                    // If you want to echo unexpected commands:
                    next_tx_header   = received_cmd;
                }
                break;

            default:
                next_tx_header = 0xDD;
                break;
        }

        // ------------------------------------------------------------------
        // 5. (Optional) Visual feedback
        // ------------------------------------------------------------------
        gpio_set_level(GPIO_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(GPIO_LED, 0);
    }
}
