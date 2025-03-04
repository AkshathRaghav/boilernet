#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST HSPI_HOST
#else
#define RCV_HOST SPI2_HOST
#endif

// Shared SPI bus pins.
#define GPIO_MOSI   23
#define GPIO_MISO   19
#define GPIO_SCLK   18
// CS
#define GPIO_CS     5
//#define GPIO_CS     17
// Use a dedicated LED for Slave 1.
#define GPIO_LED    11

#define MASTER_MSG  "BUTTON_PRESSED"
#define SLAVE_ACK   "ACK_BUTTON_PRESSED"

//static const char *TAG = "Slave1";
static const char *TAG = "Slave2";

void app_main(void)
{
    esp_err_t ret;

    // Configure SPI bus for the slave.
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // Configure SPI slave interface.
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL
    };

    // Enable pull-ups on the SPI bus lines.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS,   GPIO_PULLUP_ONLY);

    // Configure the LED GPIO as an output.
    gpio_config_t led_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(GPIO_LED),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&led_conf);

    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    WORD_ALIGNED_ATTR char sendbuf[129] = "";
    WORD_ALIGNED_ATTR char recvbuf[129] = "";
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    ESP_LOGI(TAG, "Ready. Waiting for master's message...\n");

    while (1) {
        // Preload the transmit buffer with the acknowledgement.
        memset(sendbuf, 0, sizeof(sendbuf));
        strcpy(sendbuf, SLAVE_ACK);
        memset(recvbuf, 0, sizeof(recvbuf));

        t.length = 128 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;

        // Wait for the SPI transaction from the master.
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
        assert(ret == ESP_OK);

        if (strstr(recvbuf, MASTER_MSG) != NULL) {
            ESP_LOGI(TAG, "Received BUTTON_PRESSED from master.\n");
            // Light up the LED for 500ms.
            gpio_set_level(GPIO_LED, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(GPIO_LED, 0);
        } else {
            ESP_LOGI(TAG, "Received unknown message: %s\n", recvbuf);
        }
    }
}