#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// SPI includes
#include "driver/spi_master.h"
#include "driver/gpio.h"

//-------------------------------------------------------------
// Ethernet protocol definitions
#define PACKET_TYPE_START 0x01
#define PACKET_TYPE_DATA  0x02
#define PACKET_TYPE_MID   0x03
#define PACKET_TYPE_END   0x04

#define PORT 8080
static const char *TAG = "TCP_SERVER";

//-------------------------------------------------------------
// Global variables for checksum accumulation
unsigned int computed_checksum = 0;

//-------------------------------------------------------------
// SPI definitions and globals
#define GPIO_MOSI   13
#define GPIO_MISO   12
#define GPIO_SCLK   14
#define GPIO_CS1    21
#define GPIO_CS2    5

// SPI transaction chunk size (adjust as needed)
#define SPI_CHUNK_SIZE  256

// SPI command for ending a transmission (if required)
#define SPI_CMD_END1 "END1"
#define SPI_CMD_END2 "END2"
#define SPI_ACK      "ACK"

// SPI device handles for two slaves
spi_device_handle_t spi_slave1 = NULL;
spi_device_handle_t spi_slave2 = NULL;

// Current SPI slave handle pointer (initially slave1)
spi_device_handle_t current_slave = NULL;

//-------------------------------------------------------------
// Ethernet Event Handlers and Initialization (similar to your sample)
static EventGroupHandle_t s_eth_event_group;
#define ETHERNET_CONNECTED_BIT (1 << 0)
#define ETHERNET_FAIL_BIT      (1 << 1)

#define ETH_PHY_ADDR      1
#define ETH_PHY_RST_GPIO  -1
#define ETH_MDC_GPIO      23
#define ETH_MDIO_GPIO     18
#define ETH_TAG           "ETH"

// Ethernet event handler
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
            ESP_LOGI(ETH_TAG, "Ethernet Link Up, MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                     mac_addr[0], mac_addr[1], mac_addr[2],
                     mac_addr[3], mac_addr[4], mac_addr[5]);
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGI(ETH_TAG, "Ethernet Link Down");
            xEventGroupSetBits(s_eth_event_group, ETHERNET_FAIL_BIT);
            break;
        default:
            break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;
    ESP_LOGI(ETH_TAG, "Got IP: " IPSTR, IP2STR(&ip_info->ip));
    xEventGroupSetBits(s_eth_event_group, ETHERNET_CONNECTED_BIT);
}

static esp_eth_handle_t eth_init_internal(esp_eth_mac_t **mac_out, esp_eth_phy_t **phy_out)
{
    esp_eth_handle_t ret = NULL;
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_PHY_RST_GPIO;
    // Configure EMAC specifics for ESP32
    eth_esp32_emac_config_t esp32_emac_config = {
        .smi_mdc_gpio_num  = ETH_MDC_GPIO,                       
        .smi_mdio_gpio_num = ETH_MDIO_GPIO,                      
        .interface = EMAC_DATA_INTERFACE_RMII,        
        .clock_config = { .rmii = { .clock_mode = EMAC_CLK_OUT, .clock_gpio = -1 } },
        .dma_burst_len = ETH_DMA_BURST_LEN_32 
    };
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
    esp_eth_handle_t eth_handle = NULL;
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    if (esp_eth_driver_install(&config, &eth_handle) != ESP_OK) {
        ESP_LOGE(ETH_TAG, "Ethernet driver install failed");
        goto err;
    }
    if (mac_out) *mac_out = mac;
    if (phy_out) *phy_out = phy;
    return eth_handle;
err:
    if (eth_handle) esp_eth_driver_uninstall(eth_handle);
    if (mac) mac->del(mac);
    if (phy) phy->del(phy);
    return NULL;
}

void ethernet_setup(void)
{
    s_eth_event_group = xEventGroupCreate();
    esp_eth_handle_t eth_handle = eth_init_internal(NULL, NULL);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    // For this example, we assume a static IP (adjust if needed)
    if (esp_netif_dhcpc_stop(eth_netif) != ESP_OK) {
        ESP_LOGE(ETH_TAG, "Failed to stop DHCP client");
        return;
    }
    esp_netif_ip_info_t ip_info = {0};
    IP4_ADDR(&ip_info.ip, 192,168,0,50);
    IP4_ADDR(&ip_info.gw, 192,168,0,1);
    IP4_ADDR(&ip_info.netmask, 255,255,255,0);
    if (esp_netif_set_ip_info(eth_netif, &ip_info) != ESP_OK) {
        ESP_LOGE(ETH_TAG, "Failed to set static IP info");
    } else {
        ESP_LOGI(ETH_TAG, "Static IP set");
    }
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    EventBits_t bits = xEventGroupWaitBits(s_eth_event_group,
                                           ETHERNET_CONNECTED_BIT | ETHERNET_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & ETHERNET_CONNECTED_BIT) {
        ESP_LOGI(ETH_TAG, "Ethernet connected.");
    } else {
        ESP_LOGE(ETH_TAG, "Ethernet connection failed.");
    }
}

//-------------------------------------------------------------
// SPI helper: send data in SPI_CHUNK_SIZE chunks
esp_err_t spi_send_data(spi_device_handle_t spi, const uint8_t *data, size_t length)
{
    esp_err_t ret;
    size_t offset = 0;
    while (offset < length) {
        size_t chunk = ((length - offset) > SPI_CHUNK_SIZE) ? SPI_CHUNK_SIZE : (length - offset);
        spi_transaction_t t = {0};
        t.length = chunk * 8;
        t.tx_buffer = data + offset;
        uint8_t rx_dummy[chunk];
        t.rx_buffer = rx_dummy;
        ret = spi_device_transmit(spi, &t);
        if (ret != ESP_OK) {
            ESP_LOGE("SPI_SEND", "SPI transmit failed at offset %d", offset);
            return ret;
        }
        offset += chunk;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_OK;
}

// Optionally, if you need to send an END command and then a dummy transaction to get the ACK:
esp_err_t spi_send_command_with_ack(spi_device_handle_t spi, const char *cmd)
{
    esp_err_t ret;
    spi_transaction_t t = {0};
    size_t cmd_len = strlen(cmd);
    t.length = cmd_len * 8;
    t.tx_buffer = cmd;
    uint8_t rx_dummy[cmd_len];
    t.rx_buffer = rx_dummy;
    ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI_CMD", "Failed to send command %s", cmd);
        return ret;
    }
    // Send dummy transaction to retrieve ACK from previous transmission.
    memset(&t, 0, sizeof(t));
    size_t ack_len = strlen(SPI_ACK);
    t.length = ack_len * 8;
    uint8_t dummy_tx[ack_len];
    memset(dummy_tx, 0, sizeof(dummy_tx));
    t.tx_buffer = dummy_tx;
    uint8_t rx_ack[ack_len+1];
    memset(rx_ack, 0, sizeof(rx_ack));
    t.rx_buffer = rx_ack;
    ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI_CMD", "Dummy TX for ACK failed");
        return ret;
    }
    if (strncmp((char *)rx_ack, SPI_ACK, ack_len) != 0) {
        ESP_LOGE("SPI_CMD", "Invalid ACK received: %s", rx_ack);
        return ESP_FAIL;
    }
    ESP_LOGI("SPI_CMD", "Received valid ACK for command %s", cmd);
    return ESP_OK;
}

//-------------------------------------------------------------
// TCP Server Task: process packets “hot potato” style
static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    struct sockaddr_storage dest_addr;
    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);

    int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    ESP_LOGI(TAG, "Socket created");

    if (bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "Error during listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Waiting for a connection...");
    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);
    int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    inet_ntop(source_addr.ss_family, &((struct sockaddr_in *)&source_addr)->sin_addr,
              addr_str, sizeof(addr_str));
    ESP_LOGI(TAG, "Connection accepted from %s", addr_str);

    // Set the current SPI slave to slave1 initially.
    current_slave = spi_slave1;

    // FSM for packet processing.
    typedef enum {
        FSM_WAIT_START,
        FSM_WAIT_DATA,
        FSM_WAIT_END,
        FSM_COMPLETE,
        FSM_ERROR
    } fsm_state_t;
    fsm_state_t state = FSM_WAIT_START;

    while (state != FSM_COMPLETE && state != FSM_ERROR) {
        uint8_t pkt_type;
        if (recv(sock, &pkt_type, 1, 0) != 1) {
            ESP_LOGE(TAG, "Failed to read packet type");
            state = FSM_ERROR;
            break;
        }

        switch (pkt_type) {
            case PACKET_TYPE_START:
            {
                if (state != FSM_WAIT_START) {
                    ESP_LOGE(TAG, "Unexpected START packet");
                    state = FSM_ERROR;
                    break;
                }
                // For START, read header (width, height, total data length)
                uint8_t header[8];
                if (recv(sock, header, sizeof(header), 0) != sizeof(header)) {
                    ESP_LOGE(TAG, "Failed to read START payload");
                    state = FSM_ERROR;
                    break;
                }
                uint16_t width = (header[0] << 8) | header[1];
                uint16_t height = (header[2] << 8) | header[3];
                uint32_t total_length = (header[4] << 24) | (header[5] << 16) |
                                        (header[6] << 8)  | header[7];
                ESP_LOGI(TAG, "START: Image %dx%d, total %d bytes", width, height, total_length);
                // Send ACK for START (simply echo the packet type)
                if (send(sock, &pkt_type, 1, 0) != 1) {
                    ESP_LOGE(TAG, "Failed to send START ACK");
                    state = FSM_ERROR;
                    break;
                }
                state = FSM_WAIT_DATA;
                break;
            }
            case PACKET_TYPE_DATA:
            {
                // Read DATA packet length (2 bytes)
                uint8_t len_buf[2];
                if (recv(sock, len_buf, sizeof(len_buf), 0) != sizeof(len_buf)) {
                    ESP_LOGE(TAG, "Failed to read DATA length");
                    state = FSM_ERROR;
                    break;
                }
                uint16_t data_len = (len_buf[0] << 8) | len_buf[1];
                if (data_len > 0) {
                    uint8_t *data_buf = malloc(data_len);
                    if (!data_buf) {
                        ESP_LOGE(TAG, "Memory allocation failed");
                        state = FSM_ERROR;
                        break;
                    }
                    if (recv(sock, data_buf, data_len, 0) != data_len) {
                        ESP_LOGE(TAG, "Failed to read DATA payload");
                        free(data_buf);
                        state = FSM_ERROR;
                        break;
                    }
                    // Update running checksum
                    for (int i = 0; i < data_len; i++) {
                        computed_checksum += data_buf[i];
                    }
                    // Immediately send the received data over SPI using current_slave.
                    if (spi_send_data(current_slave, data_buf, data_len) != ESP_OK) {
                        ESP_LOGE(TAG, "SPI transfer failed");
                        free(data_buf);
                        state = FSM_ERROR;
                        break;
                    }
                    free(data_buf);
                    // Send Data ACK (echo the packet type or a specific ACK byte)
                    if (send(sock, &pkt_type, 1, 0) != 1) {
                        ESP_LOGE(TAG, "Failed to send DATA ACK");
                        state = FSM_ERROR;
                        break;
                    }
                }
                break;
            }
            case PACKET_TYPE_MID:
            {
                // MID packet: read any reserved bytes (e.g., 2 bytes) if protocol requires.
                uint8_t mid_buf[2];
                if (recv(sock, mid_buf, sizeof(mid_buf), 0) != sizeof(mid_buf)) {
                    ESP_LOGE(TAG, "Failed to read MID payload");
                    state = FSM_ERROR;
                    break;
                }
                // Switch current SPI slave from slave1 to slave2.
                current_slave = spi_slave2;
                ESP_LOGI(TAG, "MID received, switching SPI output to slave2");
                // Send MID ACK
                if (send(sock, &pkt_type, 1, 0) != 1) {
                    ESP_LOGE(TAG, "Failed to send MID ACK");
                    state = FSM_ERROR;
                    break;
                }
                break;
            }
            case PACKET_TYPE_END:
            {
                // END packet: read 4-byte checksum.
                uint8_t end_buf[4];
                if (recv(sock, end_buf, sizeof(end_buf), 0) != sizeof(end_buf)) {
                    ESP_LOGE(TAG, "Failed to read END payload");
                    state = FSM_ERROR;
                    break;
                }
                unsigned int received_checksum = (end_buf[0] << 24) | (end_buf[1] << 16) |
                                                 (end_buf[2] << 8)  | end_buf[3];
                ESP_LOGI(TAG, "END received. Checksum: received=0x%08X computed=0x%08X",
                         received_checksum, computed_checksum);
                if (received_checksum != computed_checksum) {
                    ESP_LOGE(TAG, "Checksum mismatch");
                    state = FSM_ERROR;
                } else {
                    // Send END ACK
                    if (send(sock, &pkt_type, 1, 0) != 1) {
                        ESP_LOGE(TAG, "Failed to send END ACK");
                        state = FSM_ERROR;
                        break;
                    }
                    state = FSM_COMPLETE;
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Unknown packet type: 0x%02X", pkt_type);
                state = FSM_ERROR;
                break;
        } // end switch
    } // end while

    close(sock);
    close(listen_sock);
    ESP_LOGI(TAG, "TCP connection closed.");
    vTaskDelete(NULL);
}

//-------------------------------------------------------------
// Main application entry point
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ethernet_setup();

    // Initialize SPI bus and add both slave devices.
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    // Use SPI2_HOST (adjust if needed)
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t devcfg1 = {
        .clock_speed_hz = 15000000,
        .mode = 0,
        .spics_io_num = GPIO_CS1,
        .queue_size = 3,
    };
    spi_device_interface_config_t devcfg2 = {
        .clock_speed_hz = 15000000,
        .mode = 0,
        .spics_io_num = GPIO_CS2,
        .queue_size = 3,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg1, &spi_slave1));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg2, &spi_slave2));

    // Create the TCP server task.
    xTaskCreate(tcp_server_task, "tcp_server", 8192, NULL, 5, NULL);
    ESP_LOGI("APP_MAIN", "Initialization complete; waiting for Ethernet data...");
}
