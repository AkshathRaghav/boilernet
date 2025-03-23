#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netdb.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

// ESP-IDF Components
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_eth_driver.h"
#include "esp_check.h"
#include "esp_mac.h"

#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_err.h"

// lwIP
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
#include "lwip/dns.h"

// ######################################################################

static EventGroupHandle_t s_eth_event_group;

#define ETHERNET_CONNECTED_BIT  BIT0
#define ETHERNET_FAIL_BIT       BIT1

#define STATIC_IP               1

#if STATIC_IP
    #define S_IP        "192.168.0.50"     
    #define GATEWAY     "192.168.0.1"    
    #define NETMASK     "255.255.255.0"
#endif /* STATIC_IP */

#define ETH_PHY_ADDR             1
#define ETH_PHY_RST_GPIO        -1          // not connected
#define ETH_MDC_GPIO            23
#define ETH_MDIO_GPIO           18
#define ETH_TAG                 "ETH"

#define PORT 8080

#define PACKET_TYPE_START  0xA0
#define PACKET_TYPE_DATA   0xA1
#define PACKET_TYPE_MID    0xA2
#define PACKET_TYPE_END    0xA3

static netif_input_fn default_input = NULL;

extern void ethernet_setup(void);  

// Ethernet FSM state.
typedef enum {
    FSM_WAIT_START,
    FSM_WAIT_DATA,
    FSM_WAIT_MID,
    FSM_WAIT_END,
    FSM_COMPLETE,
    FSM_ERROR
} fsm_state_t;

#define PACKET_START_WRITE 0xA5  // Command to indicate start writing
#define PACKET_DAT         0xA6  // Data packet
#define PACKET_MID         0xA7  // Middle packet
#define PACKET_END         0xA8  // Finish packet
#define PACKET_FUC         0xF9  // Error condition

// -----------------------------------------------------------------------------
// Pin definitions (must match slave's settings)
// -----------------------------------------------------------------------------
#define PIN_NUM_MOSI  13
#define PIN_NUM_MISO  12
#define PIN_NUM_CLK   15
#define PIN_NUM_CS    14

// -----------------------------------------------------------------------------
// Transfer size and retry parameters (must match slave configuration)
// -----------------------------------------------------------------------------
#define MSG_BUF_SIZE  128
#define MAX_RETRIES   5

// -----------------------------------------------------------------------------
// SPI Master handle and host
// -----------------------------------------------------------------------------
static spi_device_handle_t spi_master_handle = NULL;
#define SPI_HOST_TYPE HSPI_HOST

static const char *TAG = "MASTER";

// -----------------------------------------------------------------------------
// Initialize the SPI master interface.
// -----------------------------------------------------------------------------
esp_err_t spi_master_init(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,  // 13
        .miso_io_num = PIN_NUM_MISO,  // 12
        .sclk_io_num = PIN_NUM_CLK,   // 14
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MSG_BUF_SIZE,  // 128 bytes
    };

    ret = spi_bus_initialize(SPI_HOST_TYPE, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5000000,        // 5 MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,         // 15
        .queue_size = 3,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .duty_cycle_pos = 128,
        .cs_ena_posttrans = 3,
    };

    ret = spi_bus_add_device(SPI_HOST_TYPE, &devcfg, &spi_master_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
    }
    return ret;
}

// -----------------------------------------------------------------------------
// Send a command and wait for the slave to echo back the same command.
// -----------------------------------------------------------------------------
int SPI_send_confirm_command(uint8_t cmd, const uint8_t *data, size_t len)
{
    // Allocate DMA-capable buffers.
    uint8_t *tx_buffer = heap_caps_malloc(MSG_BUF_SIZE, MALLOC_CAP_DMA);
    uint8_t *rx_buffer = heap_caps_malloc(MSG_BUF_SIZE, MALLOC_CAP_DMA);
    if (!tx_buffer || !rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffers");
        if (tx_buffer) free(tx_buffer);
        if (rx_buffer) free(rx_buffer);
        return 0;
    }

    // Initialize buffers.
    memset(tx_buffer, 0, MSG_BUF_SIZE);
    memset(rx_buffer, 0, MSG_BUF_SIZE);
    tx_buffer[0] = cmd;
    if (data && len > 0 && (len < MSG_BUF_SIZE)) {
        memcpy(&tx_buffer[1], data, len);
    }

    int retry = 0;
    esp_err_t ret;
    while (retry < MAX_RETRIES) {
        // First transaction: send the command so the slave can update its state.
        spi_transaction_t trans1;
        memset(&trans1, 0, sizeof(trans1));
        trans1.length = MSG_BUF_SIZE * 8;  // transfer length in bits
        trans1.tx_buffer = tx_buffer;
        trans1.rx_buffer = rx_buffer;

        ret = spi_device_transmit(spi_master_handle, &trans1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transmit error on attempt %d (first transaction): %s",
                     retry, esp_err_to_name(ret));
        }

        // Wait briefly to allow the slave to process the command.
        vTaskDelay(pdMS_TO_TICKS(10));

        // Second transaction: perform a confirmation transfer.
        spi_transaction_t trans2;
        memset(&trans2, 0, sizeof(trans2));
        trans2.length = MSG_BUF_SIZE * 8;  // in bits
        trans2.tx_buffer = tx_buffer;
        trans2.rx_buffer = rx_buffer;

        // Clear the RX buffer before the second transaction.
        memset(rx_buffer, 0, MSG_BUF_SIZE);

        ret = spi_device_transmit(spi_master_handle, &trans2);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transmit error on attempt %d (second transaction): %s",
                     retry, esp_err_to_name(ret));
        } else {
            // Check if the slave echoed the expected header.
            if (rx_buffer[0] == cmd) {
                ESP_LOGI(TAG, "SPI confirm: Received expected header 0x%02X", rx_buffer[0]);
                free(tx_buffer);
                free(rx_buffer);
                return 1;
            } else {
                ESP_LOGW(TAG, "SPI confirm invalid on attempt %d: Received 0x%02X, expected 0x%02X",
                         retry, rx_buffer[0], cmd);
            }
        }
        retry++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    free(tx_buffer);
    free(rx_buffer);
    return 0;
}

// -----------------------------------------------------------------------------
// Send a command without expecting any confirmation.
// -----------------------------------------------------------------------------
int SPI_send_command_no_confirm(uint8_t cmd, const uint8_t *data, size_t len)
{
    uint8_t *tx_buffer = heap_caps_malloc(MSG_BUF_SIZE, MALLOC_CAP_DMA);
    if (!tx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate TX buffer");
        return 0;
    }
    memset(tx_buffer, 0, MSG_BUF_SIZE);
    tx_buffer[0] = cmd;
    if (data && len > 0 && (len < MSG_BUF_SIZE)) {
        memcpy(&tx_buffer[1], data, len);
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = MSG_BUF_SIZE * 8;
    t.tx_buffer = tx_buffer;  // Use the allocated buffer directly
    t.rx_buffer = NULL;       // No RX buffer needed

    esp_err_t ret = spi_device_transmit(spi_master_handle, &t);
    free(tx_buffer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit (no confirm) error: %s", esp_err_to_name(ret));
        return 0;
    }
    return 1;
}


static int recv_all(int sock, void *buffer, size_t len) {
    size_t received = 0;
    uint8_t *buf = (uint8_t *)buffer;
    while (received < len) {
        int r = recv(sock, buf + received, len - received, 0);
        if (r <= 0) {
            return -1;
        }
        received += r;
    }
    return 0;
}

    
static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    struct sockaddr_storage dest_addr;
    int keepAlive = 1;
    int keepIdle = 5;
    int keepInterval = 5;
    int keepCount = 3;

    // Setup the server address structure for IPv4
    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);

    // Create socket (TCP)
    int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    ESP_LOGI(TAG, "Socket created");

    // Bind the socket to the port
    if (bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    // Listen for incoming connections
    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "Error during listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        ESP_LOGI(TAG, "Waiting for a connection...");

        // Accept incoming connection (blocking call)
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        
        // Set TCP keepalive options
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Connection accepted from %s", addr_str);

        // Initialize FSM state and checksum accumulator
        fsm_state_t state = FSM_WAIT_START;
        unsigned int computed_checksum = 0;
        unsigned int expected_total_data = 0;
        unsigned int received_data = 0;

        while (state != FSM_COMPLETE && state != FSM_ERROR) {
            uint8_t pkt_type;
            // Read the packet type (1 byte)
            if (recv_all(sock, &pkt_type, 1) != 0) {
                ESP_LOGE(TAG, "Failed to read packet type");
                state = FSM_ERROR;
                break;
            }
            // Process each packet type.
            switch(pkt_type) {
                case PACKET_TYPE_START:
                {
                    if (state != FSM_WAIT_START) {
                        ESP_LOGE(TAG, "Unexpected START packet");
                        state = FSM_ERROR;
                        break;
                    }
                    // START packet: next 8 bytes: width (2), height (2), total data length (4)
                    uint8_t buffer[8];
                    if (recv_all(sock, buffer, sizeof(buffer)) != 0) {
                        ESP_LOGE(TAG, "Failed to read START packet payload");
                        state = FSM_ERROR;
                        break;
                    }
                    uint16_t width, height;
                    unsigned int total_len;
                    width = (buffer[0] << 8) | buffer[1];
                    height = (buffer[2] << 8) | buffer[3];
                    total_len = (buffer[4] << 24) | (buffer[5] << 16) | (buffer[6] << 8) | buffer[7];
                    expected_total_data = total_len;
                    ESP_LOGI(TAG, "START: Image dimensions: %dx%d, total data: %u bytes", width, height, total_len);

                    // Send SPI command for START via confirmation function.
                    if (!SPI_send_confirm_command(PACKET_START_WRITE, NULL, 0)) {
                        ESP_LOGE(TAG, "SPI START command failed");
                        state = FSM_ERROR;
                        break;
                    }

                    // Send ACK for START (send one byte same as packet type)
                    if (send(sock, &pkt_type, 1, 0) < 0) {
                        ESP_LOGE(TAG, "Failed to send START ACK");
                        state = FSM_ERROR;
                        break;
                    }
                    state = FSM_WAIT_DATA;
                    break;
                }
                case PACKET_TYPE_DATA:
                {
                    // DATA packet: next 2 bytes: data length, then payload
                    uint8_t len_buf[2];
                    if (recv_all(sock, len_buf, sizeof(len_buf)) != 0) {
                        ESP_LOGE(TAG, "Failed to read DATA length");
                        state = FSM_ERROR;
                        break;
                    }
                    uint16_t data_len = (len_buf[0] << 8) | len_buf[1];
                    if (data_len > 0) {
                        uint8_t *data_chunk = malloc(data_len);
                        if (!data_chunk) {
                            ESP_LOGE(TAG, "Memory allocation failed");
                            state = FSM_ERROR;
                            break;
                        }
                        if (recv_all(sock, data_chunk, data_len) != 0) {
                            ESP_LOGE(TAG, "Failed to read DATA payload");
                            free(data_chunk);
                            state = FSM_ERROR;
                            break;
                        }
                        // Update computed checksum
                        for (int i = 0; i < data_len; i++) {
                            computed_checksum += data_chunk[i];
                        }
                        // Forward the received DATA packet to the SPI slave (no confirmation).
                        if (!SPI_send_command_no_confirm(PACKET_TYPE_DATA, data_chunk, data_len)) {
                            ESP_LOGE(TAG, "SPI DATA transmission failed");
                            free(data_chunk);
                            state = FSM_ERROR;
                            break;
                        }
                        received_data += data_len;
                        free(data_chunk);
                    }
                    break;
                }
                case PACKET_TYPE_MID:
                {
                    // MID packet: next 2 reserved bytes
                    uint8_t mid_buf[2];
                    if (recv_all(sock, mid_buf, sizeof(mid_buf)) != 0) {
                        ESP_LOGE(TAG, "Failed to read MID packet payload");
                        state = FSM_ERROR;
                        break;
                    }
                    ESP_LOGI(TAG, "MID packet received (halfway marker). Total received so far: %u bytes", received_data);
                    
                    // Send TCP ACK for MID.
                    if (send(sock, &pkt_type, 1, 0) < 0) {
                        ESP_LOGE(TAG, "Failed to send MID ACK");
                        state = FSM_ERROR;
                        break;
                    }
                    // Send SPI command for MID confirmation.
                    if (!SPI_send_confirm_command(PACKET_MID, NULL, 0)) {
                        ESP_LOGE(TAG, "SPI MID command failed");
                        state = FSM_ERROR;
                        break;
                    }
                    state = FSM_WAIT_DATA; // Continue receiving data
                    break;
                }
                case PACKET_TYPE_END:
                {
                    // END packet: next 4 bytes: checksum
                    uint8_t end_buf[4];
                    if (recv_all(sock, end_buf, sizeof(end_buf)) != 0) {
                        ESP_LOGE(TAG, "Failed to read END packet payload");
                        state = FSM_ERROR;
                        break;
                    }
                    unsigned int received_checksum = (end_buf[0] << 24) | (end_buf[1] << 16) | (end_buf[2] << 8) | end_buf[3];
                    ESP_LOGI(TAG, "END packet received. Received checksum: 0x%08X, Computed checksum: 0x%08X", received_checksum, computed_checksum);
                    if (received_checksum != computed_checksum) {
                        ESP_LOGE(TAG, "Checksum mismatch!");
                        state = FSM_ERROR;
                    } else {
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                        // Send SPI command for END confirmation.
                        if (!SPI_send_confirm_command(PACKET_END, NULL, 0)) {
                            ESP_LOGE(TAG, "SPI END command failed");
                            state = FSM_ERROR;
                            break;
                        }
                        // Send TCP ACK for END.
                        pkt_type = PACKET_TYPE_END;
                        if (send(sock, &pkt_type, 1, 0) < 0) {
                            ESP_LOGE(TAG, "Failed to send END ACK");
                            state = FSM_ERROR;
                            break;
                        }
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                        state = FSM_COMPLETE;
                    }
                    break;
                }
                default:
                    ESP_LOGE(TAG, "Unknown packet type received: 0x%02X", pkt_type);
                    state = FSM_ERROR;
                    break;
            } // end switch
        } // end while FSM

        if (state == FSM_COMPLETE) {
            ESP_LOGI(TAG, "Image transfer complete. Total data received: %u bytes", received_data);
        } else {
            ESP_LOGE(TAG, "Image transfer error occurred.");
        }

        shutdown(sock, 0);
        close(sock);
    } // end while listening

    close(listen_sock);
    vTaskDelete(NULL);
}



static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(ETH_TAG, "Ethernet Link Up");
        ESP_LOGI(ETH_TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        
        // esp_netif_t *eth_netif = esp_netif_get_handle_from_ifkey("ETH_DEF");
        // if (eth_netif) {
        //     ESP_LOGI(ETH_TAG, "Starting DHCP client...");
        //     esp_netif_dhcpc_start(eth_netif);
        // } else {
        //     ESP_LOGE(ETH_TAG, "Failed to get ETH_DEF interface handle");
        // }
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(ETH_TAG, "Ethernet Link Down");
        xEventGroupSetBits(s_eth_event_group, ETHERNET_FAIL_BIT);
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(ETH_TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(ETH_TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGE(ETH_TAG, "Ethernet Got IP Address");
    ESP_LOGE(ETH_TAG, "~~~~~~~~~~~");
    ESP_LOGE(ETH_TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGE(ETH_TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGE(ETH_TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGE(ETH_TAG, "~~~~~~~~~~~");

    xEventGroupSetBits(s_eth_event_group, ETHERNET_CONNECTED_BIT);
}


static esp_eth_handle_t eth_init_internal(esp_eth_mac_t **mac_out, esp_eth_phy_t **phy_out)
{
    esp_eth_handle_t ret = NULL;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_PHY_RST_GPIO;

    eth_esp32_emac_config_t esp32_emac_config = {
        .smi_mdc_gpio_num  = ETH_MDC_GPIO,                       
        .smi_mdio_gpio_num = ETH_MDIO_GPIO,                      
        .interface = EMAC_DATA_INTERFACE_RMII,        
        .clock_config = { .rmii = { .clock_mode = EMAC_CLK_OUT, .clock_gpio = EMAC_CLK_OUT_180_GPIO } },
        .dma_burst_len = ETH_DMA_BURST_LEN_32 
    };

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

    esp_eth_handle_t eth_handle = NULL;
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);

    ESP_GOTO_ON_FALSE(esp_eth_driver_install(&config, &eth_handle) == ESP_OK, NULL, err, ETH_TAG, "Ethernet driver install failed");

    if (mac_out != NULL) {
        *mac_out = mac;
    }
    if (phy_out != NULL) {
        *phy_out = phy;
    }
    return eth_handle;

err:
    if (eth_handle != NULL) {
        esp_eth_driver_uninstall(eth_handle);
    }
    if (mac != NULL) {
        mac->del(mac);
    }
    if (phy != NULL) {
        phy->del(phy);
    }
    return ret;
}


void ethernet_setup(void)
{
    s_eth_event_group = xEventGroupCreate();

    esp_eth_handle_t eth_handle = eth_init_internal(NULL, NULL);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    // Attach the Ethernet netif glue before setting IP info.
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

#if STATIC_IP
    // Disable the DHCP client now that the glue is attached.
    if (esp_netif_dhcpc_stop(eth_netif) != ESP_OK) {
        ESP_LOGE(ETH_TAG, "Failed to stop DHCP client");
        return;
    }

    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));

    // Use IP4_ADDR() macro to set your static IP information.
    IP4_ADDR(&ip_info.ip, 192, 168, 0, 50);
    IP4_ADDR(&ip_info.gw, 192, 168, 0, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    if (esp_netif_set_ip_info(eth_netif, &ip_info) != ESP_OK) {
        ESP_LOGE(ETH_TAG, "Failed to set IP info");
    } else { 
        ESP_LOGI(ETH_TAG, "Succeeded to set static IP info");
    }
#endif /* STATIC_IP */

    ESP_LOGI(ETH_TAG, "Registering ESP_EVENT_ANY_ID handlers...");
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_LOGI(ETH_TAG, "Registering IP_EVENT_ETH_GOT_IP handlers...");
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    ESP_LOGI(ETH_TAG, "Registering ESP_NETIF_IP_EVENT_GOT_IP handlers...");
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_NETIF_IP_EVENT_GOT_IP, &got_ip_event_handler, NULL));
    // ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_NETIF_IP_EVENT_LOST_IP, ip_lost_event_handler, NULL));

    // ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_NETIF_IP_EVENT_LOST_IP, &lost_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    ESP_LOGI(ETH_TAG, "ESP ETH START Done.");

    EventBits_t bits = xEventGroupWaitBits(s_eth_event_group,
                                           ETHERNET_CONNECTED_BIT | ETHERNET_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & ETHERNET_CONNECTED_BIT) {
        ESP_LOGI(ETH_TAG, "Ethernet Connection established.");
    } else if (bits & ETHERNET_FAIL_BIT) {
        ESP_LOGE(ETH_TAG, "Ethernet Connection Failed.");
    } else {
        ESP_LOGE(ETH_TAG, "UNEXPECTED EVENT");
    }
}


void app_main(void)
{
    ethernet_setup();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_LOGI(ETH_TAG, "Ethernet Initialized!");

    ESP_LOGI(TAG, "Master starting...");
    if (spi_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "SPI master initialization failed");
        return;
    }
    ESP_LOGI(ETH_TAG, "Started SPI Master");

    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    ESP_LOGI(ETH_TAG, "Started TCP server task");

}
