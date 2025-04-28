#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

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
#include <string.h>
#include <sys/errno.h>
#include <sys/socket.h>
#include <netdb.h>
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

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

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ETH_PHY_ADDR             1
#define ETH_PHY_RST_GPIO        -1          // not connected
#define ETH_MDC_GPIO            23
#define ETH_MDIO_GPIO           18
#define ETH_TAG                 "ETH"

#define EMAC_TX_CLK             0 
#define EMAC_RX_CLK             5

static EventGroupHandle_t s_eth_event_group;

#define ETHERNET_CONNECTED_BIT  BIT0
#define ETHERNET_FAIL_BIT       BIT1
#define PORT 8080

#define STATIC_IP               1

#if STATIC_IP
    #define S_IP        "192.168.0.50"     
    #define GATEWAY     "192.168.0.1"    
    #define NETMASK     "255.255.255.0"
#endif /* STATIC_IP */

static netif_input_fn default_input = NULL;

static const char *COMM_TAG = "TCP_SOCKET";

// Make sure your Ethernet setup functions are defined somewhere, for example:
extern void ethernet_setup(void);   // Your Ethernet initialization code
// And a global event group or similar mechanism used in ethernet_setup, if any.

// Packet type definitions
#define PACKET_TYPE_START_WRITE 0xA0
#define PACKET_TYPE_DATA_WRITE  0xA1
#define PACKET_TYPE_MID_WRITE   0xA2
#define PACKET_TYPE_END_WRITE   0xA3
#define PACKET_TYPE_FUC   0xA4

#define PACKET_TYPE_START_READ 0xB0
#define PACKET_TYPE_DATA_READ  0xB1
#define PACKET_TYPE_END_READ   0xB3

#define PACKET_TYPE_START_COMPUTE 0xC0   
#define PACKET_TYPE_POLL_COMPUTE  0xC1  
#define PACKET_TYPE_WAIT_COMPUTE  0xC2   

#define PACKET_TYPE_DELETE 0xE0
#define PACKET_TYPE_DELETE_RET 0xE1

#define PACKET_TYPE_END_COMPUTE   0xC3   


typedef enum {
    FSM_WAIT_START_WRITE,
    FSM_WAIT_DATA_WRITE,
    FSM_WAIT_MID_WRITE,
    FSM_WAIT_END_WRITE,
    FSM_WAIT_START_READ,
    FSM_WAIT_DATA_READ,
    FSM_WAIT_END_READ,
    FSM_WAIT_START_COMPUTE, 
    FSM_WAIT_COMPUTE,    
    FSM_COMPLETE,
    FSM_ERROR
} fsm_state_t;


// ################################################################

#define PIN_NUM_MOSI  13
#define PIN_NUM_MISO  12
#define PIN_NUM_CLK   14

#define PIN_NUM_CS_1   15
#define PIN_NUM_CS_0    33

#define GPIO_HANDSHAKE_1 36
#define GPIO_HANDSHAKE_0 39

// #define GPIO_HANDSHAKE_0 4

#define TRANSFER_SIZE 2048

#define CMD_VALID   0xA6  
#define CMD_INVALID 0xB1  

// Maximum retry attempts.
#define MAX_RETRY 5

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
        ESP_LOGE(COMM_TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 5000000,  // 5 MHz
        .duty_cycle_pos = 128,      // 50% duty cycle
        .mode = 0,
        .spics_io_num = PIN_NUM_CS_0,
        .cs_ena_posttrans = 3,      // Keep CS low a few extra cycles after transfer
        .queue_size = 3
    };

    ret = spi_bus_add_device(SPI_HOST_TYPE, &devcfg, &spi_master_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(COMM_TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
    }
    return ret;
}

static void handshake_init_master(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_HANDSHAKE_0),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
}

static void wait_for_slave_ready(void)
{
    while(gpio_get_level(GPIO_HANDSHAKE_0) == 0) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static int spi_send_packet_clear(spi_transaction_t t, uint8_t *tx_buf, uint8_t *rx_buf)
{
    memset(tx_buf, 0, TRANSFER_SIZE);

    esp_err_t ret;

    wait_for_slave_ready();

    // memset(&t, 0, sizeof(t));
    memset(tx_buf, 0, sizeof(t)); 
    // t.length = TRANSFER_SIZE * 8;  // in bits.
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    ESP_LOGI(COMM_TAG, "Sending data: 0x%02X 0x%02X 0x%02X 0x%02X ...", tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
    ret = spi_device_polling_transmit(spi_master_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(COMM_TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        return -1; 
    }
    ESP_LOGI(COMM_TAG, "Received data: 0x%02X 0x%02X 0x%02X 0x%02X ...", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

    return 0;
}

static int confirm_packet(uint8_t *rx_buf, uint8_t *expected_rx_buf) { 
    if (rx_buf[0] == PACKET_TYPE_FUC) return 1; 

    for (int i = 0; i < TRANSFER_SIZE; i++)
    {
        if ((i != 1) && (rx_buf[i] != expected_rx_buf[i])) { 
            if (i == 0) { 
                ESP_LOGE(COMM_TAG, "    MISMATCH: SPI COMMAND: (expected 0x%02X, got 0x%02X)\n", expected_rx_buf[0], rx_buf[0]);
            } else { 
                ESP_LOGE(COMM_TAG, "    MISMATCH: DATA WRONG: [%d] (expected 0x%02X, got 0x%02X)", i, expected_rx_buf[i], rx_buf[i]);
            }
            return -1; 
        }
    }

    ESP_LOGI(COMM_TAG, "    MATCH: SPI COMMAND: expected 0x%02X, got 0x%02X\n",expected_rx_buf[0], rx_buf[0]);
    return 0; 
}

static int spi_send_packet(spi_transaction_t t, uint8_t *tx_buf, uint8_t *rx_buf, uint8_t *expected_rx_buf)
{
    esp_err_t ret;
    int rett; 

    wait_for_slave_ready();

    // memset(&t, 0, sizeof(t));
    // t.length = TRANSFER_SIZE * 8;  // in bits.
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    ESP_LOGI(COMM_TAG, "Sending buffer: 0x%02X, 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X", tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[TRANSFER_SIZE - 2], tx_buf[TRANSFER_SIZE - 1]);

    ret = spi_device_polling_transmit(spi_master_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(COMM_TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        return -1; 
    }
    ESP_LOGI(COMM_TAG, "Received buffer: 0x%02X, 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[TRANSFER_SIZE - 2], rx_buf[TRANSFER_SIZE - 1]);

    if (expected_rx_buf != NULL) { 
        if ((expected_rx_buf[0] != 0) && confirm_packet(rx_buf, expected_rx_buf) != 0) {
            if (rx_buf[0] == PACKET_TYPE_FUC)
                return -1; 
                
            tx_buf[1] = 0;

            int attempt = 0; 
            while (attempt < 3) { 
                // #########################################

                wait_for_slave_ready();

                // memset(&t, 0, sizeof(t));
                // t.length = TRANSFER_SIZE * 8;  // in bits.
                expected_rx_buf[1] = 0; 
                t.tx_buffer = expected_rx_buf;
                t.rx_buffer = rx_buf;

                ESP_LOGW(COMM_TAG, "         %d,0 -> Sending data: 0x%02X 0x%02X 0x%02X 0x%02X ...", attempt, expected_rx_buf[0], expected_rx_buf[1], expected_rx_buf[2], expected_rx_buf[3]);
                ret = spi_device_polling_transmit(spi_master_handle, &t);
                if (ret != ESP_OK) {
                    ESP_LOGE(COMM_TAG, "             %d,0 -> SPI transaction failed: %s", attempt, esp_err_to_name(ret));
                    return -1; 
                }
                ESP_LOGW(COMM_TAG, "         %d,0 -> Received data: 0x%02X 0x%02X 0x%02X 0x%02X ...", attempt, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);


                // #########################################

                wait_for_slave_ready();

                // memset(&t, 0, sizeof(t));
                // t.length = TRANSFER_SIZE * 8;  // in bits.
                tx_buf[1] = 0;
                t.tx_buffer = tx_buf;
                t.rx_buffer = rx_buf;

                ESP_LOGW(COMM_TAG, "         %d,1 -> Sending data: 0x%02X 0x%02X 0x%02X 0x%02X ...", attempt, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
                ret = spi_device_polling_transmit(spi_master_handle, &t);
                if (ret != ESP_OK) {
                    ESP_LOGE(COMM_TAG, "             %d,1 -> SPI transaction failed: %s", attempt, esp_err_to_name(ret));
                    return -1; 
                }
                ESP_LOGW(COMM_TAG, "         %d,1 -> Received data: 0x%02X 0x%02X 0x%02X 0x%02X ...", attempt, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

                rett = confirm_packet(rx_buf, expected_rx_buf); 
                if (rett < 0)
                {
                    attempt++;
                    continue;
                } else if (rett > 0) { 
                    return 1; 
                }

                // #########################################

                wait_for_slave_ready();

                tx_buf[1] = 1;
                t.tx_buffer = tx_buf;
                t.rx_buffer = rx_buf;

                ESP_LOGW(COMM_TAG, "         %d,2 -> Sending SAME data: 0x%02X 0x%02X 0x%02X 0x%02X ...", attempt, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
                ret = spi_device_polling_transmit(spi_master_handle, &t);
                if (ret != ESP_OK) {
                    ESP_LOGE(COMM_TAG, "         %d,2 -> SPI transaction failed: %s", attempt, esp_err_to_name(ret));
                    return -1; 
                }
                ESP_LOGW(COMM_TAG, "         %d,2 -> Received data: 0x%02X 0x%02X 0x%02X 0x%02X ...", attempt, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

                rett = confirm_packet(rx_buf, expected_rx_buf); 
                if (rett < 0)
                {
                    attempt++;
                    continue;
                } else if (rett > 0) { 
                    return 1; 
                } else {
                    break;
                }
            }

            return -1; 
        } else { 
            tx_buf[1] = 1;
            return 0; 
        }
    }
    else
    {
        tx_buf[1] = 1;
        return 0; 
    }
}

static int spi_receive_packet(spi_transaction_t t, uint8_t *tx_buf, uint8_t *rx_buf)
{
    ESP_LOGI(COMM_TAG, "    Receiving Packet");
    return spi_send_packet(t, tx_buf, rx_buf, NULL);
}


// ################################################################


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

int fsm_file_read_master(int sock, spi_transaction_t t,
                         uint8_t *dummy_tx_buf, uint8_t *rx_buf)
{
    int ret, data_len;
    unsigned int computed_checksum = 0;
    unsigned int received_data = 0;
    memset(dummy_tx_buf, 0, TRANSFER_SIZE);
    dummy_tx_buf[0] = PACKET_TYPE_DATA_READ; 
    
    while (1) {

        // Perform one SPI transaction to receive a packet from the slave.
        ret = spi_receive_packet(t, dummy_tx_buf, rx_buf);
        if (ret != 0) {
            ESP_LOGE(COMM_TAG, "SPI read transaction failed.");
            return -1;
        }
        
        // Parse the incoming SPI packet based on packet type.
        uint8_t pkt_type = rx_buf[0];

        if (pkt_type == PACKET_TYPE_DATA_READ) {
            // DATA packet structure: 
            // [Packet Type (1 byte)] [Data length (2 bytes)] [Payload (data_chunk)]
            uint16_t len = (rx_buf[1] << 8) | rx_buf[2];
            data_len = len;
            ESP_LOGI(COMM_TAG, "DATA_READ packet: data length = %d", data_len);
            
            // Forward the DATA_READ packet to the router:
            // First, send the 1-byte packet type.
            if (send(sock, rx_buf, 1, 0) < 0) {
                ESP_LOGE(COMM_TAG, "Failed to forward packet type to router");
                return -1;
            }
            // Then, send the 2-byte length header.
            if (send(sock, rx_buf + 1, 2, 0) < 0) {
                ESP_LOGE(COMM_TAG, "Failed to forward data length to router");
                return -1;
            }
            // Finally, send the payload (data_len bytes).
            if (send(sock, rx_buf + 3, data_len, 0) < 0) {
                ESP_LOGE(COMM_TAG, "Failed to forward data payload to router");
                return -1;
            }
            
            // Accumulate checksum from payload (optional, if desired).
            for (int i = 0; i < data_len; i++) {
                computed_checksum += rx_buf[3 + i];
            }
            received_data += data_len;

            dummy_tx_buf[0] = PACKET_TYPE_DATA_READ; 

            ESP_LOGI(COMM_TAG, "Forwarded DATA_READ packet, %d bytes", data_len);
        }
        else if (pkt_type == PACKET_TYPE_END_READ) {
            // END packet structure:
            // [Packet Type (1 byte)] [Checksum (4 bytes)]
            unsigned int slave_checksum = (rx_buf[1] << 24) | (rx_buf[2] << 16) |
                                      (rx_buf[3] << 8) | rx_buf[4];
            ESP_LOGI(COMM_TAG, "END_READ packet: Slave checksum = 0x%08X, Computed checksum = 0x%08X",
                     slave_checksum, computed_checksum);
                    
            // Forward the END_READ packet to the router.
            if (send(sock, rx_buf, 5, 0) < 0) {
                ESP_LOGE(COMM_TAG, "Failed to send END_READ packet to router");
                return -1;
            }

            if (slave_checksum == 0xFFFFFFFF) return -1; 

            dummy_tx_buf[0] = PACKET_TYPE_END_READ; 
            break;  
        }
        else if (pkt_type == PACKET_TYPE_START_READ) { 
            continue; 
        }
        else {
            ESP_LOGE(COMM_TAG, "Unexpected SPI packet type received in read FSM: 0x%02X", pkt_type);
            return -1;
        }
    }

    ESP_LOGI(COMM_TAG, "Read FSM complete. Total data forwarded: %u bytes", received_data);
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
        ESP_LOGE(COMM_TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    ESP_LOGI(COMM_TAG, "Socket created");

    // Bind the socket to the port
    if (bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE(COMM_TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(COMM_TAG, "Socket bound, port %d", PORT);

    // Listen for incoming connections
    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(COMM_TAG, "Error during listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }


    while (1) {
        ESP_LOGI(COMM_TAG, "Socket listening");

        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        ESP_LOGI(COMM_TAG, "Waiting for a connection...");

        // Accept incoming connection (blocking call)
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(COMM_TAG, "Unable to accept connection: errno %d", errno);
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
        ESP_LOGI(COMM_TAG, "Connection accepted from %s", addr_str);

        // Initialize FSM state and checksum accumulator
        fsm_state_t state = FSM_WAIT_START_WRITE;
        unsigned int computed_checksum = 0;
        unsigned int expected_total_data = 0;
        unsigned int received_data = 0;
        uint8_t previous_spi_packet = 0; // For the very first transaction, no previous packet is expected.
        uint16_t previous_spi_paacket_len = 0; 
        uint8_t *previous_data_chunk = NULL; 
        uint16_t data_len = 0;

        uint8_t *tx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
        uint8_t *rx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
        uint8_t *expected_rx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
    
        spi_transaction_t t = {0};
        t.length = TRANSFER_SIZE * 8;

        if (!tx_buf || !rx_buf || !expected_rx_buf) {
            ESP_LOGE(COMM_TAG, "Heap allocation failed for SPI buffers");
            if (tx_buf) free(tx_buf);
            if (rx_buf) free(rx_buf);
            if (expected_rx_buf) free(expected_rx_buf);
            state = FSM_ERROR;
        }
        memset(tx_buf, 0, TRANSFER_SIZE);
        memset(rx_buf, 0, TRANSFER_SIZE);
        memset(expected_rx_buf, 0, TRANSFER_SIZE);

        while (state != FSM_COMPLETE && state != FSM_ERROR) {
            uint8_t pkt_type;
            if (recv_all(sock, &pkt_type, 1) != 0) {
                ESP_LOGE(COMM_TAG, "Failed to read packet type");
                state = FSM_ERROR;
                break;
            }
           
            switch(pkt_type) {
                // ---------- Write FSM handling  ------------
                case PACKET_TYPE_START_WRITE:
                {
                    if (state != FSM_WAIT_START_WRITE) {
                        ESP_LOGE(COMM_TAG, "Unexpected START packet");
                        state = FSM_ERROR;
                        break;
                    }
                    
                    uint8_t filename_payload[80];
                    if (recv_all(sock, filename_payload, sizeof(filename_payload)) != 0) {
                        ESP_LOGE(COMM_TAG, "Failed to read START packet payload (filename)");
                        state = FSM_ERROR;
                        break;
                    }
                    
                    char filename[80];
                    memcpy(filename, filename_payload, 79);
                    filename[79] = '\0';
                    ESP_LOGI(COMM_TAG, "START: Filename: %s", filename);

                    tx_buf[0] = PACKET_TYPE_START_WRITE;
                    tx_buf[1] = 0x00;
                    memcpy(tx_buf + 2, filename_payload, 80);

                    if (spi_send_packet(t, tx_buf, rx_buf, NULL) != 0) {
                        ESP_LOGE(COMM_TAG, "SPI START packet verification failed.");
                        state = FSM_ERROR;
                        break;
                    }
                    expected_rx_buf[0] = PACKET_TYPE_START_WRITE;
                    memcpy(expected_rx_buf + 2, tx_buf + 2, TRANSFER_SIZE - 2);

                    if (send(sock, &pkt_type, 1, 0) < 0) {
                        ESP_LOGE(COMM_TAG, "Failed to send START ACK");
                        state = FSM_ERROR;
                        break;
                    }
                    state = FSM_WAIT_DATA_WRITE;
                    break;
                }
                case PACKET_TYPE_DATA_WRITE:
                {
                    uint8_t len_buf[2];
                    if (recv_all(sock, len_buf, sizeof(len_buf)) != 0) {
                        ESP_LOGE(COMM_TAG, "Failed to read DATA length");
                        state = FSM_ERROR;
                        break;
                    }
                    data_len = (len_buf[0] << 8) | len_buf[1];

                    if (data_len == 0) break;

                    tx_buf[0] = PACKET_TYPE_DATA_WRITE;
                    tx_buf[1] = 0x00;  
                    tx_buf[2] = (data_len >> 8) & 0xFF;
                    tx_buf[3] = data_len & 0xFF;

                    if (recv_all(sock, tx_buf + 4, data_len) != 0)
                    {
                        ESP_LOGE(COMM_TAG, "Failed to read DATA payload");
                        state = FSM_ERROR;
                        break;
                    }
                    ESP_LOGI(COMM_TAG, "IN PACKET_TYPE_DATA_WRITE, data_len: %d", data_len);

                    if (spi_send_packet(t, tx_buf, rx_buf, expected_rx_buf) != 0) {
                        ESP_LOGE(COMM_TAG, "SPI DATA packet verification failed.");
                        state = FSM_ERROR;
                        break;
                    }
                    expected_rx_buf[0] = PACKET_TYPE_DATA_WRITE;
                    expected_rx_buf[1] = tx_buf[1];
                    memcpy(expected_rx_buf + 2, tx_buf + 2, TRANSFER_SIZE - 2);

                    for (int i = 0; i < data_len; i++) {
                        computed_checksum += tx_buf[4 + i];
                    }
                    received_data += data_len;

                    break;
                }
                case PACKET_TYPE_MID_WRITE:
                {
                    uint8_t mid_buf[2];
                    if (recv_all(sock, mid_buf, sizeof(mid_buf)) != 0) {
                        ESP_LOGE(COMM_TAG, "Failed to read MID packet payload");
                        state = FSM_ERROR;
                        break;
                    }
                    ESP_LOGI(COMM_TAG, "!!!!~MID packet received~!!!!");

                    tx_buf[0] = PACKET_TYPE_MID_WRITE; 
                    // memset(tx_buf + 2, 0, TRANSFER_SIZE - 2);
                    if (spi_send_packet(t, tx_buf, rx_buf, expected_rx_buf) != 0)
                    {
                        ESP_LOGE(COMM_TAG, "SPI DATA packet verification failed.");
                        state = FSM_ERROR;
                        break;
                    }
                    expected_rx_buf[0] = PACKET_TYPE_MID_WRITE;
                    expected_rx_buf[1] = tx_buf[1];
                    // memset(expected_rx_buf + 2, 0, TRANSFER_SIZE - 2);

                    if (send(sock, &pkt_type, 1, 0) < 0) {
                        ESP_LOGE(COMM_TAG, "Failed to send MID ACK");
                        state = FSM_ERROR;
                        break;
                    }
                    state = FSM_WAIT_DATA_WRITE; 

                    break;
                }
                case PACKET_TYPE_END_WRITE:
                {
                    uint8_t end_buf[4];
                    if (recv_all(sock, end_buf, sizeof(end_buf)) != 0) {
                        ESP_LOGE(COMM_TAG, "Failed to read END packet payload");
                        state = FSM_ERROR;
                        break;
                    }

                    unsigned int received_checksum = (end_buf[0] << 24) | (end_buf[1] << 16) | (end_buf[2] << 8) | end_buf[3];
                    ESP_LOGI(COMM_TAG, "END packet received. Received checksum: 0x%08X, Computed checksum: 0x%08X", received_checksum, computed_checksum);

                    if (received_checksum != computed_checksum) {
                        ESP_LOGE(COMM_TAG, "Checksum mismatch!");
                        state = FSM_ERROR;
                    } else {
                        tx_buf[0] = PACKET_TYPE_END_WRITE;
                        tx_buf[1] = 0x00; 
                        tx_buf[2] = end_buf[0]; 
                        tx_buf[3] = end_buf[1]; 
                        tx_buf[4] = end_buf[2]; 
                        tx_buf[5] = end_buf[3]; 

                        if (spi_send_packet(t, tx_buf, rx_buf, expected_rx_buf) != 0)
                        {
                            ESP_LOGE(COMM_TAG, "SPI DATA packet verification failed.");
                            state = FSM_ERROR;
                            break;
                        }

                        memcpy(expected_rx_buf, tx_buf, TRANSFER_SIZE);

                        state = FSM_COMPLETE;
                    }
                    break;
                }
                
                // ---------- Read FSM handling ------------
                case PACKET_TYPE_START_READ:
                {
                    if (state != FSM_WAIT_START_WRITE) { // FSM_WAIT_START_WRITE is being used as an IDLE State here. Not the best logic lol.
                        ESP_LOGE(COMM_TAG, "Unexpected START_READ packet");
                        state = FSM_ERROR;
                        break;
                    }
                    uint8_t filename_payload[80];
                    if (recv_all(sock, filename_payload, sizeof(filename_payload)) != 0) {
                        ESP_LOGE(COMM_TAG, "Failed to read START_READ filename payload");
                        state = FSM_ERROR;
                        break;
                    }
                    char filename[80];
                    memcpy(filename, filename_payload, 79);
                    filename[79] = '\0';
                    ESP_LOGI(COMM_TAG, "START_READ: Filename: %s", filename);
                    
                    // Forward the read command to the slave via SPI.
                    tx_buf[0] = PACKET_TYPE_START_READ;
                    tx_buf[1] = 0x00;
                    memcpy(tx_buf + 2, filename_payload, 80);
                    if (spi_send_packet(t, tx_buf, rx_buf, expected_rx_buf) != 0) {
                        ESP_LOGE(COMM_TAG, "SPI START_READ command failed.");
                        state = FSM_ERROR;
                        break;
                    }
                    
                    // Send an ACK back to the router for the START_READ command.
                    if (send(sock, &pkt_type, 1, 0) < 0) {
                        ESP_LOGE(COMM_TAG, "Failed to send START_READ ACK");
                        state = FSM_ERROR;
                        break;
                    }
                    
                    // Transition to the state for reading data from the slave.
                    state = FSM_WAIT_DATA_READ;
                    
                    // Now call the new read FSM that reads data over SPI and sends it over Ethernet.
                    if (fsm_file_read_master(sock, t, tx_buf, rx_buf) != 0) {
                        ESP_LOGE(COMM_TAG, "FSM file read on Master failed.");
                        state = FSM_ERROR;
                    }
                    
                    // After the read FSM completes, mark the overall transaction as complete.
                    state = FSM_COMPLETE;
                    break;
                }
                
                // ---------- Compute FSM handling ------------
                case PACKET_TYPE_START_COMPUTE:
                {
                    if (state != FSM_WAIT_START_WRITE) {
                        ESP_LOGE(COMM_TAG, "Unexpected START_COMPUTE packet in state %d", state);
                        state = FSM_ERROR;
                        break;
                    }
                    ESP_LOGI(COMM_TAG, "Received START_COMPUTE from router.");

                    uint8_t filename_payload[80];
                    if (recv_all(sock, filename_payload, sizeof(filename_payload)) != 0) {
                        ESP_LOGE(COMM_TAG, "Failed to read START_READ filename payload");
                        state = FSM_ERROR;
                        break;
                    }
                    char filename[80];
                    memcpy(filename, filename_payload, 79);
                    filename[79] = '\0';
                    ESP_LOGI(COMM_TAG, "START_READ: Filename: %s", filename);
                    
                    // Forward the read command to the slave via SPI.
                    tx_buf[0] = PACKET_TYPE_START_COMPUTE;
                    tx_buf[1] = 0x00;
                    memcpy(tx_buf + 2, filename_payload, 80);

                    if (spi_send_packet(t, tx_buf, rx_buf, NULL) != 0) {
                        ESP_LOGE(COMM_TAG, "SPI START_COMPUTE transaction failed.");
                        state = FSM_ERROR;
                        break;
                    }

                    memset(tx_buf, 0, TRANSFER_SIZE);
                    if (spi_send_packet(t, tx_buf, rx_buf, NULL) != 0) {
                        ESP_LOGE(COMM_TAG, "SPI second transaction during START_COMPUTE failed.");
                        state = FSM_ERROR;
                        break;
                    }

                    uint8_t slave_response = rx_buf[0];
                    if (slave_response == PACKET_TYPE_FUC) {
                        ESP_LOGE(COMM_TAG, "Slave returned FUC during compute startup.");
                        if (send(sock, rx_buf, 1, 0) < 0) {
                            ESP_LOGE(COMM_TAG, "Failed to forward FUC to router.");
                        }
                        state = FSM_ERROR;
                        break;
                    } else if (slave_response == PACKET_TYPE_END_COMPUTE) {
                        // Check for a flag (e.g. 0xFF means no such file).
                        if (rx_buf[1] == 0xFF) {
                            ESP_LOGE(COMM_TAG, "Slave reported compute error: no such file (flag 0xFF).");
                            if (send(sock, rx_buf, 2, 0) < 0) {
                                ESP_LOGE(COMM_TAG, "Failed to forward END_COMPUTE error to router.");
                            }
                            state = FSM_ERROR;
                            break;
                        } else {
                            ESP_LOGE(COMM_TAG, "Unexpected END_COMPUTE received during compute startup.");
                            state = FSM_ERROR;
                            break;
                        }
                    } else if (slave_response == PACKET_TYPE_START_COMPUTE) {
                        ESP_LOGI(COMM_TAG, "Slave confirmed compute start.");
                        // Forward the slave's confirmation to the router.
                        if (send(sock, rx_buf, 1, 0) < 0) {
                            ESP_LOGE(COMM_TAG, "Failed to forward START_COMPUTE ACK to router.");
                            state = FSM_ERROR;
                            break;
                        }
                        // Transition to the compute polling state.
                        state = FSM_WAIT_COMPUTE;
                    } else {
                        ESP_LOGE(COMM_TAG, "Unexpected slave response during compute startup: 0x%02X", slave_response);
                        state = FSM_ERROR;
                        break;
                    }
                    break;
                }
                case PACKET_TYPE_POLL_COMPUTE:
                {
                    if (state != FSM_WAIT_COMPUTE) {
                        ESP_LOGE(COMM_TAG, "Unexpected POLL_COMPUTE packet in state %d", state);
                        state = FSM_ERROR;
                        break;
                    }
                    ESP_LOGI(COMM_TAG, "Received POLL_COMPUTE from router. Polling slave for compute status.");
                    
                    tx_buf[0] = PACKET_TYPE_POLL_COMPUTE;
                    if (spi_send_packet(t, tx_buf, rx_buf, NULL) != 0) {
                        ESP_LOGE(COMM_TAG, "SPI POLL_COMPUTE transaction failed.");
                        state = FSM_ERROR;
                        break;
                    }
                    
                    if (rx_buf[0] == PACKET_TYPE_FUC) {
                        ESP_LOGE(COMM_TAG, "Slave returned FUC during compute polling, first time itself");
                        state = FSM_ERROR;
                        break;
                    } else if (rx_buf[0] == PACKET_TYPE_END_COMPUTE) { 
                        uint8_t flag = rx_buf[1];
                        if (flag == 0xFF) {
                            ESP_LOGE(COMM_TAG, "Slave reports compute error during polling: no such file (flag 0xFF).");
                            if (send(sock, rx_buf, 2, 0) < 0) {
                                ESP_LOGE(COMM_TAG, "Failed to forward END_COMPUTE error to router.");
                            }
                            state = FSM_ERROR;
                        } else {
                            ESP_LOGI(COMM_TAG, "Compute complete; slave returning END_COMPUTE with flag: 0x%02X", flag);
                            if (send(sock, rx_buf, 2, 0) < 0) {
                                ESP_LOGE(COMM_TAG, "Failed to forward END_COMPUTE to router.");
                                state = FSM_ERROR;
                            } else {
                                state = FSM_COMPLETE;
                            }
                        }
                        break;
                    }

                    tx_buf[0] = PACKET_TYPE_POLL_COMPUTE;
                    if (spi_send_packet(t, tx_buf, rx_buf, NULL) != 0) {
                        ESP_LOGE(COMM_TAG, "SPI POLL_COMPUTE transaction failed.");
                        state = FSM_ERROR;
                        break;
                    }

                    uint8_t poll_response = rx_buf[0];
                    if (poll_response == PACKET_TYPE_WAIT_COMPUTE) {
                        ESP_LOGI(COMM_TAG, "Slave indicates compute is still in progress.");
                        if (send(sock, rx_buf, 1, 0) < 0) {
                            ESP_LOGE(COMM_TAG, "Failed to forward WAIT_COMPUTE to router.");
                            state = FSM_ERROR;
                        }
                        // Stay in FSM_WAIT_COMPUTE state.
                    } else if (poll_response == PACKET_TYPE_END_COMPUTE) {
                        uint8_t flag = rx_buf[1];
                        if (flag == 0xFF) {
                            ESP_LOGE(COMM_TAG, "Slave reports compute error during polling: no such file (flag 0xFF).");
                            if (send(sock, rx_buf, 2, 0) < 0) {
                                ESP_LOGE(COMM_TAG, "Failed to forward END_COMPUTE error to router.");
                            }
                            state = FSM_ERROR;
                        } else {
                            ESP_LOGI(COMM_TAG, "Compute complete; slave returning END_COMPUTE with flag: 0x%02X", flag);
                            if (send(sock, rx_buf, 2, 0) < 0) {
                                ESP_LOGE(COMM_TAG, "Failed to forward END_COMPUTE to router.");
                                state = FSM_ERROR;
                            } else {
                                state = FSM_COMPLETE;
                            }
                        }
                    } else if (poll_response == PACKET_TYPE_FUC) {
                        ESP_LOGE(COMM_TAG, "Slave returned FUC during compute polling.");
                        state = FSM_ERROR;
                    } else {
                        ESP_LOGE(COMM_TAG, "Unexpected slave response during compute polling: 0x%02X", poll_response);
                        state = FSM_ERROR;
                    }
                    break;
                }

                // ---------- Delete FSM handling ------------
                case PACKET_TYPE_DELETE:
                {
                    if (state != FSM_WAIT_START_WRITE) {
                        ESP_LOGE(COMM_TAG, "Unexpected DELETE packet in state %d", state);
                        state = FSM_ERROR;
                        break;
                    }
                    ESP_LOGI(COMM_TAG, "Received DELETE from router.");

                    uint8_t filename_payload[80];
                    if (recv_all(sock, filename_payload, sizeof(filename_payload)) != 0) {
                        ESP_LOGE(COMM_TAG, "Failed to read DELETE filename payload");
                        state = FSM_ERROR;
                        break;
                    }
                    char filename[80];
                    memcpy(filename, filename_payload, 79);
                    filename[79] = '\0';
                    ESP_LOGI(COMM_TAG, "DELETE: Filename: %s", filename);
                    
                    tx_buf[0] = PACKET_TYPE_DELETE;
                    tx_buf[1] = 0x00;
                    memcpy(tx_buf + 2, filename_payload, 80);

                    if (spi_send_packet(t, tx_buf, rx_buf, NULL) != 0) {
                        ESP_LOGE(COMM_TAG, "SPI DELETE transaction failed.");
                        state = FSM_ERROR;
                        break;
                    }

                    memset(tx_buf, 0, TRANSFER_SIZE);
                    // we do a second one to check if there 
                    if (spi_send_packet(t, tx_buf, rx_buf, NULL) != 0) {
                        ESP_LOGE(COMM_TAG, "SPI second transaction during DELETE failed.");
                        state = FSM_ERROR;
                        break;
                    }

                    uint8_t slave_response = rx_buf[0];
                    if (slave_response == PACKET_TYPE_FUC) {
                        ESP_LOGE(COMM_TAG, "Slave returned FUC during DELETE.");
                        if (send(sock, rx_buf, 1, 0) < 0) {
                            ESP_LOGE(COMM_TAG, "Failed to forward FUC to router.");
                        }
                        state = FSM_ERROR;
                        break;
                    } else if (slave_response == PACKET_TYPE_DELETE_RET) {
                        if (rx_buf[1] == 0xDD) { 
                            ESP_LOGI(COMM_TAG, "Slave reported DELETE completion (flag: 0xDD).");
                            if (send(sock, rx_buf, 2, 0) < 0) {
                                ESP_LOGE(COMM_TAG, "Failed to forward DELETE error to router.");
                            }
                            state =FSM_WAIT_START_WRITE; // SUCCESS!!
                        } else {
                            ESP_LOGE(COMM_TAG, "Wrong success code in DELETE_RET received.");
                            state = FSM_ERROR;
                            break;
                        }
                    } else {
                        ESP_LOGE(COMM_TAG, "Unexpected slave response during DELETE response: 0x%02X", slave_response);
                        state = FSM_ERROR;
                        break;
                    }
                    break;
                }

                // --------------------------------------------------
                case PACKET_TYPE_FUC: 
                {
                    state = FSM_ERROR;
                    break; 
                } 
                default:
                {
                    ESP_LOGE(COMM_TAG, "Unknown packet type received: 0x%02X", pkt_type);
                    state = FSM_ERROR;
                    break;
                }
            } 
        } 

        if (state == FSM_COMPLETE) {
            ESP_LOGI(COMM_TAG, "Image transfer complete. Total data received: %u bytes", received_data);
            memset(tx_buf, 0, TRANSFER_SIZE);
            if (spi_send_packet(t, tx_buf, rx_buf, expected_rx_buf) != 0)
            {
                ESP_LOGE(COMM_TAG, "FSM_COMPLETE: SPI DATA packet verification failed.");
                state = FSM_ERROR;
            }
            spi_send_packet_clear(t, tx_buf, rx_buf); 
        } 
        
        if (state == FSM_ERROR) {
            previous_spi_packet = 0;
            memset(tx_buf, 0, TRANSFER_SIZE);
            tx_buf[0] = PACKET_TYPE_FUC;
            if (send(sock, &tx_buf[0], 1, 0) < 0)
            {
                ESP_LOGE(COMM_TAG, "Failed to send END ACK");
                state = FSM_ERROR;
                break;
            }
            if (spi_send_packet(t, tx_buf, rx_buf, expected_rx_buf) != 0)
            {
                ESP_LOGE(COMM_TAG, "FSM_ERROR: SPI DATA packet verification failed.");
                state = FSM_ERROR;
                break;
            }
            tx_buf[0] = 0x00;
            if (spi_send_packet(t, tx_buf, rx_buf, expected_rx_buf) != 0)
            {
                ESP_LOGE(COMM_TAG, "FSM_ERROR: SPI DATA packet verification failed.");
                state = FSM_ERROR;
                break;
            }
            spi_send_packet_clear(t, tx_buf, rx_buf); 
            ESP_LOGE(COMM_TAG, "Image transfer error occurred.");
        }

        free(tx_buf);
        free(rx_buf);
        free(expected_rx_buf);
        shutdown(sock, 0);
        close(sock);
    } 

    close(listen_sock);
    vTaskDelete(NULL);
}


/** Event handler for Ethernet events */
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

/** Event handler for IP_EVENT_ETH_GOT_IP */
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
        .clock_config = { .rmii = { .clock_mode = EMAC_CLK_OUT, .clock_gpio = EMAC_CLK_OUT_180_GPIO } }, // ESP32 WROOM
        // .clock_config = { .rmii = { .clock_mode = EMAC_CLK_OUT, .clock_gpio = EMAC_TX_CLK } }, // ESP32 WROVER IE 
        // .clock_config = { .rmii = { .clock_mode = EMAC_CLK_EXT_IN, .clock_gpio = EMAC_APPL_CLK_OUT_GPIO } }, // ESP32 WROVER IE Input
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
    ESP_LOGI(ETH_TAG, "ETHERNET INITIALIZED!");

    esp_err_t ret = spi_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(COMM_TAG, "SPI MASTER INITIALIZATION FAILED!: %s", esp_err_to_name(ret));
        return;
    }
    handshake_init_master();
    ESP_LOGI(COMM_TAG, "SPI MASTER INITIALIZED!");

    xTaskCreate(tcp_server_task, "tcp_handler", 4096, NULL, 5, NULL);
    ESP_LOGI(ETH_TAG, "TCP HANDLER INITIALIZED!");
}

