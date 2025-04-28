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

static EventGroupHandle_t s_eth_event_group;

#define ETHERNET_CONNECTED_BIT  BIT0
#define ETHERNET_FAIL_BIT       BIT1

#define STATIC_IP               1

#if STATIC_IP
    #define S_IP        "192.168.0.50"     
    #define GATEWAY     "192.168.0.1"    
    #define NETMASK     "255.255.255.0"
#endif /* STATIC_IP */

static netif_input_fn default_input = NULL;


// Make sure your Ethernet setup functions are defined somewhere, for example:
extern void ethernet_setup(void);   // Your Ethernet initialization code
// And a global event group or similar mechanism used in ethernet_setup, if any.

// Packet type definitions
#define PACKET_TYPE_START 0xA0
#define PACKET_TYPE_DATA  0xA1
#define PACKET_TYPE_MID   0xA2
#define PACKET_TYPE_END   0xA3

#define PORT 8080
static const char *TAG = "TCP_SOCKET";

// FSM states
typedef enum {
    FSM_WAIT_START,
    FSM_WAIT_DATA,    // After START received, before MID or END
    FSM_WAIT_MID,     // After MID marker received (optional state; we simply mark that MID is reached)
    FSM_WAIT_END,
    FSM_COMPLETE,
    FSM_ERROR
} fsm_state_t;

// ################################################################

// SPI pins (must match the slave configuration)
#define PIN_NUM_MOSI  13
#define PIN_NUM_MISO  12
#define PIN_NUM_CLK   14
#define PIN_NUM_CS    15

// Handshake pin number (slave drives this, master reads it)
#define GPIO_HANDSHAKE 4

// Transfer size: 2048 bytes.
#define TRANSFER_SIZE 1024

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


/**
 * @brief Send a packet over SPI and verify that the slave echoes it correctly.
 *
 * This function builds a full TRANSFER_SIZE packet. The first byte is set to packet_type.
 * If payload is provided (and payload_len > 0 and <= TRANSFER_SIZE-1), it is copied into the TX buffer.
 * The remaining bytes are filled with random data.
 *
 * The function then performs two SPI transactions (send and echo read) with up to MAX_RETRY attempts.
 *
 * @param packet_type The command/packet type to send (placed in byte 0).
 * @param payload Optional payload data (can be NULL).
 * @param payload_len Length of payload data.
 *
 * @return 0 on success, -1 on failure.
 */
int spi_send_packet(uint8_t packet_type, uint8_t *payload, size_t payload_len)
{
    if (payload_len > (TRANSFER_SIZE - 1)) {
        ESP_LOGE(TAG, "Payload too large: %d bytes (max %d)", payload_len, TRANSFER_SIZE - 1);
        return -1;
    }

    // Allocate buffers from the heap instead of the stack.
    uint8_t *tx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
    uint8_t *rx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
    if (!tx_buf || !rx_buf) {
        ESP_LOGE(TAG, "Heap allocation failed for SPI buffers");
        if (tx_buf) free(tx_buf);
        if (rx_buf) free(rx_buf);
        return -1;
    }

    // Build TX buffer: set first byte as packet_type.
    tx_buf[0] = packet_type;
    if (payload && payload_len > 0) {
        memcpy(&tx_buf[1], payload, payload_len);
    }
    // Fill remaining bytes with random data.
    for (int i = 1 + payload_len; i < TRANSFER_SIZE; i++) {
        tx_buf[i] = (uint8_t)0;
    }

    int attempt;
    esp_err_t ret;
    spi_transaction_t t = {0};

    for (attempt = 0; attempt < MAX_RETRY; attempt++) {
        wait_for_slave_ready();

        // Transaction 1: Send the full TX buffer.
        memset(&t, 0, sizeof(t));
        t.length = TRANSFER_SIZE * 8;
        t.tx_buffer = tx_buf;
        t.rx_buffer = NULL;
        ret = spi_device_polling_transmit(spi_master_handle, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transmit (send) failed on attempt %d: %s", attempt+1, esp_err_to_name(ret));
            continue;
        }
        ESP_LOGI(TAG, "SPI send (attempt %d) successful", attempt+1);

        // Transaction 2: Read echo from the slave.
        memset(&t, 0, sizeof(t));
        t.length = TRANSFER_SIZE * 8;
        t.tx_buffer = tx_buf;  // Dummy TX data.
        t.rx_buffer = rx_buf;
        ret = spi_device_polling_transmit(spi_master_handle, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI echo read failed on attempt %d: %s", attempt+1, esp_err_to_name(ret));
            continue;
        }

        // Compare echoed data.
        int mismatches = 0;
        for (int i = 0; i < TRANSFER_SIZE; i++) {
            if (rx_buf[i] != tx_buf[i]) {
                mismatches++;
            }
        }
        if (mismatches == 0) {
            ESP_LOGI(TAG, "\033[0;32mSPI packet echo verified on attempt %d.\033[0m", attempt+1);
            free(tx_buf);
            free(rx_buf);
            return 0;
        } else {
            ESP_LOGW(TAG, "Attempt %d: %d mismatches in echoed data.", attempt+1, mismatches);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    ESP_LOGE(TAG, "SPI packet echo failed after %d attempts.", MAX_RETRY);
    free(tx_buf);
    free(rx_buf);
    return -1;
}


int spi_send_packet_clear()
{
    uint8_t *tx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
    uint8_t *rx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
    if (!tx_buf || !rx_buf) {
        ESP_LOGE(TAG, "Heap allocation failed for SPI buffers");
        if (tx_buf) free(tx_buf);
        if (rx_buf) free(rx_buf);
        return -1;
    }

    for (int i = 0; i < TRANSFER_SIZE; i++) {
        tx_buf[i] = 0;
    }

    spi_transaction_t t = {0};
    esp_err_t ret;
    int attempt;
    attempt = -1; 

    wait_for_slave_ready();

    // Single transaction: send tx_buf and receive rx_buf.
    memset(&t, 0, sizeof(t));
    t.length = TRANSFER_SIZE * 8;  // in bits.
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    ret = spi_device_polling_transmit(spi_master_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed on attempt %d: %s", attempt+1, esp_err_to_name(ret));
        return -1; 
    }
    ESP_LOGI(TAG, "CLEARING: Echoed data: 0x%02X 0x%02X 0x%02X ...", rx_buf[0], rx_buf[1], rx_buf[2]);

    free(tx_buf);
    free(rx_buf);
    return 0;
}

int spi_send_packet_v2(uint8_t packet_type, uint8_t previous_packet_type, uint8_t *payload, size_t payload_len)
{
    if (payload_len > (TRANSFER_SIZE - 1)) {
        ESP_LOGE(TAG, "Payload too large: %d bytes (max %d)", payload_len, TRANSFER_SIZE - 1);
        return -1;
    }

    // Allocate TX and RX buffers from heap (DMA-capable if needed).
    uint8_t *tx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
    uint8_t *rx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
    if (!tx_buf || !rx_buf) {
        ESP_LOGE(TAG, "Heap allocation failed for SPI buffers");
        if (tx_buf) free(tx_buf);
        if (rx_buf) free(rx_buf);
        return -1;
    }

    // Build the TX buffer.
    tx_buf[0] = packet_type;
    if (payload && payload_len > 0) {
        memcpy(&tx_buf[1], payload, payload_len);
    }
    for (int i = 1 + payload_len; i < TRANSFER_SIZE; i++) {
        tx_buf[i] = (uint8_t) esp_random();
    }

    spi_transaction_t t = {0};
    esp_err_t ret;
    int attempt;
    attempt = -1; 

    wait_for_slave_ready();

    // Single transaction: send tx_buf and receive rx_buf.
    memset(&t, 0, sizeof(t));
    t.length = TRANSFER_SIZE * 8;  // in bits.
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    ESP_LOGI(TAG, "Sending data: 0x%02X 0x%02X 0x%02X ...", tx_buf[0], tx_buf[1], tx_buf[2]);

    ret = spi_device_polling_transmit(spi_master_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed on attempt %d: %s", attempt+1, esp_err_to_name(ret));
        return -1; 
    }
    ESP_LOGI(TAG, "Echoed data: 0x%02X 0x%02X 0x%02X ...", rx_buf[0], rx_buf[1], rx_buf[2]);

    if (previous_packet_type != 0 && rx_buf[0] != previous_packet_type) {
        ESP_LOGE(TAG, "SPI echo mismatch on attempt %d: expected 0x%02X, got 0x%02X", 
                    attempt+1, previous_packet_type, rx_buf[0]);
        vTaskDelay(pdMS_TO_TICKS(10));
        return -1; 
    }

    ESP_LOGI(TAG, "\033[0;32mSPI packet (0x%02X) verified on attempt %d.\033[0m", previous_packet_type, attempt+1);
    
    free(tx_buf);
    free(rx_buf);
    return 0;
}

void build_tx_buffer(uint8_t *buf, uint8_t *pl, uint8_t packet_type, size_t pl_len) {
    buf[0] = packet_type;
    if (pl && pl_len > 0) {
        memcpy(&buf[1], pl, pl_len);
    }
    for (int i = 1 + pl_len; i < TRANSFER_SIZE; i++) {
        buf[i] = (uint8_t) esp_random();
    }
}

int spi_send_packet_v3(uint8_t packet_type,
                       uint8_t previous_packet_type,
                       uint8_t *payload, size_t payload_len,
                       uint8_t *backup_payload, size_t backup_payload_len)
{
    if (payload_len > (TRANSFER_SIZE - 1)) {
        ESP_LOGE(TAG, "Payload too large: %d bytes (max %d)", payload_len, TRANSFER_SIZE - 1);
        return -1;
    }
    // Only check backup length if a backup payload is provided (nonzero length).
    if (payload && backup_payload_len != 0 && (backup_payload_len < payload_len)) {
        ESP_LOGE(TAG, "Backup payload length (%d) is less than payload length (%d)", backup_payload_len, payload_len);
        return -1;
    }

    // Allocate TX and RX buffers from the heap (DMA-capable if needed).
    uint8_t *tx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
    uint8_t *rx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
    if (!tx_buf || !rx_buf) {
        ESP_LOGE(TAG, "Heap allocation failed for SPI buffers");
        if (tx_buf) free(tx_buf);
        if (rx_buf) free(rx_buf);
        return -1;
    }

    spi_transaction_t t = {0};
    esp_err_t ret;

    // --- Attempt 1: Send current payload ---
    build_tx_buffer(tx_buf, payload, packet_type, payload_len);
    wait_for_slave_ready();
    memset(&t, 0, sizeof(t));
    t.length = TRANSFER_SIZE * 8;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    ESP_LOGI(TAG, "Sending packet 0x%02X (current payload)", packet_type);
    ret = spi_device_polling_transmit(spi_master_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed on current payload attempt: %s", esp_err_to_name(ret));
        free(tx_buf); free(rx_buf);
        return -1;
    }
    ESP_LOGI(TAG, "Echoed data: 0x%02X 0x%02X 0x%02X ...", rx_buf[0], rx_buf[1], rx_buf[2]);

    // If previous_packet_type is nonzero, then we check:
    // - Always verify rx_buf[0] matches previous_packet_type.
    // - Verify the payload if backup_payload_len is nonzero.
    if (previous_packet_type != 0) {
        if (rx_buf[0] != previous_packet_type ||
            (payload_len > 0 && backup_payload_len > 0 &&
             memcmp(&rx_buf[1], backup_payload, payload_len) != 0)) {
            ESP_LOGE(TAG, "SPI echo mismatch on current payload: expected cmd 0x%02X, got 0x%02X", 
                     previous_packet_type, rx_buf[0]);
            ESP_LOGI(TAG, "Echoed data: 0x%02X 0x%02X 0x%02X ...", rx_buf[0], rx_buf[1], rx_buf[2]);
            // Now try resending the backup payload up to MAX_RETRY times.
            int attempt;
            for (attempt = 0; attempt < MAX_RETRY; attempt++) {
                build_tx_buffer(tx_buf, backup_payload, previous_packet_type, payload_len);
                wait_for_slave_ready();
                memset(&t, 0, sizeof(t));
                t.length = TRANSFER_SIZE * 8;
                t.tx_buffer = tx_buf;
                t.rx_buffer = rx_buf;

                ESP_LOGI(TAG, "Retrying backup payload attempt %d for packet 0x%02X", attempt+1, packet_type);
                ret = spi_device_polling_transmit(spi_master_handle, &t);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "SPI transaction failed on backup payload attempt %d: %s", attempt+1, esp_err_to_name(ret));
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                if (rx_buf[0] == previous_packet_type &&
                    (payload_len == 0 || 
                     (backup_payload_len > 0 && memcmp(&rx_buf[1], backup_payload, payload_len) == 0))) {
                    ESP_LOGI(TAG, "\033[0;32mBackup payload verified on attempt %d.\033[0m", attempt+1);
                    // Update backup_payload with current payload for future use.
                    if (payload && payload_len > 0) {
                        memcpy(backup_payload, payload, payload_len);
                    }
                    free(tx_buf); free(rx_buf);
                    return 0;
                } else {
                    ESP_LOGE(TAG, "Backup payload echo mismatch on attempt %d: expected 0x%02X", attempt+1, previous_packet_type);
                    ESP_LOGI(TAG, "Echoed data: 0x%02X 0x%02X 0x%02X ...", rx_buf[0], rx_buf[1], rx_buf[2]);
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
            ESP_LOGE(TAG, "Backup payload failed after %d attempts.", MAX_RETRY);
            free(tx_buf); free(rx_buf);
            return -1;
        }
    }
    ESP_LOGI(TAG, "\033[0;32mSPI packet 0x%02X verified on current payload.\033[0m", packet_type);
    
    free(tx_buf);
    free(rx_buf);
    return 0;
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
        uint8_t previous_spi_packet = 0; // For the very first transaction, no previous packet is expected.
        uint16_t previous_spi_paacket_len = 0; 
        uint8_t *previous_data_chunk = NULL; 

        uint8_t *tx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
        uint8_t *rx_buf = heap_caps_malloc(TRANSFER_SIZE, MALLOC_CAP_DMA);
        if (!tx_buf || !rx_buf) {
            ESP_LOGE(TAG, "Heap allocation failed for SPI buffers");
            if (tx_buf) free(tx_buf);
            if (rx_buf) free(rx_buf);
            return -1;
        }
        memset(tx_buf, 0, TRANSFER_SIZE);
        memset(rx_buf, 0, TRANSFER_SIZE);

        uint8_t *data_chunk; 

        while (state != FSM_COMPLETE && state != FSM_ERROR) {
            uint8_t pkt_type;
            if (recv_all(sock, &pkt_type, 1) != 0) {
                ESP_LOGE(TAG, "Failed to read packet type");
                state = FSM_ERROR;
                break;
            }
            switch(pkt_type) {
                case PACKET_TYPE_START:
                {
                    if (state != FSM_WAIT_START) {
                        ESP_LOGE(TAG, "Unexpected START packet");
                        state = FSM_ERROR;
                        break;
                    }
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

                    if (spi_send_packet_v2(PACKET_TYPE_START, previous_spi_packet, NULL, 0) != 0) {
                        ESP_LOGE(TAG, "SPI START packet verification failed.");
                        state = FSM_ERROR;
                        break;
                    }
                    // if (spi_send_packet_v3(PACKET_TYPE_START, previous_spi_packet,
                    //                         NULL, 0, NULL, 0) != 0) {
                    //     ESP_LOGE(TAG, "SPI DATA packet verification failed.");
                    //     free(data_chunk);
                    //     state = FSM_ERROR;
                    //     break;
                    // }
                    previous_spi_packet = PACKET_TYPE_START;

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
                    ESP_LOGI(TAG, "In PACKET_TYPE_DATA");
                    // DATA packet: next 2 bytes: data length, then payload
                    uint8_t len_buf[2];
                    if (recv_all(sock, len_buf, sizeof(len_buf)) != 0) {
                        ESP_LOGE(TAG, "Failed to read DATA length");
                        state = FSM_ERROR;
                        break;
                    }
                    uint16_t data_len = (len_buf[0] << 8) | len_buf[1];
                    if (data_len > 0) {
                        data_chunk = malloc(data_len);
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
                        ESP_LOGI(TAG, "IN PACKET_TYPE_DATA, data_len: %d", data_len);


                        if (spi_send_packet_v2(PACKET_TYPE_DATA, previous_spi_packet, data_chunk, data_len) != 0) {
                            ESP_LOGE(TAG, "SPI DATA packet verification failed.");
                            free(data_chunk);
                            state = FSM_ERROR;
                            break;
                        }
                        // if (spi_send_packet_v3(PACKET_TYPE_DATA, previous_spi_packet,
                        //                         data_chunk, data_len, previous_data_chunk, previous_spi_paacket_len) != 0) {
                        //     ESP_LOGE(TAG, "SPI DATA packet verification failed.");
                        //     free(data_chunk);
                        //     state = FSM_ERROR;
                        //     break;
                        // }
                        previous_spi_packet = PACKET_TYPE_DATA;
                        previous_data_chunk = data_chunk;
                        previous_spi_paacket_len = data_len;

                        for (int i = 0; i < data_len; i++) {
                            computed_checksum += data_chunk[i];
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

                    if (spi_send_packet_v2(PACKET_TYPE_MID, previous_spi_packet, NULL, 0) != 0) {
                        ESP_LOGE(TAG, "SPI MID packet verification failed.");
                        state = FSM_ERROR;
                        break;
                    }
                    // if (spi_send_packet_v3(PACKET_TYPE_MID, previous_spi_packet,
                    //                         NULL, 0, previous_data_chunk, previous_spi_paacket_len) != 0) {
                    //     ESP_LOGE(TAG, "SPI MID packet verification failed.");
                    //     free(previous_data_chunk);
                    //     state = FSM_ERROR;
                    //     break;
                    // }
                    previous_spi_packet = PACKET_TYPE_MID;
                    previous_data_chunk = NULL;
                    previous_spi_paacket_len = 0;

                    if (send(sock, &pkt_type, 1, 0) < 0) {
                        ESP_LOGE(TAG, "Failed to send MID ACK");
                        state = FSM_ERROR;
                        break;
                    }
                    state = FSM_WAIT_DATA; // Continue receiving data
                    break;
                }
                case PACKET_TYPE_END:
                {
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
                        pkt_type = PACKET_TYPE_END; 
    
                        if (spi_send_packet_v2(PACKET_TYPE_END, previous_spi_packet, NULL, 0) != 0) {
                            ESP_LOGE(TAG, "SPI END packet verification failed.");
                            state = FSM_ERROR;
                            break;
                        }
                        // if (spi_send_packet_v3(PACKET_TYPE_END, previous_spi_packet,
                        //                         NULL, 0, NULL, 0) != 0) {
                        //     ESP_LOGE(TAG, "END packet verification failed.");
                        //     state = FSM_ERROR;
                        //     break;
                        // }
                        previous_spi_packet = PACKET_TYPE_END;

                        if (send(sock, &pkt_type, 1, 0) < 0)
                        {
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
            } 
        } 

        if (state == FSM_COMPLETE) {
            ESP_LOGI(TAG, "Image transfer complete. Total data received: %u bytes", received_data);
        } else {
            previous_spi_packet = 0;
            spi_send_packet_clear(); 
            spi_send_packet_clear(); 
            ESP_LOGE(TAG, "Image transfer error occurred.");
        }

        shutdown(sock, 0);
        close(sock);
    } // end while listening

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
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    ESP_LOGI(ETH_TAG, "Started TCP server task");
    esp_err_t ret = spi_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI master initialization failed");
        return;
    }
    handshake_init_master();
    ESP_LOGI(TAG, "SPI master initialized");


}