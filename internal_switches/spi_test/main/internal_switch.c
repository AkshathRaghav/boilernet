#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/spi_slave.h"
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"



// ##############################################################################

static const char *COMM_TAG = "SPI_SLAVE";

#define RCV_HOST    VSPI_HOST

#define GPIO_MOSI     23
#define GPIO_MISO     19
#define GPIO_SCLK     18
#define GPIO_CS       5

#define GPIO_HANDSHAKE 4

#define TRANSFER_SIZE 2048

void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 1);
}

void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 0);
}

void handshake_init_slave(void)
{
    // Configure handshake pin as output.
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
}

static esp_err_t spi_slave_init(void)
{
    esp_err_t ret;

    handshake_init_slave();

    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TRANSFER_SIZE,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb
    };

    // Enable pull-ups on SPI lines so that the signals are stable when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS,   GPIO_PULLUP_ONLY);

    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    return ret; 
}


// ##############################################################################

#define RCV_HOST2 HSPI_HOST   

#define PACKET_TYPE_START_WRITE 0xA0
#define PACKET_TYPE_DATA_WRITE  0xA1
#define PACKET_TYPE_MID_WRITE   0xA2
#define PACKET_TYPE_END_WRITE   0xA3
#define PACKET_TYPE_FUC   0xA4

#define PACKET_TYPE_START_READ 0xB0 // new
#define PACKET_TYPE_DATA_READ  0xB1 // new
#define PACKET_TYPE_END_READ   0xB3 // new

typedef enum {
    FSM_WAIT_START_WRITE,
    FSM_WAIT_DATA_WRITE,
    FSM_WAIT_START_READ,    // <-- New:
    FSM_WAIT_DATA_READ,     // <-- New: 
    FSM_WAIT_END_READ,      // <-- New: 
    FSM_COMPLETE,
    FSM_ERROR
} fsm_state_t;

// ##############################################################################

#define MOUNT_POINT "/sdcard"

#define PIN_NUM_MISO  12
#define PIN_NUM_MOSI  13
#define PIN_NUM_CLK   14
#define PIN_NUM_CS    15

// ##############################################################################

static esp_err_t SDCardInit(void) {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2048,
    };

    host.max_freq_khz = 5200;
    int ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE("SDCARD", "Failed to initialize SPI bus for SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE("SDCARD", "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("SDCARD", "SD Card mounted at: %s", MOUNT_POINT);
    return ESP_OK;
}

static FILE *sdcard_open(const char *filename, const char *mode) {
    char path[128];
    snprintf(path, sizeof(path), "%s/%s", MOUNT_POINT, filename);
    return fopen(path, mode);
}

static size_t sdcard_write(FILE *file, const void *buffer, size_t size) {
    return fwrite(buffer, 1, size, file);
}

static int sdcard_read(FILE *file, void *buffer, size_t size) {
    if (file == NULL) {
        return -1;
    }
    size_t bytes_read = fread(buffer, 1, size, file);
    if (ferror(file)) {
        return -1;
    }
    return bytes_read;
}


static void sdcard_close(FILE *file) {
    fclose(file);
}

static void sdcard_delete(const char *filename) {
    char path[128];
    snprintf(path, sizeof(path), "%s/%s", MOUNT_POINT, filename);
    remove(path);
}

// ##############################################################################

static void spi_handler_task(void *pvParameters) { 
    uint8_t *rx_buf_parent = spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0);
    uint8_t *rx_buf         = spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0);
    uint8_t *tx_buf         = spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0);

    if (!rx_buf || !tx_buf) {
        ESP_LOGE(COMM_TAG, "Failed to allocate DMA buffers");
        return;
    }
    memset(tx_buf, 0, TRANSFER_SIZE);
    memset(rx_buf_parent, 0, TRANSFER_SIZE);
    
    fsm_state_t fsm_state = FSM_WAIT_START_WRITE;
    unsigned int computed_checksum = 0;
    FILE *sd_file = NULL;
    char filename[64] = {0};

    int count = 0; 
    spi_slave_transaction_t t_recv = {0};
    t_recv.length = TRANSFER_SIZE * 8;

    int file_opened = 0; 


    while (1) {
        memset(rx_buf, 0, TRANSFER_SIZE);

        ESP_LOGI(COMM_TAG, "%d: Sent buffer; first bytes: 0x%02X 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X", 
                 count, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], 
                 tx_buf[TRANSFER_SIZE - 2], tx_buf[TRANSFER_SIZE - 1]);
                 
        t_recv.tx_buffer = tx_buf;
        t_recv.rx_buffer = rx_buf;

        esp_err_t ret = spi_slave_transmit(RCV_HOST, &t_recv, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(COMM_TAG, "Receive transaction failed: %s", esp_err_to_name(ret));
            continue;
        }


        ESP_LOGI(COMM_TAG, "%d: Received buffer; first bytes: 0x%02X 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X", 
                 count, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], 
                 rx_buf[TRANSFER_SIZE - 2], rx_buf[TRANSFER_SIZE - 1]);

        if ((tx_buf[1] == 0xFF) && (fsm_state == FSM_WAIT_END_READ)) { 
            fsm_state = FSM_WAIT_START_WRITE;
            memset(tx_buf, 0, TRANSFER_SIZE);

        }

        // if (fsm_state == FSM_WAIT_START_WRITE) { 
        // }

        // ##########################################################
        uint8_t pkt_type = rx_buf[0];
        switch (pkt_type) {

            // ---------- Write FSM handling  ------------
            case PACKET_TYPE_START_WRITE:   
            {
                if (fsm_state != FSM_WAIT_START_WRITE)
                {
                    ESP_LOGE(COMM_TAG, "Unexpected START packet received in state %d", fsm_state);
                    fsm_state = FSM_ERROR;
                    break;
                }
                // Byte 0: packet type, Byte 1: CHECK_BYTE, Bytes [2..49]: filename (48 bytes)
                memcpy(filename, &rx_buf[2], 48);
                filename[47] = '\0';  
                sd_file = sdcard_open(filename, "w");
                if (sd_file == NULL) {
                    ESP_LOGE(COMM_TAG, "Failed to open file on SD card: %s", filename);
                    fsm_state = FSM_ERROR;
                } else {
                    ESP_LOGI(COMM_TAG, "START packet: Opened file %s for writing", filename);
                    fsm_state = FSM_WAIT_DATA_WRITE;
                    computed_checksum = 0;  
                }
                break;
            }
            case PACKET_TYPE_DATA_WRITE:
            {
                if (fsm_state != FSM_WAIT_DATA_WRITE) {
                    ESP_LOGE(COMM_TAG, "DATA packet received out of sequence");
                    fsm_state = FSM_ERROR;
                    break;
                } else {
                    // Byte 0: Packet type, Byte 1: CHECK_BYTE, Bytes [2-3]: DATA LEN, Bytes [4...]: Data payload
                    uint16_t data_len = (rx_buf[2] << 8) | rx_buf[3];
                    if (data_len > 0 && data_len <= (TRANSFER_SIZE - 4)) {
                        size_t written = sdcard_write(sd_file, &rx_buf[4], data_len);
                        if (written != data_len) {
                            ESP_LOGE(COMM_TAG, "SD card write error. Expected %d bytes, wrote %d", data_len, written);
                            fsm_state = FSM_ERROR;
                        } else {
                            for (int i = 0; i < data_len; i++) {
                                computed_checksum += rx_buf[4 + i];
                            }
                            ESP_LOGI(COMM_TAG, "DATA packet processed: %d bytes written", data_len);
                        }
                    } else {
                        ESP_LOGE(COMM_TAG, "Invalid DATA packet length: %d", data_len);
                        fsm_state = FSM_ERROR;
                    }
                }
                break;
            }
            case PACKET_TYPE_MID_WRITE:
            {
                if (fsm_state != FSM_WAIT_DATA_WRITE) {
                    ESP_LOGE(COMM_TAG, "MID packet received out of sequence");
                    fsm_state = FSM_ERROR;
                } else {
                    ESP_LOGI(COMM_TAG, "MID packet received (ignored)");
                }
                break;
            }
            case PACKET_TYPE_END_WRITE:
            {
                if (fsm_state != FSM_WAIT_DATA_WRITE) {
                    ESP_LOGE(COMM_TAG, "END packet received out of sequence");
                    fsm_state = FSM_ERROR;
                } else {
                    // Byte 0: packet type, Byte 1: CHECK_BYTE, Bytes [2-5]: checksum (big-endian)
                    unsigned int received_checksum = (rx_buf[2] << 24) | (rx_buf[3] << 16) |
                                                     (rx_buf[4] << 8)  | rx_buf[5];
                    ESP_LOGI(COMM_TAG, "END packet received. Received checksum: 0x%08X, Computed checksum: 0x%08X",
                             received_checksum, computed_checksum);
                    if (received_checksum != computed_checksum) {
                        ESP_LOGE(COMM_TAG, "Checksum mismatch");
                        fsm_state = FSM_ERROR;
                    } else {
                        ESP_LOGI(COMM_TAG, "File transfer complete: %s", filename);
                        file_opened = 0; 
                        fsm_state = FSM_COMPLETE;
                    }
                }
                break;
            }
            case PACKET_TYPE_FUC:
            {
                ESP_LOGE(COMM_TAG, "FUC packet received. Transaction failed.");
                if (sd_file != NULL) {
                    sdcard_close(sd_file);
                    sdcard_delete(filename);
                    sd_file = NULL;
                }
                fsm_state = FSM_ERROR;
                break;
            }

            // ---------- Read FSM handling ------------
            case PACKET_TYPE_START_READ:
            {
                if (fsm_state != FSM_WAIT_START_WRITE) { // FSM_WAIT_START_WRITE is being used as an IDLE State here. Not the best logic lol.
                    ESP_LOGE(COMM_TAG, "Unexpected START_READ packet received in state %d", fsm_state);
                    fsm_state = FSM_ERROR;
                    break;
                }
                // Byte 0: packet type, Byte 1: CHECK_BYTE, Bytes [2..49]: filename (48 bytes)
                memcpy(filename, &rx_buf[2], 48);
                filename[47] = '\0';
                sd_file = sdcard_open(filename, "r");
                if (sd_file == NULL) {
                    ESP_LOGE(COMM_TAG, "Failed to open file for reading: %s", filename);
                    // Even if the file is not found, transition into READ state
                    // so we can send an error checksum in the END_READ packet.
                    fsm_state = FSM_WAIT_END_READ;
                    tx_buf[0] = PACKET_TYPE_END_READ;
                    tx_buf[1] = 0xFF;
                    file_opened = 0; 
                }
                else
                {
                    ESP_LOGI(COMM_TAG, "START_READ: Opened file %s for reading", filename);
                    fsm_state = FSM_WAIT_DATA_READ;
                    computed_checksum = 0;
                    tx_buf[0] = PACKET_TYPE_START_READ; 
                    file_opened = 1;
                }

                break;
            }
            case PACKET_TYPE_DATA_READ:
            {
                if (fsm_state == FSM_WAIT_START_WRITE) {
                    continue; 
                }
                else if (fsm_state == FSM_WAIT_END_READ)
                {
                    if (!file_opened) { 
                        ESP_LOGE(COMM_TAG, "DATA_READ packet received out of sequence, fsmwaitendread");
                        fsm_state = FSM_ERROR;
                        break;
                    } else {
                        tx_buf[0] = PACKET_TYPE_END_READ;
                        tx_buf[1] = (computed_checksum >> 24) & 0xFF;
                        tx_buf[2] = (computed_checksum >> 16) & 0xFF;
                        tx_buf[3] = (computed_checksum >> 8) & 0xFF;
                        tx_buf[4] = computed_checksum & 0xFF;
                        
                        ESP_LOGI(COMM_TAG, "END_READ packet sent: checksum = 0x%08X", computed_checksum);
                        fsm_state = FSM_COMPLETE;
                    }
                }
                else if (fsm_state != FSM_WAIT_DATA_READ)
                {
                    ESP_LOGE(COMM_TAG, "DATA_READ packet received out of sequence, fsmwaitdataread");
                    fsm_state = FSM_ERROR;
                    break;
                }
                else
                {
                    // In READ mode, instead of writing file data,
                    // the slave reads from the file and puts data into tx_buf.
                    // We reserve 3 bytes for the header (1 for packet type, 2 for length).
                    int file_bytes = sdcard_read(sd_file, &tx_buf[3], TRANSFER_SIZE - 3);
                    if (file_bytes < 0) {
                        ESP_LOGE(COMM_TAG, "Error reading file for DATA_READ");
                        fsm_state = FSM_ERROR;
                        break;
                    }
                    // Prepare the DATA_READ packet in tx_buf.
                    tx_buf[0] = PACKET_TYPE_DATA_READ;
                    tx_buf[1] = (file_bytes >> 8) & 0xFF;
                    tx_buf[2] = file_bytes & 0xFF;
                    
                    // Update checksum with the file data that is sent.
                    for (int i = 0; i < file_bytes; i++) {
                        computed_checksum += tx_buf[3 + i];
                    }
                    
                    ESP_LOGI(COMM_TAG, "DATA_READ packet prepared: %d bytes read", file_bytes);
                    
                    // If we read less than the maximum possible bytes, assume EOF.
                    if (file_bytes < (TRANSFER_SIZE - 3)) {
                        fsm_state = FSM_WAIT_END_READ;
                    }
                }
                break;
            }
            case PACKET_TYPE_END_READ: // I don't think we come here at all, we finish in the DATA_READ && file_opened 
            {
                if (fsm_state != FSM_WAIT_END_READ) {
                    ESP_LOGE(COMM_TAG, "END_READ packet received out of sequence");
                    fsm_state = FSM_ERROR;
                } else {
                    // Build END_READ packet: [Packet Type (1 byte)] + [Checksum (4 bytes)]
                    tx_buf[0] = PACKET_TYPE_END_READ;
                    tx_buf[1] = (computed_checksum >> 24) & 0xFF;
                    tx_buf[2] = (computed_checksum >> 16) & 0xFF;
                    tx_buf[3] = (computed_checksum >> 8) & 0xFF;
                    tx_buf[4] = computed_checksum & 0xFF;
                    
                    if (sd_file != NULL) {
                        sdcard_close(sd_file);
                        sd_file = NULL;
                    }
                    ESP_LOGI(COMM_TAG, "END_READ packet sent: checksum = 0x%08X", computed_checksum);
                    fsm_state = FSM_COMPLETE;
                }
                break;
            }

            // --------------------------------------------------
            default: 
            {
                if ((pkt_type == 0x00)) { 
                    continue; 
                }   
                ESP_LOGE(COMM_TAG, "Unknown packet type received: 0x%02X, fsm_state: 0x%02X", pkt_type, fsm_state);
                fsm_state = FSM_ERROR;
                break;
            }
        }

        // ##########################################################
        
        if ((fsm_state != FSM_WAIT_START_READ) && (fsm_state != FSM_WAIT_DATA_READ) && (fsm_state != FSM_WAIT_END_READ) && !((fsm_state == FSM_COMPLETE) && (file_opened))) {
            memcpy(tx_buf, rx_buf, TRANSFER_SIZE);
        }
        if ((fsm_state == FSM_COMPLETE) && (!file_opened)) { 
            memcpy(tx_buf, rx_buf, TRANSFER_SIZE);
        }



        // ##########################################################
        if (fsm_state == FSM_COMPLETE) {
            computed_checksum = 0;
            if (sd_file != NULL) {
                sdcard_close(sd_file);
                sd_file = NULL;
            }
            memset(filename, 0, sizeof(filename));
            ESP_LOGI(COMM_TAG, "FSM completed successfully. Ready for next transaction.");
            fsm_state = FSM_WAIT_START_WRITE;
        } else if (fsm_state == FSM_ERROR) {
            ESP_LOGE(COMM_TAG, "FSM encountered an error. Cleaning up and resetting.");
            if (sd_file != NULL) {
                sdcard_close(sd_file);
                sdcard_delete(filename);
                sd_file = NULL;
            }
            fsm_state = FSM_WAIT_START_WRITE;
            tx_buf[0] = PACKET_TYPE_FUC;
            computed_checksum = 0;
            memset(filename, 0, sizeof(filename));
        }

        count = (count == 1) ? 0 : 1;
    }
}


// ##############################################################################

void app_main(void)
{

    esp_err_t sdret = SDCardInit(); 
    if (sdret != 0)
    {
        ESP_LOGE(COMM_TAG, "SDCARD INITIALIZATION FAILED!: %s", esp_err_to_name(sdret));
        return;
    }
    ESP_LOGI(COMM_TAG, "SDCARD INITIALIZED!");

    esp_err_t ret = spi_slave_init(); 
    if (ret != ESP_OK)
    {
        ESP_LOGE(COMM_TAG, "SPI SLAVE INITIALIZATION FAILED!: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(COMM_TAG, "SPI SLAVE INITIALIZED!");

    xTaskCreate(spi_handler_task, "spi_handler", 4096, NULL, 5, NULL);
    ESP_LOGI(COMM_TAG, "SPI HANDLER INITIALIZED!");
}
