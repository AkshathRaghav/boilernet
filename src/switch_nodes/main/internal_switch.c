#include "internal_switch.h"

// ##############################################################################

void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(NETWORK_HANDSHAKE, 1);
}

void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(NETWORK_HANDSHAKE, 0);
}

void handshake_init_slave(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << NETWORK_HANDSHAKE),
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
        .mosi_io_num = NETWORK_MOSI,
        .miso_io_num = NETWORK_MISO,
        .sclk_io_num = NETWORK_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TRANSFER_SIZE,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = NETWORK_CS,
        .queue_size = 2,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb
    };

    gpio_set_pull_mode(NETWORK_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(NETWORK_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(NETWORK_CS,   GPIO_PULLUP_ONLY);

    ret = spi_slave_initialize(NETWORK_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    return ret; 
}

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
    host.slot = SD_HOST; 

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_NUM_MOSI,
        .miso_io_num = SD_NUM_MISO,
        .sclk_io_num = SD_NUM_CLK,
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
    slot_config.gpio_cs = SD_NUM_CS;
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

esp_err_t compute_spi_master_init(void)
{

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz  = 5000000,  // 1 MHz—tune to your blades’ needs
        .mode            = 0,
        .spics_io_num    = -1,               // will set per-device below
        .queue_size      = 1,
    };

    for (int i = 0; i < NUM_COMPUTE_BLADES; i++) {
        devcfg.spics_io_num = compute_cs_pins[i];
        esp_err_t ret = spi_bus_add_device(SD_HOST, &devcfg, &compute_handles[i]);
        if (ret != ESP_OK) {
            ESP_LOGW(COMM_TAG,
                     "Blade %d not registered (CS pin %d): %s",
                     i, compute_cs_pins[i], esp_err_to_name(ret));
        } else {
            ESP_LOGI(COMM_TAG,
                     "Blade %d registered on CS pin %d",
                     i, compute_cs_pins[i]);
        }
    }
    return ESP_OK;
}

void setup_compute_handshake_gpios(void)
{
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = 0,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    for (int i = 0; i < NUM_COMPUTE_BLADES; i++) {
        io_conf.pin_bit_mask |= 1ULL << compute_handshake_pins[i];
    }
    gpio_config(&io_conf);
    ESP_LOGI(COMM_TAG, "Configured %d compute handshake GPIOs", NUM_COMPUTE_BLADES);
}

int find_available_compute_blades(void)
{
    int count = 0;
    for (int i = 0; i < NUM_COMPUTE_BLADES; i++) {
        int lvl = gpio_get_level(compute_handshake_pins[i]);
        if (lvl) {
            count++;
        } else {
            ESP_LOGW(COMM_TAG,
                     "Blade %d not responding (HS pin %d low)",
                     i, compute_handshake_pins[i]);
        }
    }
    ESP_LOGI(COMM_TAG,
             "%d/%d compute blades available",
             count, NUM_COMPUTE_BLADES);
    return count;
}

bool make_result_filename(const char *in, char *out, size_t out_sz) {
    const char *dot = strrchr(in, '.');
    size_t base_len = dot ? (size_t)(dot - in) : strlen(in);

    const char *suffix = "_results.txt";
    size_t suffix_len = strlen(suffix);

    if (base_len + suffix_len + 1 > out_sz) {
        return false;
    }

    for (size_t i = 0; i < base_len; i++) {
        out[i] = (char) tolower((unsigned char)in[i]);
    }
    memcpy(out + base_len, suffix, suffix_len + 1); 

    return true;
}

int write_failed(FILE *result_file, int blade) {
    char prefix[32];
    int prefix_len = snprintf(prefix, sizeof(prefix), "%d: FAILED\n", blade);
    if (sdcard_write(result_file, prefix, (size_t)prefix_len) != (size_t)prefix_len) {
        ESP_LOGE(COMM_TAG, "Write_failed() error writing prefix for blade %d", blade);
        return -1;
    }
    return 0;
}


void compute_fsm_process(FILE *sd_file,      
    const char             *result_filename,    
    int                      num_blades,
    spi_device_handle_t    *blade_handles,
    const int             *blade_hs_pins,
    uint8_t                *tx_buf,
    uint8_t                *rx_buf,
    TickType_t            *compute_start_tick,
    compute_fsm_state_t   *compute_fsm_state,
    fsm_state_t           *outer_fsm_state
) {
    const int   max_payload    = TRANSFER_SIZE - 10;  
    FILE       *result_file    = NULL;
    int         file_bytes;
    char prefix[32];

    spi_transaction_t t = {
            .length    = TRANSFER_SIZE * 8,
            .tx_buffer = tx_buf,
            .rx_buffer = rx_buf
        };

    bool *passed = (bool*) calloc(num_blades, sizeof(bool));
    if (!passed) {
        ESP_LOGE(COMM_TAG, "Failed to alloc passed[]");
        *outer_fsm_state = FSM_ERROR;
        return;
    }

    if (fseek(sd_file, 0, SEEK_SET) != 0) {
        ESP_LOGE(COMM_TAG, "Failed to rewind SD file");
        *outer_fsm_state = FSM_ERROR;
        goto cleanup;
    }

    tx_buf[0] = PACKET_TYPE_START_DATA_TO_BLADE;
    for (int blade = 0; blade < num_blades; blade++) {
        while (!gpio_get_level(blade_hs_pins[blade])) {
            CHECK_TIMEOUT(blade);
        }

        ESP_LOGI(COMM_TAG, "START TOLD TO BLADE %d: ", blade); 

        if (spi_device_transmit(blade_handles[blade], &t) != ESP_OK) {
            ESP_LOGE(COMM_TAG, "SPI receive error from blade %d", blade);
            passed[blade] = false; 
        } else {
            passed[blade] = true; 
        }
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }

    int count = 0; 

    while (true) {
        file_bytes = sdcard_read(sd_file, &tx_buf[3], max_payload);
        if (file_bytes < 0) {
            ESP_LOGE(COMM_TAG, "Error reading SD file");
            *outer_fsm_state = FSM_ERROR;
            goto cleanup;
        }
        if (file_bytes == 0) {
            break;
        }
        count++; 

        if (file_bytes < max_payload) { 
           tx_buf[0] = PACKET_TYPE_END_DATA_TO_BLADE;
        } else { 
           tx_buf[0] = PACKET_TYPE_DATA_TO_BLADE;
        }

        tx_buf[1] = (file_bytes >> 8) & 0xFF;
        tx_buf[2] = (file_bytes) & 0xFF; 
        
        ESP_LOGI(COMM_TAG, "Chunk %d (%d bytes)", count, file_bytes);
        
        for (int blade = 0; blade < num_blades; blade++) {
            if (!passed[blade]) continue;
            while (!gpio_get_level(blade_hs_pins[blade])) {
                CHECK_TIMEOUT(blade);
            }

            if (spi_device_transmit(blade_handles[blade], &t) != ESP_OK) {
                ESP_LOGE(COMM_TAG, "SPI transmit error to blade %d", blade);
                passed[blade] = false; 
                goto cleanup;
            }
            ESP_LOGI(COMM_TAG, "      Sent to BLADE %d", blade); 
        }
    }

    vTaskDelay(pdMS_TO_TICKS(5000)); 
    *compute_start_tick = xTaskGetTickCount();

    result_file = sdcard_open(result_filename, "w");
    if (!result_file) {
        ESP_LOGE(COMM_TAG, "Cannot open result file %s", result_filename);
        *outer_fsm_state = FSM_ERROR;
        goto cleanup;
        return;
    }

    for (int blade = 0; blade < num_blades; blade++) {

        RECORD_FAILED(blade);
        while (!gpio_get_level(blade_hs_pins[blade])) {
            CHECK_TIMEOUT(blade);
        }

        tx_buf[0] = PACKET_TYPE_GET_RESULT_FROM_BLADE;

        if (spi_device_transmit(blade_handles[blade], &t) != ESP_OK) {
            ESP_LOGE(COMM_TAG, "SPI receive error from blade %d", blade);
            passed[blade] = false;
            RECORD_FAILED(blade);
            continue;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1)); 

        ESP_LOGI(COMM_TAG, "%d: Received buffer; first bytes: 0x%02X 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X", 
                 count, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], 
                 rx_buf[TRANSFER_SIZE - 2], rx_buf[TRANSFER_SIZE - 1]);

        int data_len = (rx_buf[1] << 8) | rx_buf[2];
        if (data_len <= 0 || data_len > max_payload) {
            ESP_LOGE(COMM_TAG, "Invalid data length %d from blade %d", data_len, blade);
            passed[blade] = false;
            RECORD_FAILED(blade);
            continue;
        }
        
        int prefix_len = snprintf(prefix, sizeof(prefix), "%d: ", blade);
        if (sdcard_write(result_file, prefix, (size_t)prefix_len) != (size_t)prefix_len) {
            ESP_LOGE(COMM_TAG, "Write error to prefix the entry from blade %d", blade);
            passed[blade] = false;
            RECORD_FAILED(blade);
            continue;
        }
        if (sdcard_write(result_file, &rx_buf[3], data_len) != (size_t)data_len) {
            ESP_LOGE(COMM_TAG, "Write error to result file from blade %d", blade);
            passed[blade] = false;
            RECORD_FAILED(blade);
            continue;
        }
        ESP_LOGI(COMM_TAG, "Blade %d -> %d bytes", blade, data_len);
    }

    sdcard_close(result_file);
    *compute_fsm_state = FSM_IDLE;
    ESP_LOGI(COMM_TAG, "All blade results written to %s", result_filename);

cleanup:
    if (sd_file) sdcard_close(sd_file);
    if (result_file) sdcard_close(result_file);
    free(passed);
}


// ##############################################################################

static void spi_handler_task(void *pvParameters) { 
    uint8_t *rx_buf_parent = spi_bus_dma_memory_alloc(NETWORK_HOST, TRANSFER_SIZE, 0);
    uint8_t *rx_buf = spi_bus_dma_memory_alloc(NETWORK_HOST, TRANSFER_SIZE, 0);
    uint8_t *tx_buf = spi_bus_dma_memory_alloc(NETWORK_HOST, TRANSFER_SIZE, 0);

    if (!rx_buf || !tx_buf) {
        ESP_LOGE(COMM_TAG, "Failed to allocate DMA buffers");
        return;
    }
    memset(tx_buf, 0, TRANSFER_SIZE);
    memset(rx_buf_parent, 0, TRANSFER_SIZE);
    
    fsm_state_t fsm_state = FSM_WAIT_START_WRITE;
    compute_fsm_state_t compute_fsm_state = FSM_IDLE;

    unsigned int computed_checksum = 0;
    FILE *sd_file = NULL;
    char filename[80] = {0};
    char result_filename[100] = {0};

    int count = 0; 
    spi_slave_transaction_t t_recv = {0};
    t_recv.length = TRANSFER_SIZE * 8;

    int file_opened = 0;
    int num_computes = -1;

    uint32_t compute_start_time = 0; 

    while (1) {
        memset(rx_buf, 0, TRANSFER_SIZE);

        ESP_LOGI(COMM_TAG, "%d: Sent buffer; first bytes: 0x%02X 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X", 
                 count, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], 
                 tx_buf[TRANSFER_SIZE - 2], tx_buf[TRANSFER_SIZE - 1]);
                 
        t_recv.tx_buffer = tx_buf;
        t_recv.rx_buffer = rx_buf;

        esp_err_t ret = spi_slave_transmit(NETWORK_HOST, &t_recv, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(COMM_TAG, "Receive transaction failed: %s", esp_err_to_name(ret));
            continue;
        }


        ESP_LOGI(COMM_TAG, "%d: Received buffer; first bytes: 0x%02X 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X", 
                 count, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], 
                 rx_buf[TRANSFER_SIZE - 2], rx_buf[TRANSFER_SIZE - 1]);

        // ##########################################################
        
        if ((fsm_state == FSM_WAIT_COMPUTE_PROCESS) && (file_opened)) {

            if (!make_result_filename(filename, result_filename, sizeof(result_filename))) {
                ESP_LOGE(COMM_TAG, "Filename too long for buffer");
                fsm_state = FSM_ERROR; 
            }

            uint8_t *rx_buf2  = spi_bus_dma_memory_alloc(SD_HOST, TRANSFER_SIZE, 0);
            uint8_t *tx_buf2 = spi_bus_dma_memory_alloc(SD_HOST, TRANSFER_SIZE, 0);
            compute_fsm_process(
                sd_file,
                result_filename,
                num_computes,
                compute_handles,
                compute_handshake_pins,
                tx_buf2,
                rx_buf2,
                &compute_start_time,
                &compute_fsm_state,
                &fsm_state);
            ESP_LOGW(COMM_TAG, "Compute FSM Completed!");

            file_opened = 0; 
        }

        if (((tx_buf[1] == 0xFF) && (fsm_state == FSM_WAIT_END_READ)) || (fsm_state == FSM_WAIT_END_COMPUTE)) { 
            fsm_state = FSM_WAIT_START_WRITE;
            memset(tx_buf, 0, TRANSFER_SIZE);
            num_computes = -1; 
        }

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
                memcpy(filename, &rx_buf[2], 80);
                filename[79] = '\0';  
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

            // ---------- Read FSM handling ------------
            case PACKET_TYPE_START_READ:
            {
                if (fsm_state != FSM_WAIT_START_WRITE) { // FSM_WAIT_START_WRITE is being used as an IDLE State here. Not the best logic lol.
                    ESP_LOGE(COMM_TAG, "Unexpected START_READ packet received in state %d", fsm_state);
                    fsm_state = FSM_ERROR;
                    break;
                }
                memcpy(filename, &rx_buf[2], 80);
                filename[79] = '\0';
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
                    int file_bytes = sdcard_read(sd_file, &tx_buf[3], TRANSFER_SIZE - 3);
                    if (file_bytes < 0) {
                        ESP_LOGE(COMM_TAG, "Error reading file for DATA_READ");
                        fsm_state = FSM_ERROR;
                        break;
                    }

                    tx_buf[0] = PACKET_TYPE_DATA_READ;
                    tx_buf[1] = (file_bytes >> 8) & 0xFF;
                    tx_buf[2] = file_bytes & 0xFF;
                    
                    for (int i = 0; i < file_bytes; i++) {
                        computed_checksum += tx_buf[3 + i];
                    }
                    
                    ESP_LOGI(COMM_TAG, "DATA_READ packet prepared: %d bytes read", file_bytes);
                    
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

            // ---------- Compute FSM handling ------------
            case PACKET_TYPE_START_COMPUTE:
            {
                if (fsm_state == FSM_WAIT_COMPUTE_PROCESS) { // dummy resend from network, acceptable mistake 
                    continue; 
                }
                if (fsm_state != FSM_WAIT_START_WRITE) {
                    ESP_LOGE(COMM_TAG, "Unexpected START_COMPUTE packet in state %d", fsm_state);
                    fsm_state = FSM_ERROR;
                    break;
                }

                num_computes = find_available_compute_blades();
                if (num_computes <= 0) {
                    ESP_LOGE(COMM_TAG, "No compute blades available!");
                    fsm_state = FSM_ERROR;
                    break;
                }

                memcpy(filename, &rx_buf[2], 80);
                filename[79] = '\0';
                sd_file = sdcard_open(filename, "r");
                if (sd_file == NULL) {
                    ESP_LOGE(COMM_TAG, "File not found on SD: %s", filename);
                    fsm_state = FSM_ERROR;
                    break;
                }
                file_opened = 1; 

                ESP_LOGI(COMM_TAG, "Received START_COMPUTE; sending ACK.");
                tx_buf[0] = PACKET_TYPE_START_COMPUTE;  
                tx_buf[1] = 0x00;  
                compute_start_time = xTaskGetTickCount();

                compute_fsm_state = FSM_SEND_DATA_TO_BLADE;
                fsm_state = FSM_WAIT_COMPUTE_PROCESS;
                break;
            }
            case PACKET_TYPE_POLL_COMPUTE:
            {
                if (fsm_state == FSM_WAIT_START_WRITE) continue; 

                if (fsm_state != FSM_WAIT_COMPUTE_PROCESS) {
                    ESP_LOGE(COMM_TAG, "Unexpected POLL_COMPUTE packet in state %d", fsm_state);
                    fsm_state = FSM_ERROR;
                    break;
                }

                if (compute_fsm_state == FSM_IDLE) {
                    tx_buf[0] = PACKET_TYPE_END_COMPUTE;
                    tx_buf[1] = 0x00;
                    memcpy(tx_buf + 2, result_filename, strlen(result_filename));
                    memset(result_filename, 0, 80);
                    file_opened = 0; 
                    ESP_LOGI(COMM_TAG, "Compute processing complete; sending END_COMPUTE, but will have to resend.");
                    fsm_state = FSM_WAIT_END_COMPUTE;
                }
                else
                {
                    tx_buf[0] = PACKET_TYPE_WAIT_COMPUTE;
                    ESP_LOGI(COMM_TAG, "Compute is still processing; sending WAIT_COMPUTE."); // Since the compute talking is happening in a blocking manner, we won't reach here i think
                }
                break;
            }

            // ---------- Delete FSM handling ------------
            case PACKET_TYPE_DELETE:
            {
                if (fsm_state != FSM_WAIT_START_WRITE) {
                    ESP_LOGE(COMM_TAG, "Unexpected DELETE packet in state %d", fsm_state);
                    fsm_state = FSM_ERROR;
                    break;
                }

                memcpy(filename, &rx_buf[2], 80);
                filename[79] = '\0';
                sdcard_delete(filename); 

                ESP_LOGI(COMM_TAG, "Received DELETE");
                tx_buf[0] = PACKET_TYPE_DELETE_RET;  
                tx_buf[1] = 0xAA;  

                fsm_state = FSM_WAIT_START_WRITE;
                break;
            }

            // --------------------------------------------------
            case PACKET_TYPE_FUC:
            {
                ESP_LOGE(COMM_TAG, "FUC packet received. Transaction failed.");
                if (sd_file != NULL) {
                    sdcard_close(sd_file);
                    sdcard_delete(filename);
                    sd_file = NULL;
                }
                fsm_state = FSM_WAIT_START_WRITE;
                computed_checksum = 0;
                memset(filename, 0, sizeof(filename));
                memset(tx_buf, 0, TRANSFER_SIZE);
                break;
            }
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
        
        if ((fsm_state != FSM_WAIT_COMPUTE_PROCESS) && (fsm_state != FSM_WAIT_END_COMPUTE) &&
            (fsm_state != FSM_WAIT_START_READ) && (fsm_state != FSM_WAIT_DATA_READ) && (fsm_state != FSM_WAIT_END_READ) && 
            !((fsm_state == FSM_COMPLETE) && (file_opened)) && ((fsm_state == FSM_COMPLETE) && (!file_opened))
        ) memcpy(tx_buf, rx_buf, TRANSFER_SIZE);

        if (fsm_state == FSM_WAIT_DATA_WRITE) { 
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
        } else if (fsm_state == FSM_ERROR)
        {
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
    
    if (compute_spi_master_init() != ESP_OK) {
        ESP_LOGI(COMM_TAG, "Failed to init compute SPI master");
        return;
    }
    setup_compute_handshake_gpios();
    ESP_LOGI(COMM_TAG, "COMPUTE GPIOs INITIALIZED!");

}
