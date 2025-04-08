#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

// ===== SPI/SD CARD CONFIGURATION (from SPI example) =====
#define PIN_NUM_MISO  12
#define PIN_NUM_MOSI  13
#define PIN_NUM_CLK   14
#define PIN_NUM_CS    15

// Mount point used in the SPI example (do not mix with the UART example!)
#define MOUNT_POINT   "/switch_sd_root"

// ===== UART CONFIGURATION =====
#define UART_NUM        UART_NUM_0    // Using the USB UART
#define BUF_SIZE        1024
#define FILEPATH_BUFFER_SIZE 512

// ===== PATCH TRANSFER COMMANDS =====
#define START_PATCH_CMD  "_START_PATCH_"
#define END_PATCH_CMD    "_END_PATCH_"

// Logging tag
static const char *TAG = "PatchSD";

/*------------------------------------------------------------
  Function: log_sd_files
  Description: Lists all files on the SD card and logs each file's size.
-------------------------------------------------------------*/
static void log_sd_files(void)
{
    DIR *dir;
    struct dirent *entry;
    char filepath[FILEPATH_BUFFER_SIZE];
    struct stat st;

    dir = opendir(MOUNT_POINT);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", MOUNT_POINT);
        return;
    }
    ESP_LOGI(TAG, "Listing files on SD card:");
    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;
        snprintf(filepath, sizeof(filepath), "%s/%s", MOUNT_POINT, entry->d_name);
        if (stat(filepath, &st) == 0) {
            ESP_LOGI(TAG, "File: %s, Size: %d bytes", filepath, (int)st.st_size);
        } else {
            ESP_LOGE(TAG, "Failed to get info for file: %s", filepath);
        }
    }
    closedir(dir);
}

/*------------------------------------------------------------
  Function: clear_sd_files
  Description: Deletes all files on the SD card.
-------------------------------------------------------------*/
static void clear_sd_files(void)
{
    DIR *dir;
    struct dirent *entry;
    char filepath[FILEPATH_BUFFER_SIZE];

    dir = opendir(MOUNT_POINT);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", MOUNT_POINT);
        return;
    }
    ESP_LOGI(TAG, "Deleting all files on SD card:");
    while ((entry = readdir(dir)) != NULL) {
        // Skip the current and parent directory entries
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;
        snprintf(filepath, sizeof(filepath), "%s/%s", MOUNT_POINT, entry->d_name);
        if (remove(filepath) == 0) {
            ESP_LOGI(TAG, "Deleted file: %s", filepath);
        } else {
            ESP_LOGE(TAG, "Failed to delete file: %s", filepath);
        }
    }
    closedir(dir);
}

/*------------------------------------------------------------
  Initialize the SD card using SPI (from the SPI example)
-------------------------------------------------------------*/
static void init_sd_card(void)
{
    esp_err_t ret;
    // Mount configuration
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,  // Set to true to auto-format if mounting fails
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    ESP_LOGI(TAG, "Initializing SD card");

    // Initialize SPI host (from SPI example)
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount filesystem: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted at %s", MOUNT_POINT);
}

/*------------------------------------------------------------
  Initialize the UART (from UART example, SPI parts omitted)
-------------------------------------------------------------*/
static void init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
}

/*------------------------------------------------------------
  Main application: receive image patches over UART and store 
  each patch as a separate file (patch0.bin, patch1.bin, etc.)
-------------------------------------------------------------*/
void app_main(void)
{
    init_uart();
    init_sd_card();

    // Clear any existing files on the SD card before receiving new data.
    clear_sd_files();

    uint8_t data[BUF_SIZE];
    int receiving = 0;  // Flag: set to 1 when currently receiving a patch
    FILE *f = NULL;
    int patch_counter = 0;
    int patch_size = 0;  // Total bytes received for the current patch
    char filename[64];

    ESP_LOGI(TAG, "Waiting for image patch data over UART...");

    while (1) {
        // Read data from UART (with a timeout)
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            // For command packets (start/end), we assume the data is ASCII.
            // (Binary patch data may contain 0 bytes so we only check for commands when not in a burst.)
            data[len] = '\0';

            if (!receiving && strcmp((char *)data, START_PATCH_CMD) == 0) {
                // Received a start packet: open a new patch file and reset patch size counter
                receiving = 1;
                patch_size = 0;
                snprintf(filename, sizeof(filename), MOUNT_POINT"/patch%d.bin", patch_counter);
                f = fopen(filename, "wb");
                if (f == NULL) {
                    ESP_LOGE(TAG, "Failed to open file %s for writing", filename);
                    uart_write_bytes(UART_NUM, "ERR_OPEN", strlen("ERR_OPEN"));
                    receiving = 0;
                } else {
                    ESP_LOGI(TAG, "Started receiving patch %d", patch_counter);
                    uart_write_bytes(UART_NUM, "ACK_START", strlen("ACK_START"));
                }
            }
            else if (receiving && strcmp((char *)data, END_PATCH_CMD) == 0) {
                // Received an end packet: close the current patch file and log total length
                if (f) {
                    fclose(f);
                    f = NULL;
                    ESP_LOGI(TAG, "Finished receiving patch %d, total length: %d bytes", patch_counter, patch_size);
                    uart_write_bytes(UART_NUM, "ACK_END", strlen("ACK_END"));
                }
                receiving = 0;
                patch_counter++;  // Increment counter for next patch

                // Log all files on the SD card and their sizes
                log_sd_files();
            }
            else if (receiving) {
                // We are currently receiving a patch: write the data chunk to file.
                if (f) {
                    fwrite(data, 1, len, f);
                    fflush(f);
                    patch_size += len;  // Update the patch size
                    // Optionally, acknowledge each data chunk:
                    uart_write_bytes(UART_NUM, "ACK_DATA", strlen("ACK_DATA"));
                } else {
                    uart_write_bytes(UART_NUM, "ERR_NOFILE", strlen("ERR_NOFILE"));
                }
            }
            else {
                // Not in patch receiving mode and received an unexpected packet
                uart_write_bytes(UART_NUM, "NACK", strlen("NACK"));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
