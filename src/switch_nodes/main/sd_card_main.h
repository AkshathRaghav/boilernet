#ifndef SD_CARD_MAIN_H
#define SD_CARD_MAIN_H

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sd_test_io.h"

#define EXAMPLE_MAX_CHAR_SIZE    64
#define CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
#define PIN_NUM_MISO  6
#define PIN_NUM_MOSI  4
#define PIN_NUM_CLK   5
#define PIN_NUM_CS    1
#define CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED

static esp_err_t s_example_write_file(const char *path, char *data);
static esp_err_t s_example_read_file(const char *path);
void main(void);

#endif SD_CARD_MAIN_H