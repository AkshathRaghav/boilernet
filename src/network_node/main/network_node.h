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
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"

// ################################################################

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

extern void ethernet_setup(void); 

// ################################################################

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

#define SD_NUM_MOSI     13
#define SD_NUM_MISO     12
#define SD_NUM_CLK      14
#define SD_NUM_CS       4

#define MOUNT_POINT "/sdcard"

#define SD_HOST   HSPI_HOST   

// ################################################################

#define SWITCH_NUM_MOSI  13
#define SWITCH_NUM_MISO  12
#define SWITCH_NUM_CLK   14

#define SWITCH_NUM_CS_0   15
#define SWITCH_NUM_CS_1    33

#define SWITCH_HANDSHAKE_0 36
#define SWITCH_HANDSHAKE_1 39

static const int switch_handshake_pins[] = {
    SWITCH_HANDSHAKE_0,
    SWITCH_HANDSHAKE_1
};

static const int switch_cs_pins[] = {
    SWITCH_NUM_CS_0,
    SWITCH_NUM_CS_1
};


#define NUM_SWITCH_NODES  (sizeof(switch_cs_pins)/sizeof(switch_cs_pins[0]))

static spi_device_handle_t switch_handles[NUM_SWITCH_NODES];

#define TRANSFER_SIZE 2048
#define MAX_RETRY 5

#define SWITCH_HOST   HSPI_HOST   

// ################################################################

