#include "main.h"

// ##############################################################################

void turn_on() { 
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level(BLINK_GPIO, 1);  
}


void turn_off() { 
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level(BLINK_GPIO, 0);  
}

static void configure_led(void)
{
    gpio_config_t led_conf = {
        .pin_bit_mask = BIT64((gpio_num_t)BLINK_GPIO),   
        .mode         = GPIO_MODE_OUTPUT,                
        .pull_up_en   = GPIO_PULLUP_DISABLE,             
        .pull_down_en = GPIO_PULLDOWN_DISABLE,           
        .intr_type    = GPIO_INTR_DISABLE,               
    };
    gpio_config(&led_conf);
}

// ##############################################################################

void my_post_setup_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(GPIO_HANDSHAKE, 1);
}
void my_post_trans_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(GPIO_HANDSHAKE, 0);
}
void handshake_init_slave(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_HANDSHAKE),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

// ##############################################################################

esp_err_t spi_slave_init(void) {
    handshake_init_slave();

    spi_bus_config_t buscfg = {
        .mosi_io_num   = GPIO_MOSI,
        .miso_io_num   = GPIO_MISO,
        .sclk_io_num   = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TRANSFER_SIZE,
    };
    spi_slave_interface_config_t slvcfg = {
        .spics_io_num   = GPIO_CS,
        .flags          = 0,
        .queue_size     = 3,
        .mode           = 0,
        .post_setup_cb  = my_post_setup_cb,
        .post_trans_cb  = my_post_trans_cb
    };
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS,   GPIO_PULLUP_ONLY);

    return spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
}


// ##############################################################################

void blade_spi_task(void *pvParameters) {
    uint8_t *rx_buf = reinterpret_cast<uint8_t*>(spi_bus_dma_memory_alloc(HSPI_HOST, TRANSFER_SIZE, 0));
    uint8_t *tx_buf = reinterpret_cast<uint8_t*>(spi_bus_dma_memory_alloc(HSPI_HOST, TRANSFER_SIZE, 0));
    blade_fsm_t fsm = BLADE_FSM_WAIT_START;
    uint8_t *png_buf = nullptr;
    uint8_t *rgba_ps = nullptr; 
    size_t png_len = 0;
    uint8_t *result_buf = nullptr;
    size_t  result_len = 0;
    int64_t t0 = 0;
    int64_t t1 = 0; 
    int64_t t2 = 0;
    int64_t t3 = 0; 
    int64_t t4 = 0;
    int64_t t5 = 0; 
    uint8_t* rgba = nullptr;
    uint8_t count = 0; 
    unsigned w,h;
    unsigned err; 

    turn_off();
    turn_on(); 
    const tflite::Model* model = tflite::GetModel(model_tflite);

    static tflite::MicroMutableOpResolver<6> resolver;  
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    resolver.AddConv2D();
    resolver.AddMean();
    resolver.AddDepthwiseConv2D();
    resolver.AddAdd();

    uint8_t* tensor_arena = (uint8_t*)heap_caps_malloc(TENSOR_ARENA_SZ, MALLOC_CAP_SPIRAM);

    tflite::MicroInterpreter interpreter(model, resolver, tensor_arena, TENSOR_ARENA_SZ, nullptr, nullptr);

    ESP_LOGI(COMM_TAG, "Max internal used: %u", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    if (interpreter.AllocateTensors() != kTfLiteOk) {
      ESP_LOGE(COMM_TAG, "AllocateTensors() failed");
      return;
    }

    spi_slave_transaction_t t_recv = {0};
    t_recv.length = TRANSFER_SIZE * 8;

    while (1) {
        memset(rx_buf, 0, TRANSFER_SIZE);

        turn_off();

        ESP_LOGI(COMM_TAG, "%d: Sending buffer; first bytes: 0x%02X 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X", 
                 count, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], 
                 tx_buf[TRANSFER_SIZE - 2], tx_buf[TRANSFER_SIZE - 1]);


        t_recv.tx_buffer = tx_buf;
        t_recv.rx_buffer = rx_buf;

        esp_err_t ret = spi_slave_transmit(HSPI_HOST, &t_recv, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(COMM_TAG, "Receive transaction failed: %s", esp_err_to_name(ret));
            continue;
        }

        ESP_LOGI(COMM_TAG, "%d: Received buffer; first bytes: 0x%02X 0x%02X 0x%02X 0x%02X ... 0x%02X 0x%02X", 
                 count, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], 
                 rx_buf[TRANSFER_SIZE - 2], rx_buf[TRANSFER_SIZE - 1]);

        // ##########################################################
        
        uint8_t pkt = rx_buf[0];

        switch (fsm) {
            case BLADE_FSM_WAIT_START:
            {
              ESP_LOGI(COMM_TAG, "IN BLADE_FSM_WAIT_START");
              if (pkt == PACKET_TYPE_START_DATA_TO_BLADE)
              {
                ESP_LOGI(COMM_TAG, "START_DATA_TO_BLADE");

                size_t free_int = heap_caps_get_free_size(MALLOC_CAP_8BIT);
                size_t free_psr = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
                ESP_LOGI(COMM_TAG, "BLADE_FSM_WAIT_START, starting receive:  internal free=%u, psram free=%u", (unsigned)free_int, (unsigned)free_psr);

                const uint32_t PSRAM_ALIGN = cache_hal_get_cache_line_size(0, CACHE_TYPE_DATA);
                ESP_LOGI(COMM_TAG, "align=%u", (unsigned)PSRAM_ALIGN);

                png_buf = (uint8_t *)heap_caps_aligned_alloc(32, BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (!png_buf)
                {
                  ESP_LOGE(COMM_TAG, "Failed to alloc PSRAM for PNG buffer");
                  continue;
                }
                else
                {
                  ESP_LOGW(COMM_TAG, "PNGBUF made!");
                }
                png_len = 0;
                t0 = esp_timer_get_time();
                fsm = BLADE_FSM_RECEIVE;
                memcpy(tx_buf, rx_buf, TRANSFER_SIZE);
              }
              else if (pkt == PACKET_TYPE_FUC)
              {
                fsm = BLADE_FSM_ERROR;
              }
              else if (pkt == PACKET_TYPE_GET_RESULT_FROM_BLADE) { 
                if (tx_buf[0] == PACKET_TYPE_GET_RESULT_FROM_BLADE) memset(tx_buf, 0, TRANSFER_SIZE);
                else { 
                  ESP_LOGE(COMM_TAG, "In BLADE_FSM_WAIT_START, tx_buf doesn't have the result, but switch asked for a result: 0x%02X", pkt);
                  fsm = BLADE_FSM_ERROR;
                }
              } else {
                ESP_LOGE(COMM_TAG, "Got wrong Packet while in BLADE_FSM_WAIT_START: 0x%02X", pkt);
                fsm = BLADE_FSM_ERROR;
              }
                break;
            }
            case BLADE_FSM_RECEIVE:
            {
              ESP_LOGI(COMM_TAG, "IN BLADE_FSM_RECEIVE");

                if (pkt == PACKET_TYPE_FUC) {
                    fsm = BLADE_FSM_ERROR;
                    break;
                }
                if ((pkt == PACKET_TYPE_DATA_TO_BLADE) || (pkt == PACKET_TYPE_END_DATA_TO_BLADE)) {
                    int len = (rx_buf[1]<<8)|rx_buf[2];

                    ESP_LOGI(COMM_TAG, "In RECEIVE: Got %d bytes", len);

                    if (png_len + len > BUF_SIZE) {
                      ESP_LOGE(COMM_TAG, "PNG buffer overflow: have %u, need %d", (unsigned)png_len, len);
                      fsm = BLADE_FSM_ERROR;
                      break;
                    }

                    if (!png_buf) { 
                      ESP_LOGE(COMM_TAG, "PNG Buf is not even made!!");
                      fsm = BLADE_FSM_ERROR;
                      break; 
                    } 
                    
                    memcpy(png_buf + png_len, rx_buf + 3, len);
                    png_len += len;
                    memcpy(tx_buf, rx_buf, TRANSFER_SIZE);

                    if (pkt == PACKET_TYPE_END_DATA_TO_BLADE) {
                      t1 = esp_timer_get_time();
                      ESP_LOGI(COMM_TAG, "SPI recv %d bytes in %lld ms", png_len, (t1 - t0) / 1000);
                      fsm = BLADE_FSM_PROCESS; // DIRECTLY GO into BLADE_FSM_PROCESS; 
                    }
                    else
                    {
                      break;
                    }
                } else if (pkt == PACKET_TYPE_FUC) {
                    fsm = BLADE_FSM_ERROR; 
                  break;
                }  else {
                    ESP_LOGE(COMM_TAG, "Got wrong Packet while in BLADE_FSM_RECEIVE: 0x%02X", pkt);
                  fsm = BLADE_FSM_ERROR;
                  break;
                }

            }
            case BLADE_FSM_PROCESS:
            {
              ESP_LOGI(COMM_TAG, "In BLADE_FSM_PROCESS; BEGINNING PROCESSING!");
              if (pkt == PACKET_TYPE_FUC)
              {
                fsm = BLADE_FSM_ERROR;
                break;
              }
                turn_on(); 

                t2 = esp_timer_get_time();

                ESP_LOGI(COMM_TAG, "HEADER: %d, %d, %d, %d, %d, %d, %d, %d", png_buf[0], png_buf[1], png_buf[2], png_buf[3], png_buf[4], png_buf[5], png_buf[6], png_buf[7]);
                err = lodepng_decode32(&rgba, &w, &h, png_buf, png_len);
                t3 = esp_timer_get_time();
                if (err || w!=IMG_SIZE || h!=IMG_SIZE) {
                    if (err) ESP_LOGE(COMM_TAG, "PNG decode error %u: %s", err, lodepng_error_text(err));
                    else ESP_LOGE(COMM_TAG, "PNG decode error, not err, but the h, w problem | %d, %d | %u: %s", h, w, err, lodepng_error_text(err));
                    fsm = BLADE_FSM_ERROR; 
                }
                ESP_LOGI(COMM_TAG, "PNG decode in %lld ms", (t3-t2)/1000);

                size_t img_bytes = w * h * 4;
                rgba_ps = (uint8_t*)heap_caps_malloc(img_bytes, MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
                if (!rgba_ps) {
                    ESP_LOGE(COMM_TAG, "PSRAM alloc failed for RGBA");
                    free(rgba);
                    break;
                }
                memcpy(rgba_ps, rgba, img_bytes);
                free(rgba);
                heap_caps_free(png_buf);

                TfLiteTensor* input = interpreter.input(0);
                float scale = input->params.scale;
                int zero_point = input->params.zero_point;

                t4 = esp_timer_get_time();

                int8_t* in_data = input->data.int8;
                for (unsigned y=0; y<h; ++y) {
                    for (unsigned x=0; x<w; ++x) {
                        unsigned idx_rgba = (y*w + x)*4;
                        float r = rgba_ps[idx_rgba+0]/255.f;
                        float g = rgba_ps[idx_rgba+1]/255.f;
                        float b = rgba_ps[idx_rgba+2]/255.f;
                        unsigned idx_rgb = (y*w + x)*3;
                        in_data[idx_rgb+0] = int8_t(r/scale) + zero_point;
                        in_data[idx_rgb+1] = int8_t(g/scale) + zero_point;
                        in_data[idx_rgb+2] = int8_t(b/scale) + zero_point;
                    }
                }
                if (interpreter.Invoke() != kTfLiteOk) {
                    ESP_LOGE(COMM_TAG, "Invoke failed");
                    fsm = BLADE_FSM_ERROR;
                    break;
                }

                t5 = esp_timer_get_time();
                long long int total_time = (t5 - t4) / 1000; 
                ESP_LOGI(COMM_TAG, "Inference in %lld ms", total_time);

                TfLiteTensor* output = interpreter.output(0);
                int8_t* out_data = output->data.int8;
                int best = 0;
                for (int i=1; i<NUM_CLASSES; ++i) {
                    if (out_data[i] > out_data[best]) best = i;
                }

                const char* res = model_labels[best];
                ESP_LOGI(COMM_TAG, "Result: %s", res);

                memset(tx_buf, 0, TRANSFER_SIZE);
                tx_buf[0] = PACKET_TYPE_GET_RESULT_FROM_BLADE;

                char* p = (char*)(tx_buf + 3);
                int pos = 0;

                // start JSON
                pos += snprintf(p + pos, TRANSFER_SIZE - 3 - pos,
                    "{"
                      "\"pred\":\"%s\","
                      "\"time\":%lld,"
                      "\"scores\":{",
                    model_labels[best],
                    total_time
                );

                for (int i = 0; i < NUM_CLASSES; i++) {
                  // float conf = (out_data[i] - zero_point) * scale;
                  pos += snprintf(p + pos, TRANSFER_SIZE - 3 - pos,
                    "\"%s\":%.3f%s",
                    model_labels[i],
                    (float) out_data[i],
                    (i < NUM_CLASSES - 1) ? "," : ""
                  );
                }

                pos += snprintf(p + pos, TRANSFER_SIZE - 3 - pos, "}}\n");

                tx_buf[1] = (pos >> 8) & 0xFF;
                tx_buf[2] = (pos     ) & 0xFF;

                fsm = BLADE_FSM_WAIT_START;
                turn_off();
                break;
            }
            default: {
                fsm = BLADE_FSM_ERROR;
                memset(tx_buf, 0, TRANSFER_SIZE);
                tx_buf[0] = PACKET_TYPE_FUC;
                ESP_LOGE(COMM_TAG, "FSM ERROR, don't know what command is sent");
                break;
            }
        }

        if (fsm == BLADE_FSM_ERROR) {
            ESP_LOGE(COMM_TAG, "BLADE FSM ERROR!");
            if (png_buf) { 
              heap_caps_free(png_buf);
            }
            if (rgba) { 
              free(rgba);
            } 
            png_buf = nullptr;
            rgba = nullptr; 
            fsm = BLADE_FSM_WAIT_START;
            tx_buf[0] = PACKET_TYPE_FUC;
            t0 = 0;
            t1 = 0;
            t2 = 0;
            t3 = 0;
            t4 = 0;
            t5 = 0;
            png_len = 0;
            turn_off();
        } 
        count = (count == 1) ? 0 : 1;
    }
}

// ##############################################################################

extern "C" void app_main() {
    configure_led(); 

    esp_err_t ret = spi_slave_init(); 
    if (ret != ESP_OK)
    {
        ESP_LOGE(COMM_TAG, "SPI SLAVE INITIALIZATION FAILED!: %s", esp_err_to_name(ret));
        return;
    }
    
    xTaskCreate(blade_spi_task, "blade_spi", 8*1024, NULL, 5, NULL);
    ESP_LOGI(COMM_TAG, "BLADE HANDLER INITIALIZED!");

    for (int i=1; i<NUM_CLASSES; ++i) {
      const char* res = model_labels[i];
      ESP_LOGI(COMM_TAG, "Class Name: %s", res);
    }


}