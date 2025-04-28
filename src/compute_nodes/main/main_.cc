// main.cc

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "lodepng.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"

// TFLM includes
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_profiler.h"

// Model header (const array lives in Flash/RODATA)
#include "dogs_model_data.h"

static const char* TAG = "CLASSIFY";
#define BUF_SIZE     (300 * 1024)
#define IMG_SIZE        224
#define NUM_CLASSES     5
#define TENSOR_ARENA_SZ (1024 * 1024)  // lives in INTERNAL SRAM by default
#define UART_NUM        UART_NUM_0    // Using the USB UART

#define RCV_HOST VSPI_HOST
#define TRANSFER_SIZE 2048
#define PACKET_TYPE_GET_DATA_FROM_BLADE 0xD3
#define BLINK_GPIO GPIO_NUM_32

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


// Labels
const char* dog_labels[NUM_CLASSES] = {
  "German Shepherd",
  "Labrador Retriever",
  "Pug",
  "Chihuahua",
  "Poodle"
};

void init_uart() {
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

void turn_on() { 
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level(BLINK_GPIO, 1);  
}


void turn_off() { 
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level(BLINK_GPIO, 0);  
}


extern "C" void app_main() {
    init_uart();
    configure_led(); 

    turn_off();
    turn_on(); 
    static tflite::MicroErrorReporter micro_error_reporter;
    tflite::ErrorReporter* err_reporter = &micro_error_reporter;
    const tflite::Model* model = tflite::GetModel(dogs_model_tflite);

    static tflite::MicroMutableOpResolver<6> resolver;  
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    resolver.AddConv2D();
    resolver.AddMean();
    resolver.AddDepthwiseConv2D();
    resolver.AddAdd();

    turn_off();
    turn_on(); 
    static tflite::MicroProfiler profiler;

    uint8_t* tensor_arena = (uint8_t*)heap_caps_malloc(TENSOR_ARENA_SZ, MALLOC_CAP_SPIRAM);
    tflite::MicroAllocator* allocator = tflite::MicroAllocator::Create(tensor_arena, TENSOR_ARENA_SZ);

    tflite::MicroInterpreter interpreter(
        model,
        resolver,
        tensor_arena, 
        TENSOR_ARENA_SZ,
        nullptr,
        &profiler);

    ESP_LOGI(TAG, "Max internal used: %u", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    if (interpreter.AllocateTensors() != kTfLiteOk) {
      ESP_LOGE(TAG, "AllocateTensors() failed");
      return;
    }

    uint8_t* response = reinterpret_cast<uint8_t*>(spi_bus_dma_memory_alloc(RCV_HOST, TRANSFER_SIZE, 0));
    if (!response) {
      ESP_LOGE(TAG, "SPI DMA alloc failed");
      return;
    }

    while (true)
    {
      turn_off();
      ESP_LOGI(TAG, "Waiting for 'START\\n'...");

      uint8_t header[6];
      while (true)
      {
        uart_read_bytes(UART_NUM, &header[0], 1, portMAX_DELAY);
        if (header[0] == 'S')
        {
          // Now read the next 5 bytes: "TART\n"
          int r = uart_read_bytes(UART_NUM, &header[1], 5, portMAX_DELAY);
          if (r == 5 && memcmp(header, "START\n", 6) == 0)
          {
            break;
          }
          memmove(header, header + 1, 5);
        }
      }
      turn_on();
      ESP_LOGI(TAG, "Received START");

      // Allocate PNG buffer in PSRAM
      uint8_t *png_buf = (uint8_t *)heap_caps_malloc(BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (!png_buf)
      {
        ESP_LOGE(TAG, "Failed to alloc PSRAM for PNG buffer");
        continue;
      }

      int total = 0;
      int64_t t0 = esp_timer_get_time();
      int retry = 0;

      while (true)
      {
        uint8_t b;
        int len = uart_read_bytes(UART_NUM, &b, 1, pdMS_TO_TICKS(200));
        if (len > 0)
        {
          retry = 0; // reset retry counter
          // END marker?
          if (total >= 4 &&
              png_buf[total - 4] == 'E' &&
              png_buf[total - 3] == 'N' &&
              png_buf[total - 2] == 'D' &&
              png_buf[total - 1] == '\n')
          {
            total -= 4; // strip END\n
            break;
          }
          png_buf[total++] = b;
        } else {
        // no bytes this cycle
        if (total > 0) {
          // if we've collected _some_ data and then get no more,
          // assume the sender is done (EOF)
          break;
        }
        if (++retry > 50) {
          ESP_LOGE(TAG, "UART timeout waiting for PNG data");
          heap_caps_free(png_buf);
          continue;
        }
      }
    }


    int64_t t1 = esp_timer_get_time();
    ESP_LOGI(TAG, "UART recv %d bytes in %lld ms", total, (t1 - t0) / 1000);

    turn_off();
    turn_on(); 
    uint8_t* rgba = nullptr;
    unsigned w,h;
    int64_t t2 = esp_timer_get_time();
    unsigned err = lodepng_decode32(&rgba, &w, &h, png_buf, total);
    int64_t t3 = esp_timer_get_time();
    if (err || w!=IMG_SIZE || h!=IMG_SIZE) {
      if (err) ESP_LOGE(TAG, "PNG decode error %u: %s", err, lodepng_error_text(err));
      else ESP_LOGE(TAG, "PNG decode error, not err, but the h, w problem | %d, %d | %u: %s", h, w, err, lodepng_error_text(err));
      free(rgba);
      heap_caps_free(png_buf);
      continue;
    }
    ESP_LOGI(TAG, "PNG decode in %lld ms", (t3-t2)/1000);

    turn_off();
    turn_on(); 
    // Move RGBA to PSRAM
    size_t img_bytes = w * h * 4;
    uint8_t* rgba_ps = (uint8_t*)heap_caps_malloc(img_bytes, MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if (!rgba_ps) {
      ESP_LOGE(TAG, "PSRAM alloc failed for RGBA");
      free(rgba);
      heap_caps_free(png_buf);
      continue;
    }
    memcpy(rgba_ps, rgba, img_bytes);
    free(rgba);
    heap_caps_free(png_buf);

    turn_off();
    turn_on(); 
    TfLiteTensor* input = interpreter.input(0);
    float scale = input->params.scale;
    int zero_point = input->params.zero_point;
    int64_t t4 = esp_timer_get_time();
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
      ESP_LOGE(TAG, "Invoke failed");
      continue;
    }

    turn_off();
    turn_on();
    // profiler.Log(); # failing, even though I have the latest version. there was a push in Jan which made this work 
    int64_t t5 = esp_timer_get_time();
    long long int total_time = (t5 - t4) / 1000; 
    ESP_LOGI(TAG, "Inference in %lld ms", total_time);

    TfLiteTensor* output = interpreter.output(0);
    int8_t* out_data = output->data.int8;
    int best = 0;
    for (int i=1; i<NUM_CLASSES; ++i) {
      if (out_data[i] > out_data[best]) best = i;
    }

    turn_off();
    turn_on(); 
    const char* res = dog_labels[best];
    uart_write_bytes(UART_NUM, res, strlen(res));
    uart_write_bytes(UART_NUM, "\n", 1);
    ESP_LOGI(TAG, "Result: %s", res);

    memset(response, 0, TRANSFER_SIZE);
    response[0] = PACKET_TYPE_GET_DATA_FROM_BLADE;
    size_t res_len = strlen(res) + 1; // +1 to include the trailing '\0'
    memcpy(response + 1, res, res_len);
    memcpy(response + 1 + (res_len + 1), &total_time, sizeof(total_time));

    heap_caps_free(rgba_ps);
    turn_off();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
