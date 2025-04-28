#define BUF_SIZE     (200 * 1024)
#define IMG_SIZE        224
#define NUM_CLASSES     5
#define TENSOR_ARENA_SZ (800 * 1024)  

extern const unsigned char model_tflite[];
extern const unsigned int model_tflite_len;
extern const char* model_labels[];