#include <stdio.h>
#include <stdlib.h>
#include <lz4frame.h>

#define CHANNELS 3  // Always RGB
#define MAX_DECOMPRESSED_SIZE (1920 * 1080 * CHANNELS)  // Max 1920x1080 RGB image

void decompress_lz4(const char *input_file, const char *output_txt) {
    FILE *in_file = fopen(input_file, "rb");
    if (!in_file) {
        perror("Failed to open input file");
        return;
    }

    // Get file size
    fseek(in_file, 0, SEEK_END);
    size_t compressed_size = ftell(in_file);
    rewind(in_file);

    // Allocate buffer for compressed data
    unsigned char *compressed_data = (unsigned char *)malloc(compressed_size);
    if (!compressed_data) {
        perror("Memory allocation failed for compressed data");
        fclose(in_file);
        return;
    }
    fread(compressed_data, 1, compressed_size, in_file);
    fclose(in_file);

    // Allocate buffer for decompressed data
    unsigned char *decompressed_data = (unsigned char *)malloc(MAX_DECOMPRESSED_SIZE);
    if (!decompressed_data) {
        perror("Memory allocation failed for decompressed data");
        free(compressed_data);
        return;
    }

    // Create decompression context
    LZ4F_dctx *dctx;
    size_t ctx_status = LZ4F_createDecompressionContext(&dctx, LZ4F_VERSION);
    if (LZ4F_isError(ctx_status)) {
        fprintf(stderr, "Failed to create decompression context: %s\n", LZ4F_getErrorName(ctx_status));
        free(compressed_data);
        free(decompressed_data);
        return;
    }

    // Decompress data
    size_t decompressed_size = MAX_DECOMPRESSED_SIZE;
    size_t ret = LZ4F_decompress(dctx, decompressed_data, &decompressed_size, compressed_data, &compressed_size, NULL);
    if (LZ4F_isError(ret)) {
        fprintf(stderr, "Decompression failed: %s\n", LZ4F_getErrorName(ret));
        free(compressed_data);
        free(decompressed_data);
        LZ4F_freeDecompressionContext(dctx);
        return;
    }

    // Free decompression context
    LZ4F_freeDecompressionContext(dctx);

    // Write RGB values to output file
    FILE *out_file = fopen(output_txt, "w");
    if (!out_file) {
        perror("Failed to open output file");
        free(compressed_data);
        free(decompressed_data);
        return;
    }

    for (size_t i = 0; i < decompressed_size; i += CHANNELS) {
        fprintf(out_file, "%d %d %d\n", decompressed_data[i], decompressed_data[i+1], decompressed_data[i+2]);
    }

    fclose(out_file);
    free(compressed_data);
    free(decompressed_data);
    printf("Decompression completed: %s\n", output_txt);
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: %s <input.lz4> <output.txt>\n", argv[0]);
        return 1;
    }

    decompress_lz4(argv[1], argv[2]);
    return 0;
}
