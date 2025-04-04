#include <omp.h> 
#include <time.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define ORDER 24
#define SIZE (1<<ORDER) 
#define NUM_THREADS 8

#define GET_TIME(start, end) (((double) (end - start)) / CLOCKS_PER_SEC)

static uint32_t naive_iteration(uint32_t start, uint32_t end, uint32_t *array) { 
    uint32_t local_max = 0;
    
    for (uint32_t i = start; i < end; i++)
    {
        if (array[i] > local_max) local_max = array[i];
    }

    return local_max; 
}

int main() {

    clock_t start, end; 
    srand(time(NULL));

    uint32_t golden_max = 0;
    uint32_t *array = malloc(SIZE * sizeof(uint32_t));
    for (uint32_t i = 0; i < SIZE; i++) {
        array[i] = rand();
    }

    start = clock(); 
    golden_max = naive_iteration(0, SIZE, array);
    end = clock();

    fprintf(stdout, "Total time for naive: %f\n", GET_TIME(start, end));

    uint32_t chunk_size = (SIZE / NUM_THREADS);
    uint32_t calculated_max = 0;

    start = clock(); 
    omp_set_num_threads(NUM_THREADS);
#pragma omp parallel
    {
        uint32_t id = omp_get_thread_num();
        uint32_t local_start = chunk_size * id;
        uint32_t local_end = (id == NUM_THREADS - 1) ? SIZE : local_start + chunk_size;
        uint32_t private_max = naive_iteration(local_start, local_end, array);

        #pragma omp critical  
        if (private_max > calculated_max) calculated_max = private_max; 
    }
    end = clock();

    fprintf(stdout, "Total time for parallel: %f\n", GET_TIME(start, end));

    if (golden_max == calculated_max) {
        fprintf(stdout, "Values match! (%d, %d)\n", calculated_max, golden_max);
    } else {
        fprintf(stdout, "Wrong value! (%d, %d)\n", calculated_max, golden_max);
    }
}