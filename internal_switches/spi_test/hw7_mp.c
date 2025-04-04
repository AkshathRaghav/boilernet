#include <mpi.h> 
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

int main(int argc, char *argv[]) {

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

    uint32_t rank, num_procs;
    
    MPI_Init(&argc, &argv); 
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);

    start = MPI_Wtime(); 

    uint32_t chunk = SIZE / num_procs;
    uint32_t remainder = (SIZE % num_procs); 
    uint32_t local_chunk = (rank < remainder) ? chunk + 1 : chunk;

    uint32_t *local_chunks, *local_offsets;
    uint32_t *local_array;

    local_array = malloc(local_chunk * sizeof(uint32_t)); 

    if (rank == 0)
    {
        uint32_t sum = 0; 
        local_chunks = malloc(num_procs * sizeof(uint32_t));
        local_offsets = malloc(num_procs * sizeof(uint32_t));
        for (int i = 0; i < num_procs; i++) {
            local_chunks[i] = (i < remainder) ? chunk + 1 : chunk;
            local_offsets[i] = sum; 
            sum += local_chunks[i];
        }
    }

    MPI_Scatterv(array, local_chunks, local_offsets, MPI_UINT32_T , local_array, local_chunk, MPI_UINT32_T , 0, MPI_COMM_WORLD);
    uint32_t local_max = naive_iteration(0, local_chunk, local_array);
    MPI_Reduce(&local_max, &calculated_max, 1, MPI_UINT32_T , MPI_MAX, 0, MPI_COMM_WORLD);

    end = MPI_Wtime();

    if (rank == 0) { 
        fprintf(stdout, "Total time for parallel: %f\n", GET_TIME(start, end));

        if (golden_max == calculated_max) {
            fprintf(stdout, "Values match! (%d, %d)\n", calculated_max, golden_max);
        } else {
            fprintf(stdout, "Wrong value! (%d, %d)\n", calculated_max, golden_max);
        }
    }

    free(local_array);
    if (rank == 0) {
        free(array);
        free(local_chunks);
        free(local_offsets);
    }

    MPI_Finalize();
}