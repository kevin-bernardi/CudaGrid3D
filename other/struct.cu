#include <assert.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

struct vect {
    int* values;
};

__global__ void alloc(vect* vct, int n) {
    cudaMalloc((void**)&(vct->values), sizeof(int) * n);

    for (int i = 0; i < n; i++) {
        vct->values[i] = i * 10;
    }
}

// __global__ void getValues(vect* vct, int n, int* value) {
//     cudaMemcpy(&value, &vct->values, sizeof(int) * n,
//     cudaMemcpyDeviceToHost);
// }

__global__ void kernel2(vect in) { in.values[blockIdx.x] *= 2; }

int main() {
    int n = 10;

    int h_vettore[n] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    int* d_vettore;

    vect v_a;

    cudaMalloc((void**)&d_vettore, sizeof(int) * n);

    cudaMemcpy(d_vettore, h_vettore, sizeof(int) * n, cudaMemcpyHostToDevice);

    v_a.values = d_vettore;

    kernel2<<<n, 1>>>(v_a);

    cudaMemcpy(h_vettore, d_vettore, sizeof(int) * n, cudaMemcpyDeviceToHost);

    v_a.values = h_vettore;

    for (int i = 0; i < n; i++) {
        printf("arr[%d]: %d\n", i, v_a.values[i]);
    }
}