#include <assert.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

struct point3D {
  double x;
  double y;
  double z;
};

__global__ void checkValueOnDevice(point3D *d_arr, int n) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;

  if (tid < n) {
    if (tid == n - 1) {
      printf("reached last point\n");
    }
    point3D pt = d_arr[tid];

    if (pt.x != tid || pt.y != tid || pt.z != tid) {
      printf("Value is not correct");
    }
  }
}

int main() {
  int n = 100000000;
  bool check = false;
  bool checkOnHost = true;

  struct timeval t1, t2;

  int num_blocks = (n + 256) / 256;

  printf("begin transfer test\n");

  point3D *array = (point3D *)malloc(sizeof(point3D) * n);

  if (array == NULL) {
    printf("memory problem! returning...");
    return 1;
  }

  for (int i = 0; i < n; i++) {
    point3D pt;
    pt.x = i;
    pt.y = i;
    pt.z = i;

    array[i] = pt;
  }

  point3D *d_array;

  cudaMalloc((void **)&d_array, sizeof(point3D) * n);

  gettimeofday(&t1, NULL);

  cudaMemcpy(d_array, array, sizeof(point3D) * n, cudaMemcpyHostToDevice);

  if (check == true) {
    checkValueOnDevice<<<num_blocks, 256>>>(d_array, n);
    cudaDeviceSynchronize();
  }

  if (checkOnHost == true) {
    for (int i = 0; i < n; i++) {
      point3D pt = array[i];

      if (pt.x != i || pt.y != i || pt.z != i) {
        printf("Value is not correct\n");
      }
    }
  }

  for (int i = 0; i < n; i++) {
    point3D pt;
    pt.x = i;
    pt.y = i;
    pt.z = i;

    array[i] = pt;
  }

  gettimeofday(&t2, NULL);

  printf(
      "transfer took %f ms\n",
      ((t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec) / 1000.0);
}