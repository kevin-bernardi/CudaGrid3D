/*

BASIC GUIDE

host variable/code/function... -> variable/code/function... allocated on CPU + RAM
device variable/code/function... -> variable/code/function... allocated on GPU + VRAM

d_variable -> variable allocated on the device (it is not mandatory, just to remember where a variable is allocated)
h_variable -> variable allocated on the host

kernel functions (with the __global__ void before the name of the function) are not described because are called by wrapper host functions (which are described) to simplify
the signature of the function calls from outside the library

*/

#include <assert.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

#include "CImg.h"
#include "grid3d.h"

using namespace cimg_library;

#define UNKNOWN_CELL 0
#define OCCUPIED_CELL 1
#define FREE_CELL 2

float approxFloat(float x) {
    if (x >= 0) {
        return floor(x);
    } else {
        return ceil(x);
    }
}

__device__ float approxFloatKernel(float x) {
    if (x >= 0.0) {
        return floor(x);
    } else {
        return ceil(x);
    }
}

__global__ void initMapKernel(char *d_grid_2D, char *d_grid_3D, int numCellsGrid2D, int numCellsGrid3D) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    // if (tid == numCells - 1) {
    //     printf("initializing vector3D in kernel\n");
    // }

    if (tid < numCellsGrid2D) {
        d_grid_2D[tid] = UNKNOWN_CELL;
    }

    if (tid < numCellsGrid3D) {
        d_grid_3D[tid] = UNKNOWN_CELL;
    }
}

// initialize 3d grid, make every cell false
void initMap(Map *h_map, int dimX, int dimY, int dimZ, float resolution, int freeVoxelsMargin, int robotVoxelsHeight) {
    h_map->dimX = dimX;
    h_map->dimY = dimY;
    h_map->dimZ = dimZ;
    h_map->resolution = resolution;
    h_map->freeVoxelsMargin = freeVoxelsMargin;
    h_map->robotVoxelsHeight = robotVoxelsHeight;

    if (robotVoxelsHeight < freeVoxelsMargin) {
        printf("ERROR initMap(): robotVoxelsHeight must be higher than freeVoxelsMargin\n");
        return;
    }

    if (robotVoxelsHeight * 1.2 > dimZ) {
        printf("ERROR initMap(): robotVoxelsHeight * 1.2 can't be higher than dimZ\n");
        return;
    }

    int numCellsGrid2D = h_map->dimX * h_map->dimY;
    int numCellsGrid3D = h_map->dimX * h_map->dimY * h_map->dimZ;

    int numBlocks = (numCellsGrid3D + 256) / 256;

    // grid2D on the device (GPU)
    char *d_grid_2D;

    // grid3D on the device (GPU)
    char *d_grid_3D;

    // allocate the grid2D on the device (GPU)
    cudaMalloc((void **)&d_grid_2D, numCellsGrid2D * sizeof(char));

    // allocate the grid3D on the device (GPU)
    cudaMalloc((void **)&d_grid_3D, numCellsGrid3D * sizeof(char));

    h_map->d_grid_2D = d_grid_2D;
    h_map->d_grid_3D = d_grid_3D;

    initMapKernel<<<numBlocks, 256>>>(h_map->d_grid_2D, h_map->d_grid_3D, numCellsGrid2D, numCellsGrid3D);
    // cudaDeviceSynchronize();
}

// free vector space
void freeMap(Map *h_map) {
    cudaFree(h_map->d_grid_3D);
    free(h_map);
}

void getCoordinatesInv3D(Map *h_map, int index, int *result) {
    int dimX = h_map->dimX;
    int dimY = h_map->dimY;
    int dimZ = h_map->dimZ;

    if (index >= dimX * dimY * dimZ) {
        printf("getCoordinatesInv3D() ERROR! Index out of bounds!\n");
        result[0] = -1;
        result[1] = -1;
        result[2] = -1;
        return;
    }

    int z = floor(index / (dimX * dimY));

    int x = floor((index - (z * dimX * dimY)) / dimY);

    int y = index - (x * dimY + z * dimX * dimY);

    result[0] = x;
    result[1] = y;
    result[2] = z;

    return;
}

void getCoordinatesInv2D(Map *h_map, int index, int *result) {
    int dimX = h_map->dimX;
    int dimY = h_map->dimY;

    if (index >= dimX * dimY) {
        printf("getCoordinatesInv2D() ERROR! Index out of bounds!\n");
        result[0] = -1;
        result[1] = -1;
        return;
    }

    int x = floor(index / dimY);

    int y = index - (x * dimY);

    result[0] = x;
    result[1] = y;

    return;
}

// input 3 coordinates (x,y,z) of the cell in the 3D grid and get the linear index
int getLinearIndex3D(Map *h_map, int x, int y, int z) {
    if (x < h_map->dimX && y < h_map->dimY && z < h_map->dimZ) {
        int result = y + x * h_map->dimY + z * h_map->dimX * h_map->dimY;
        return result;

    } else {
        printf("Error! Input coordinates are not valid!\n");
        return -1;
    }
}

// input 2 coordinates (x,y) of the cell in the 2D grid and get the linear index
int getLinearIndex2D(Map *h_map, int x, int y) {
    if (x < h_map->dimX && y < h_map->dimY) {
        int result = y + x * h_map->dimY;
        return result;

    } else {
        printf("Error! Input coordinates are not valid!\n");
        return -1;
    }
}

__global__ void insertPointcloudKernel(char *d_grid_2D, char *d_grid_3D, Point *pointcloud, int n, int dimX, int dimY, int dimZ, float resolution, int freeVoxelsMargin, int robotVoxelsHeight) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    // check if point is within the pointcloud vector of lenght n
    if (tid < n) {
        Point pt = pointcloud[tid];
        // check if point is within bounds (correct coordinates)

        float numCellRawX = pt.x / resolution;
        float numCellRawY = pt.y / resolution;
        float numCellRawZ = pt.z / resolution;

        int x = approxFloatKernel(numCellRawX);
        int y = approxFloatKernel(numCellRawY);
        int z = approxFloatKernel(numCellRawZ);

        if (x == 0 && y == 0 && z == 0) {
            // don't add the point at 0,0,0 (without this the mesh would always have a cube at 0,0,0)
            return;
        }

        x += (dimX / 2);
        y += (dimY / 2);
        // z += (dimZ / 10);

        // int x = floor(pt.x / resolution) + (dimX / 2);
        // int y = floor(pt.y / resolution) + (dimY / 2);
        // int z = floor(pt.z / resolution);

        // printf("Floored point (%f, %f, %f): %d, %d, %d\n", pt.x, pt.y, pt.z, x, y, z);

        if (x < dimX && y < dimY && z < dimZ && x >= 0 && y >= 0 && z >= 0) {
            int idx3D = y + x * dimY + z * dimX * dimY;

            // printf("Adding Point (%f, %f, %f)\n", pt.x, pt.y, pt.z);
            // printf("The point is floored to (%d, %d, %d) and added at idx %d\n", x, y, z, idx);
            d_grid_3D[idx3D] = OCCUPIED_CELL;

        } else {
            // point out of bound
            // printf("Point (%d, %d, %d) out of bound!\n", x, y, z);
        }
    }
}

// insert a pointcloud (array of points (x,y,z)) in the 3D grid
void insertPointcloud(Map *h_map, Point *d_pointcloud, int sizePointcloud) {
    if (sizePointcloud <= 0) {
        printf("insertPointcloud() ERROR! sizePointcloud is not valid!\n");
        return;
    }

    int numBlocks = (sizePointcloud + 256) / 256;
    // printf("Size of pointcloud: %d\n", sizePointcloud);

    insertPointcloudKernel<<<numBlocks, 256>>>(h_map->d_grid_2D, h_map->d_grid_3D, d_pointcloud, sizePointcloud, h_map->dimX, h_map->dimY, h_map->dimZ, h_map->resolution, h_map->freeVoxelsMargin, h_map->robotVoxelsHeight);
    cudaDeviceSynchronize();
}

void initDevicePointcloud(Point **d_pointcloud, int length) {
    cudaMalloc((void **)d_pointcloud, sizeof(Point) * length);
}

void initDevicePointcloud(Point **d_pointcloud, Point *h_pointcloud, int length) {
    cudaMalloc((void **)d_pointcloud, sizeof(Point) * length);

    cudaMemcpy(*d_pointcloud, h_pointcloud, sizeof(Point) * length, cudaMemcpyHostToDevice);
}

void freeDevicePointcloud(Point *d_pointcloud) {
    cudaFree(d_pointcloud);
}

__global__ void generateRandomPcKernel(Point *pointcloud, int n, curandState *state, int dimX, int dimY, int dimZ, float resolution) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    float dimX_meters = dimX * resolution;
    float dimY_meters = dimY * resolution;
    float dimZ_meters = dimZ * resolution;

    if (tid < n) {
        if (tid == n - 1) {
            printf("Generating random pointcloud in kernel\n");
        }

        curand_init(clock(), tid, 0, &state[tid]);

        float x = (curand_uniform(&(state[tid])) * dimX_meters) - (dimX_meters / 2.0);
        float y = (curand_uniform(&(state[tid])) * dimY_meters) - (dimY_meters / 2.0);
        float z = (curand_uniform(&(state[tid])) * dimZ_meters);

        Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;

        if (x >= dimX_meters / 2 || y >= dimY_meters / 2 || z >= dimZ_meters || x <= -dimX_meters / 2 || y <= -dimY_meters / 2 || z <= 0) {
            printf("***************ERROR, the generated point doesn't have valid coordinates!***************\n");
            printf("generated point (%f, %f, %f)\n", pt.x, pt.y, pt.z);
        } else {
            // printf("generated point (%f, %f, %f)\n", pt.x, pt.y, pt.z);
            pointcloud[tid] = pt;
        }
    }
}

void generateRandomPointcloud(Map *h_map, Point *pointcloud, int sizePointcloud) {
    int numBlocks = (sizePointcloud + 256) / 256;

    // state for the generation of random numbers in kernel code
    curandState *d_state;

    // allocate the space for state on the gpu
    cudaMalloc((void **)&d_state, sizeof(curandState) * sizePointcloud);

    generateRandomPcKernel<<<numBlocks, 256>>>(pointcloud, sizePointcloud, d_state, h_map->dimX, h_map->dimY, h_map->dimZ, h_map->resolution);

    cudaFree(d_state);

    cudaDeviceSynchronize();
}

__global__ void checkDuplicatesKernel(Point *pointcloud, int *pointcloudIntIdx, int numPoints, int dimX, int dimY, int dimZ, float resolution) {
    for (int i = 0; i < numPoints; i++) {
        Point pt = pointcloud[i];
        int x = floor(pt.x / resolution);
        int y = floor(pt.y / resolution);
        int z = floor(pt.z / resolution);

        int idx = y + x * dimY + z * dimX * dimY;

        pointcloudIntIdx[i] = idx;
    }

    bool duplicates = false;

    for (int i = 0; i < numPoints - 1; i++) {
        for (int j = i + 1; j < numPoints; j++) {
            if (pointcloudIntIdx[j] == pointcloudIntIdx[i]) {
                // printf("Found duplicate! Point idx %d\n", pointcloudIntIdx[j]);
                duplicates = true;
            }
        }
    }

    if (duplicates) {
        printf("WARNING! There are duplicates in the pointcloud!\n");
    } else {
        printf("NO Duplicates in the pointcloud\n");
    }
}

// check if there are duplicate points in the pointcloud (points with the same x,y,z coordinates)
void checkDuplicates(Map *h_map, Point *pointcloud, int sizePointcloud) {
    int *d_pointcloudIntIdx;

    cudaMalloc((void **)&d_pointcloudIntIdx, sizeof(int) * sizePointcloud);
    checkDuplicatesKernel<<<1, 1>>>(pointcloud, d_pointcloudIntIdx, sizePointcloud, h_map->dimX, h_map->dimY, h_map->dimZ, h_map->resolution);
    cudaDeviceSynchronize();
    cudaFree(d_pointcloudIntIdx);
}

// same as printGrid2D but for grids allocated on the host
void printGrid2DHost(char *h_grid_2D, int dimx, int dimy) {
    if (dimx * dimy <= 0) {
        printf("ERROR! Empty Grid\n");
        return;
    }

    printf("\n\n2D Grid Output\n");

    for (int x = 0; x < dimx; x++) {
        for (int y = 0; y < dimy; y++) {
            int idx = y + x * dimy;

            if (y == dimy - 1) {
                printf("%d", h_grid_2D[idx]);
            } else {
                printf("%d | ", h_grid_2D[idx]);
            }
        }

        printf("\n");
    }
}

// same as printGrid but for grids allocated on the host
void printGrid3DHost(char *h_grid_3D, int dimx, int dimy, int dimz) {
    if (dimx * dimy * dimz <= 0) {
        printf("ERROR! Empty Grid\n");
        return;
    }

    printf("\n\n3D Grid Output\n");

    for (int z = 0; z < dimz; z++) {
        printf("Layer %d\n", z);
        for (int x = 0; x < dimx; x++) {
            for (int y = 0; y < dimy; y++) {
                int idx = y + x * dimy + z * dimx * dimy;

                if (y == dimy - 1) {
                    printf("%d", h_grid_3D[idx]);
                } else {
                    printf("%d | ", h_grid_3D[idx]);
                }
            }

            printf("\n");
        }
    }
}

__global__ void printLinearGrid2DKernel(char *d_grid_2D, int numCells) {
    printf("\n\nLinear Grid 2D Output (DEVICE)\n");

    for (int i = 0; i < numCells; i++) {
        printf("%d | ", d_grid_2D[i]);
    }

    printf("\n");
}

// print the grid on a line (as it really is in the memory)
void printLinearGrid2D(Map *h_map) {
    int numCells = h_map->dimX * h_map->dimY;

    if (numCells <= 0) {
        printf("Invalid Vector Size! (num. cells: %d)\n", numCells);
    }

    printLinearGrid2DKernel<<<1, 1>>>(h_map->d_grid_2D, numCells);
    cudaDeviceSynchronize();
}

__global__ void printLinearGrid3DKernel(char *d_grid_3D, int numCells) {
    printf("\n\nLinear Grid 3D Output (DEVICE)\n");

    for (int i = 0; i < numCells; i++) {
        printf("%d | ", d_grid_3D[i]);
    }

    printf("\n");
}

// print the grid on a line (as it really is in the memory)
void printLinearGrid3D(Map *h_map) {
    int numCells = h_map->dimX * h_map->dimY * h_map->dimZ;

    if (numCells <= 0) {
        printf("Invalid Vector Size! (num. cells: %d)\n", numCells);
    }

    printLinearGrid3DKernel<<<1, 1>>>(h_map->d_grid_3D, numCells);
    cudaDeviceSynchronize();
}

__global__ void printGrid2DKernel(char *d_grid_2D, int dimX, int dimY) {
    if (dimX * dimY <= 0) {
        printf("ERROR! Empty matrix\n");
        return;
    }

    printf("\n\n2D Grid Output (DEVICE)\n");

    for (int x = 0; x < dimX; x++) {
        for (int y = 0; y < dimY; y++) {
            int idx = y + x * dimY;

            if (y == dimY - 1) {
                printf("%d", d_grid_2D[idx]);
            } else {
                printf("%d | ", d_grid_2D[idx]);
            }
        }

        printf("\n");
    }
}

// print the 2D grid
void printGrid2D(Map *h_map) {
    printGrid2DKernel<<<1, 1>>>(h_map->d_grid_2D, h_map->dimX, h_map->dimY);
    cudaDeviceSynchronize();
}

__global__ void printGrid3DKernel(char *d_grid_3D, int dimX, int dimY, int dimZ) {
    if (dimX * dimY * dimZ <= 0) {
        printf("ERROR! Empty matrix\n");
        return;
    }

    printf("\n\n3D Grid Output (DEVICE)\n");

    for (int z = 0; z < dimZ; z++) {
        printf("Layer %d\n", z);
        for (int x = 0; x < dimX; x++) {
            for (int y = 0; y < dimY; y++) {
                int idx = y + x * dimY + z * dimX * dimY;

                if (y == dimY - 1) {
                    printf("%d", d_grid_3D[idx]);
                } else {
                    printf("%d | ", d_grid_3D[idx]);
                }
            }

            printf("\n");
        }
    }
}

// print the 3D grid in many 2D planes (dimZ 2D planes of size dimX x dimY)
void printGrid3D(Map *h_map) {
    printGrid3DKernel<<<1, 1>>>(h_map->d_grid_3D, h_map->dimX, h_map->dimY, h_map->dimZ);
    cudaDeviceSynchronize();
}

__global__ void printPcKernel(Point *pointcloud, int sizePointcloud) {
    for (int i = 0; i < sizePointcloud; i++) {
        Point pt = pointcloud[i];
        printf("Point %d: (%f,%f,%f)\n", i, pt.x, pt.y, pt.z);
    }
}

// print the list of points inside the pointcloud
void printPointcloud(Point *pointcloud, int sizePointcloud) {
    printf("\n\nPointcloud Output (DEVICE)\n");
    printPcKernel<<<1, 1>>>(pointcloud, sizePointcloud);
    cudaDeviceSynchronize();
}

void vertex(FILE *file, float x, float y, float z) {
    fprintf(file, "v %f %f %f\n", x, y, z);
}

void face(FILE *file, int v1, int v2, int v3) {
    fprintf(file, "f %d %d %d\n", v1, v2, v3);
}

void squareVertices(FILE *file, float res, float x, float y) {
    vertex(file, x, y, 0);
    vertex(file, x + res, y, 0);
    vertex(file, x, y + res, 0);
    vertex(file, x + res, y + res, 0);
}

void squareFace(FILE *file, int nSquare) {
    int i = (nSquare * 4) + 1;

    face(file, i, i + 1, i + 2);
    face(file, i + 1, i + 2, i + 3);
}

void cubeVertex(FILE *file, float res, float x, float y, float z) {
    vertex(file, x, y, z);
    vertex(file, x, y, z + res);
    vertex(file, x, y + res, z);
    vertex(file, x, y + res, z + res);
    vertex(file, x + res, y, z);
    vertex(file, x + res, y, z + res);
    vertex(file, x + res, y + res, z);
    vertex(file, x + res, y + res, z + res);
}

void cubeFace(FILE *file, int nCube) {
    int i = (nCube * 8) + 1;

    face(file, i, i + 6, i + 4);
    face(file, i, i + 2, i + 6);

    face(file, i, i + 3, i + 2);
    face(file, i, i + 1, i + 3);

    face(file, i + 2, i + 7, i + 6);
    face(file, i + 2, i + 3, i + 7);

    face(file, i + 4, i + 6, i + 7);
    face(file, i + 4, i + 7, i + 5);

    face(file, i, i + 4, i + 5);
    face(file, i, i + 5, i + 1);

    face(file, i + 1, i + 5, i + 7);
    face(file, i + 1, i + 7, i + 3);
}

void generateMeshGrid2D(Map *h_map, const char *path) {
    int dimX = h_map->dimX;
    int dimY = h_map->dimY;

    float resolution = h_map->resolution;

    int numCells = dimX * dimY;

    char *h_grid = (char *)malloc(sizeof(char) * numCells);

    cudaMemcpy(h_grid, h_map->d_grid_2D, sizeof(char) * numCells, cudaMemcpyDeviceToHost);

    FILE *fptr;

    fptr = fopen(path, "w");

    for (int i = 0; i < numCells; i++) {
        if (h_grid[i] == OCCUPIED_CELL) {
            int result[2];
            getCoordinatesInv2D(h_map, i, result);

            float x = result[0];
            float y = result[1];

            x = (x * resolution) - (dimX * resolution / 2);
            y = (y * resolution) - (dimY * resolution / 2);

            squareVertices(fptr, resolution, x, y);
        }
    }

    int nSquare = 0;

    for (int i = 0; i < numCells; i++) {
        if (h_grid[i] == OCCUPIED_CELL) {
            squareFace(fptr, nSquare);
            nSquare++;
        }
    }

    fclose(fptr);
    free(h_grid);
}

void generateMesh(Map *h_map, const char *path) {
    int dimX = h_map->dimX;
    int dimY = h_map->dimY;
    int dimZ = h_map->dimZ;
    float resolution = h_map->resolution;

    int numCells = dimX * dimY * dimZ;

    char *h_grid = (char *)malloc(sizeof(char) * numCells);

    cudaMemcpy(h_grid, h_map->d_grid_3D, sizeof(char) * numCells, cudaMemcpyDeviceToHost);

    FILE *fptr;

    fptr = fopen(path, "w");

    int nCube = 0;

    for (int i = 0; i < numCells; i++) {
        if (h_grid[i] == OCCUPIED_CELL) {
            // float z = floor(i / (dimX * dimY));

            // float x = floor((i - (z * dimX * dimY)) / dimY);

            // float y = i - (x * dimY + z * dimX * dimY);

            int *result = (int *)malloc(sizeof(int) * 3);
            getCoordinatesInv3D(h_map, i, result);

            float x = result[0];
            float y = result[1];
            float z = result[2];

            x = (x * resolution) - (dimX * resolution / 2);
            y = (y * resolution) - (dimY * resolution / 2);
            z = (z * resolution);

            cubeVertex(fptr, resolution, x, y, z);
            free(result);
        }
    }

    for (int i = 0; i < numCells; i++) {
        if (h_grid[i] == OCCUPIED_CELL) {
            cubeFace(fptr, nCube);
            nCube++;
        }
    }

    fclose(fptr);

    free(h_grid);
}

void generateSimpleMesh(Map *h_map, const char *path) {
    float dimX = h_map->dimX;
    float dimY = h_map->dimY;
    float dimZ = h_map->dimZ;
    float resolution = h_map->resolution;

    int numCells = dimX * dimY * dimZ;

    char *h_grid = (char *)malloc(sizeof(char) * numCells);

    cudaMemcpy(h_grid, h_map->d_grid_3D, sizeof(char) * numCells, cudaMemcpyDeviceToHost);

    FILE *fptr;

    fptr = fopen(path, "w");

    for (int i = 0; i < numCells; i++) {
        if (h_grid[i] == OCCUPIED_CELL) {
            int *result = (int *)malloc(sizeof(int) * 3);
            getCoordinatesInv3D(h_map, i, result);

            float x = result[0];
            float y = result[1];
            float z = result[2];

            x = (x * resolution) - (dimX * resolution / 2);
            y = (y * resolution) - (dimY * resolution / 2);
            z = (z * resolution);

            vertex(fptr, x, y, z);
            free(result);
        }
    }

    fclose(fptr);

    free(h_grid);
}

__global__ void arrToPointcloudKernel(Point *d_pointcloud, float *d_arr, int length, CudaTransform3D tf) {
    int tid = (blockIdx.x * blockDim.x + threadIdx.x) * 4;

    if (tid < length) {
        float x, y, z;

        x = d_arr[tid];
        y = d_arr[tid + 1];
        z = d_arr[tid + 2];

        if (!isnan(x) && !isnan(y) && !isnan(z) && !isinf(x) && !isinf(y) && !isinf(z)) {
            Point point;

            point.x = x;
            point.y = y;
            point.z = z;

            Point res;

            // rototranslate point
            res.x = tf.tra[0] + tf.rot[0][0] * point.x + tf.rot[0][1] * point.y + tf.rot[0][2] * point.z;
            res.y = tf.tra[1] + tf.rot[1][0] * point.x + tf.rot[1][1] * point.y + tf.rot[1][2] * point.z;
            res.z = tf.tra[2] + tf.rot[2][0] * point.x + tf.rot[2][1] * point.y + tf.rot[2][2] * point.z;

            // res.x = x;
            // res.y = y;
            // res.z = z;

            // remove floating point division error
            res.x = approxFloatKernel(res.x * 100000.0) / 100000.0;
            res.y = approxFloatKernel(res.y * 100000.0) / 100000.0;
            res.z = approxFloatKernel(res.z * 100000.0) / 100000.0;

            // printf("rt_point in kernel: %f, %f, %f\n", res.x, res.y, res.z);

            d_pointcloud[tid / 4] = res;
        }
    }
}

void cvMatToPointcloud(float *h_cvmat_arr, int length, Point **d_pointcloud, CudaTransform3D tf) {
    float *d_cvmat_arr;
    cudaMalloc(&d_cvmat_arr, sizeof(float) * length);

    // copy the content of host cvmat_arr to device
    cudaMemcpy(d_cvmat_arr, h_cvmat_arr, sizeof(float) * length, cudaMemcpyHostToDevice);

    // initialize the pointcloud that holds the points found in the input matrix
    int numPoints = length / 4;
    initDevicePointcloud(d_pointcloud, numPoints);

    // call kernel function
    int numBlocks = (numPoints + 256) / 256;
    arrToPointcloudKernel<<<numBlocks, 256>>>(*d_pointcloud, d_cvmat_arr, length, tf);

    // free device memory
    cudaFree(d_cvmat_arr);

    cudaDeviceSynchronize();
}

__device__ void printVisitedVoxelsRT(Point *arr, int start, int len) {
    for (int i = 0; i < len; i++) {
        printf("> %f, %f, %f\n", arr[start + i].x, arr[start + i].y, arr[start + i].z);
    }
    printf("\n-------------------\n");
}

__device__ void insertFreePointsKernel(char *d_grid_2D, char *d_grid_3D, int dimX, int dimY, int dimZ, Point *d_free_points, int start, int length, int zMin, int zMax) {
    for (int i = 1; i < length; i++) {
        Point pt = d_free_points[start + i];

        int idx3D = pt.y + pt.x * dimY + pt.z * dimX * dimY;

        if (idx3D < dimX * dimY * dimZ) {
            d_grid_3D[idx3D] = FREE_CELL;
        }
    }
}

__device__ bool checkPointInGridBounds(int dimX, int dimY, int dimZ, Point pt) {
    // point with positive integer like coordinates (coordinates of the cell, not the point in meters)
    if (pt.x < dimX && pt.y < dimY && pt.z < dimZ && pt.x >= 0 && pt.y >= 0 && pt.z >= 0) {
        return true;
    }
    return false;
}

__global__ void rayTracingKernel(Point *d_visited_voxels, int i, int points_per_it, char *d_grid_2D, char *d_grid_3D, int dimX, int dimY, int dimZ, int maxTraversedVoxelsPerRay, float resolution, Point *pointcloud, int sizePointcloud, Point ray_start, int minZ, int maxZ) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    int start_idx = i * points_per_it;
    int d_vst_idx = tid * maxTraversedVoxelsPerRay;

    if (tid < points_per_it && (start_idx + tid) < sizePointcloud) {
        int array_idx = 0;
        float _bin_size = 1.0;
        Point ray_end = pointcloud[start_idx + tid];

        if (ray_end.x == 0.0 && ray_end.y == 0.0 && ray_end.z == 0.0) {
            return;
        }

        // This id of the first/current voxel hit by the ray.
        Point current_voxel;
        current_voxel.x = approxFloatKernel(ray_start.x / resolution);
        current_voxel.y = approxFloatKernel(ray_start.y / resolution);
        current_voxel.z = approxFloatKernel(ray_start.z / resolution);

        // printf("current voxel: %f, %f, %f\n", current_voxel.x, current_voxel.y, current_voxel.z);

        Point last_voxel;
        last_voxel.x = approxFloatKernel(ray_end.x / resolution);
        last_voxel.y = approxFloatKernel(ray_end.y / resolution);
        last_voxel.z = approxFloatKernel(ray_end.z / resolution);

        // end the ray a diagonal cell before

        if (last_voxel.x - current_voxel.x > 0) {
            last_voxel.x -= 1;
        } else if (last_voxel.x - current_voxel.x < 0) {
            last_voxel.x += 1;
        }

        if (last_voxel.y - current_voxel.y > 0) {
            last_voxel.y -= 1;
        } else if (last_voxel.y - current_voxel.y < 0) {
            last_voxel.y += 1;
        }

        if (last_voxel.z - current_voxel.z > 0 && last_voxel.z > resolution) {
            last_voxel.z -= 1;
        } else if (last_voxel.z - current_voxel.z < 0) {
            last_voxel.z += 1;
        }

        // printf("last voxel: %f, %f, %f\n", last_voxel.x, last_voxel.y, last_voxel.z);

        current_voxel.x += (dimX / 2);
        current_voxel.y += (dimY / 2);
        // current_voxel.z += (dimZ / 10);

        last_voxel.x += (dimX / 2);
        last_voxel.y += (dimY / 2);
        // last_voxel.z += (dimZ / 10);

        if (checkPointInGridBounds(dimX, dimY, dimZ, current_voxel)) {
            // printf("current voxel floored: %f, %f, %f\n", current_voxel.x, current_voxel.y, current_voxel.z);
        } else {
            // printf("current voxel out of bounds! pt (%f,%f,%f)\n", current_voxel.x, current_voxel.y, current_voxel.z);
        }

        if (checkPointInGridBounds(dimX, dimY, dimZ, last_voxel)) {
            // printf("last voxel floored: %f, %f, %f\n", last_voxel.x, last_voxel.y, last_voxel.z);
        } else {
            // printf("last voxel out of bounds! pt (%f,%f,%f)\n", last_voxel.x, last_voxel.y, last_voxel.z);
        }

        // Compute normalized ray direction.
        Point ray;

        ray.x = last_voxel.x - current_voxel.x;
        ray.y = last_voxel.y - current_voxel.y;
        ray.z = last_voxel.z - current_voxel.z;

        // printf("ray: %f, %f, %f\n", ray.x, ray.y, ray.z);

        // In which direction the voxel ids are incremented.
        float stepX = (ray.x >= 0) ? 1 : -1;
        float stepY = (ray.y >= 0) ? 1 : -1;
        float stepZ = (ray.z >= 0) ? 1 : -1;

        // Distance along the ray to the next voxel border from the current position (tMaxX, tMaxY, tMaxZ).
        float next_voxel_boundary_x = (current_voxel.x + stepX) * _bin_size;
        float next_voxel_boundary_y = (current_voxel.y + stepY) * _bin_size;
        float next_voxel_boundary_z = (current_voxel.z + stepZ) * _bin_size;

        // tMaxX, tMaxY, tMaxZ -- distance until next intersection with voxel-border
        // the value of t at which the ray crosses the first vertical voxel boundary
        float tMaxX = (ray.x != 0) ? (next_voxel_boundary_x - current_voxel.x) / ray.x : FLT_MAX;
        float tMaxY = (ray.y != 0) ? (next_voxel_boundary_y - current_voxel.y) / ray.y : FLT_MAX;
        float tMaxZ = (ray.z != 0) ? (next_voxel_boundary_z - current_voxel.z) / ray.z : FLT_MAX;

        // tDeltaX, tDeltaY, tDeltaZ --
        // how far along the ray we must move for the horizontal component to equal the width of a voxel
        // the direction in which we traverse the grid
        // can only be FLT_MAX if we never go in that direction
        float tDeltaX = (ray.x != 0) ? _bin_size / ray.x * stepX : FLT_MAX;
        float tDeltaY = (ray.y != 0) ? _bin_size / ray.y * stepY : FLT_MAX;
        float tDeltaZ = (ray.z != 0) ? _bin_size / ray.z * stepZ : FLT_MAX;

        d_visited_voxels[d_vst_idx + array_idx] = current_voxel;
        array_idx++;

        // printf("before cycle tMaxX=%f, tMaxY=%f, tMaxZ=%f\n", tMaxX, tMaxY, tMaxZ);
        while (current_voxel.x != last_voxel.x || current_voxel.y != last_voxel.y || current_voxel.z != last_voxel.z) {
            // printf("tMaxX=%f, tMaxY=%f, tMaxZ=%f\n", tMaxX, tMaxY, tMaxZ);
            if (tMaxX < tMaxY) {
                if (tMaxX < tMaxZ) {
                    current_voxel.x += stepX;
                    tMaxX += tDeltaX;
                } else {
                    current_voxel.z += stepZ;
                    tMaxZ += tDeltaZ;
                }
            } else {
                if (tMaxY < tMaxZ) {
                    current_voxel.y += stepY;
                    tMaxY += tDeltaY;
                } else {
                    current_voxel.z += stepZ;
                    tMaxZ += tDeltaZ;
                }
            }

            d_visited_voxels[d_vst_idx + array_idx] = current_voxel;
            array_idx++;
        }
        // printf("array idx: %d\n", array_idx);
        //  printVisitedVoxels(d_visited_voxels, d_vst_idx, array_idx);

        insertFreePointsKernel(d_grid_2D, d_grid_3D, dimX, dimY, dimZ, d_visited_voxels, d_vst_idx, array_idx, minZ, maxZ);
    }
}

void pointcloudRayTracing(Map *h_map, Point *d_pointcloud, int sizePointcloud, Point origin) {
    // int numBlocks = (sizePointcloud + 256) / 256;

    int dimX = h_map->dimX;
    int dimY = h_map->dimY;
    int dimZ = h_map->dimZ;

    // parallelize everything is not memory feasible!
    // create nIterations serial jobs
    // for each job raytrace points_per_it points which is equal to sizePointcloud/nIterations
    int nIterations = 1000;
    int points_per_it = (sizePointcloud / nIterations) + 1;

    // security factor
    // the number of traversed voxels is equal to the number of traversed voxels by the longest possible ray which is
    // the diagonal of the 3D grid (= sqrt(a^2+b^2+c^2)) multiplied by a security factor (the number of traversed voxel by a ray is AT LEAST equal to the length of the ray expressed in number of voxels)
    int diagonal = floor(sqrt(dimX * dimX + dimY * dimY + dimZ * dimZ));
    int spaceFactor = 10;
    int maxTraversedVoxelsPerRay = spaceFactor * diagonal;

    Point *d_visited_voxels;
    // cudaMalloc only of the space required by a single serial job
    cudaMalloc(&d_visited_voxels, points_per_it * sizeof(Point) * maxTraversedVoxelsPerRay);

    for (int i = 0; i < nIterations; i++) {
        int numBlocks = (points_per_it + 256) / 256;
        rayTracingKernel<<<numBlocks, 256>>>(d_visited_voxels, i, points_per_it, h_map->d_grid_2D, h_map->d_grid_3D, h_map->dimX, h_map->dimY, h_map->dimZ, maxTraversedVoxelsPerRay, h_map->resolution, d_pointcloud, sizePointcloud, origin, h_map->freeVoxelsMargin, h_map->robotVoxelsHeight);
    }

    cudaFree(d_visited_voxels);
}

__global__ void updateGrid2DKernel(char *d_grid_2D, char *d_grid_3D, int dimX, int dimY, int dimZ, int freeVoxelsMargin, int robotVoxelsHeight, int maxUnknownConfidence, int maybeOccupiedConfidence) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    int numPlaneCells = dimX * dimY;

    if (tid < numPlaneCells) {
        int countUnknown = 0;
        int countOccupied = 0;

        int scanHeight = robotVoxelsHeight - freeVoxelsMargin;

        int x = (tid / dimY);
        int y = tid - (x * dimY);

        for (int z = freeVoxelsMargin; z < robotVoxelsHeight; z++) {
            int idx3D = y + x * dimY + z * dimX * dimY;

            if (d_grid_3D[idx3D] == OCCUPIED_CELL) {
                countOccupied++;

                // 10% extension
                // nt extendedScan = (scanHeight + 10) / 10;
                // int increasePerOccupiedVoxel = (100 - maybeOccupiedConfidence) / extendedScan;

                // printf("robot vx height: %d, extendedscan: %d, increase: %d\n", robotVoxelsHeight, extendedScan, increasePerOccupiedVoxel);

                // for (int zExt = robotVoxelsHeight; zExt < (robotVoxelsHeight + extendedScan); zExt++) {
                //     int idx3DExt = y + x * dimY + zExt * dimX * dimY;

                //     if (d_grid_3D[idx3DExt] == OCCUPIED_CELL) {
                //         // occupiedConfidence += increasePerOccupiedVoxel;
                //         // printf("occupied in ext (%d, %d, %d): %d\n", x, y, zExt, occupiedConfidence);
                //         d_grid_2D[tid] = 100;
                //         return;
                //     }
                // }

                // return;

            } else if (d_grid_3D[idx3D] == UNKNOWN_CELL) {
                countUnknown++;
            }
        }

        float unknownRatio = countUnknown / (float)scanHeight;

        int unknownConfidence = unknownRatio * maxUnknownConfidence;
        d_grid_2D[tid] = unknownConfidence;

        if (countOccupied == 1) {
            d_grid_2D[tid] = maybeOccupiedConfidence;
        } else if (countOccupied == 2) {
            d_grid_2D[tid] = maybeOccupiedConfidence + (100 - maybeOccupiedConfidence) / 2;
        } else if (countOccupied >= 2) {
            d_grid_2D[tid] = 100;
        }
    }
}

void updateGrid2D(Map *h_map, int maxUnknownConfidence, int maybeOccupiedConfidence) {
    int dimX = h_map->dimX;
    int dimY = h_map->dimY;
    int dimZ = h_map->dimZ;
    int minZ = h_map->freeVoxelsMargin;
    int maxZ = h_map->robotVoxelsHeight;

    if (maxUnknownConfidence >= maybeOccupiedConfidence) {
        printf("ERROR updateGrid2D(): maxUnknownConfidence can't be higher or equal than maybeOccupiedConfidence");
        return;
    }

    int numCellsPlane = dimX * dimY;

    int numBlocks = (numCellsPlane + 256) / 256;

    updateGrid2DKernel<<<numBlocks, 256>>>(h_map->d_grid_2D, h_map->d_grid_3D, dimX, dimY, dimZ, minZ, maxZ, maxUnknownConfidence, maybeOccupiedConfidence);
    cudaDeviceSynchronize();
}

void visualizeAndSaveGrid2D(Map *h_map, const char *path, bool show, int freeThreshold, int warningThreshold, int occupiedThreshold) {
    if (freeThreshold >= warningThreshold || freeThreshold >= occupiedThreshold) {
        printf("ERROR, freeThreshold must be lower than avery other threshold\n");
        return;
    }

    if (freeThreshold >= warningThreshold || warningThreshold >= occupiedThreshold) {
        printf("ERROR, warningThreshold must be lower than occupiedThreshold and higher than freeThreshold \n");
        return;
    }

    int dimX = h_map->dimX;
    int dimY = h_map->dimY;

    int numCells = dimX * dimY;

    char *h_grid = (char *)malloc(sizeof(char) * numCells);

    cudaMemcpy(h_grid, h_map->d_grid_2D, sizeof(char) * numCells, cudaMemcpyDeviceToHost);

    CImg<unsigned char> image(dimX, dimY, 1, 3, 0);

    const unsigned char free[] = {0, 255, 0};
    const unsigned char unknown[] = {138, 129, 124};
    const unsigned char warning[] = {255, 160, 0};
    const unsigned char occupied[] = {155, 0, 0};

    // cimg inverts x and y axis

    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            int idx = j + i * dimY;
            if (h_grid[idx] <= freeThreshold) {
                image(j, i, 0, 0) = free[0];
                image(j, i, 0, 1) = free[1];
                image(j, i, 0, 2) = free[2];
            } else if (h_grid[idx] > freeThreshold && h_grid[idx] < warningThreshold) {
                image(j, i, 0, 0) = unknown[0];
                image(j, i, 0, 1) = unknown[1];
                image(j, i, 0, 2) = unknown[2];
            } else if (h_grid[idx] >= warningThreshold && h_grid[idx] < occupiedThreshold) {
                image(j, i, 0, 0) = warning[0];
                image(j, i, 0, 1) = warning[1];
                image(j, i, 0, 2) = warning[2];
            } else {
                image(j, i, 0, 0) = occupied[0];
                image(j, i, 0, 1) = occupied[1];
                image(j, i, 0, 2) = occupied[2];
            }
        }
    }

    image.save(path);

    if (show) {
        CImgDisplay main_disp(image, "Cuda Grid 2D");

        while (!main_disp.is_closed()) {
            main_disp.wait();
        }
    }
}