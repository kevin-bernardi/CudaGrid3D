/*

BASIC GUIDE

host variable/code/function... -> variable/code/function... allocated on CPU + RAM
device variable/code/function... -> variable/code/function... allocated on GPU + VRAM (like ram but only for the gpu)

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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "grid3d.h"

using namespace cv;

#define UNKNOWN_CELL 0
#define OCCUPIED_CELL 1
#define FREE_CELL 2
#define FRONTIER_CELL 150

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

    if (tid < numCellsGrid2D) {
        d_grid_2D[tid] = UNKNOWN_CELL;
    }

    if (tid < numCellsGrid3D) {
        d_grid_3D[tid] = UNKNOWN_CELL;
    }
}

// initialize 3d grid, make every cell false
void CudaGrid3D::initMap(Map *h_map, int dimX, int dimY, int dimZ, float cellSize, int floorVoxelsMargin, int robotVoxelsHeight) {
    // h_map->numFrontiers_3D = 0;
    h_map->dimX = dimX;
    h_map->dimY = dimY;
    h_map->dimZ = dimZ;
    h_map->cellSize = cellSize;
    h_map->floorVoxelsMargin = floorVoxelsMargin;
    h_map->robotVoxelsHeight = robotVoxelsHeight;

    if (robotVoxelsHeight < floorVoxelsMargin) {
        printf("ERROR initMap(): robotVoxelsHeight must be higher than floorVoxelsMargin\n");
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

    // array of frontier cells on the device (GPU)
    // IntPoint *d_frontiers_3D;

    // allocate the grid2D on the device (GPU)
    cudaMalloc((void **)&d_grid_2D, numCellsGrid2D * sizeof(char));

    // allocate the grid3D on the device (GPU)
    cudaMalloc((void **)&d_grid_3D, numCellsGrid3D * sizeof(bool));

    // // allocate the array of frontier cells on the device (GPU)
    // cudaError_t err = cudaMalloc((void **)&d_frontiers_3D, numCellsGrid3D * sizeof(IntPoint));

    // if (err != cudaSuccess) {
    //     printf("ERROR MALLOC\n");
    // } else if (err == cudaSuccess) {
    //     printf("SUCCESS MALLOC\n");
    // }

    h_map->d_grid_2D = d_grid_2D;
    h_map->d_grid_3D = d_grid_3D;
    // h_map->d_frontiers_3D = d_frontiers_3D;

    initMapKernel<<<numBlocks, 256>>>(h_map->d_grid_2D, h_map->d_grid_3D, numCellsGrid2D, numCellsGrid3D);
    cudaDeviceSynchronize();
}

// free map space
void CudaGrid3D::freeMap(Map *h_map) {
    cudaFree(h_map->d_grid_2D);
    cudaFree(h_map->d_grid_3D);
    // cudaFree(h_map->d_frontiers_3D);
    free(h_map);
}

void CudaGrid3D::getCoordinatesInv2D(Map *h_map, int index, int *result) {
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

void CudaGrid3D::getCoordinatesInv3D(Map *h_map, int index, int *result) {
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

// input 2 coordinates (x,y) of the cell in the 2D grid and get the linear index
int CudaGrid3D::getLinearIndex2D(Map *h_map, int x, int y) {
    if (x < h_map->dimX && y < h_map->dimY) {
        int result = y + x * h_map->dimY;
        return result;

    } else {
        printf("Error! Input coordinates are not valid!\n");
        return -1;
    }
}

// input 3 coordinates (x,y,z) of the cell in the 3D grid and get the linear index
int CudaGrid3D::getLinearIndex3D(Map *h_map, int x, int y, int z) {
    if (x < h_map->dimX && y < h_map->dimY && z < h_map->dimZ) {
        int result = y + x * h_map->dimY + z * h_map->dimX * h_map->dimY;
        return result;

    } else {
        printf("Error! Input coordinates are not valid!\n");
        return -1;
    }
}

__device__ int getIdx3D(int dimx, int dimy, int dimz, int x, int y, int z) {
    // check if the idx is inside the 3d grid
    if (x >= 0 && y >= 0 && z >= 0 && x < dimx && y < dimy && z < dimz) {
        return y + x * dimy + z * dimx * dimy;
    }

    // error, out of bound
    return -1;
}

void CudaGrid3D::initDevicePointcloud(Point **d_pointcloud, int numPoints) {
    cudaMalloc((void **)d_pointcloud, sizeof(Point) * numPoints);
}

void CudaGrid3D::initDevicePointcloud(Point **d_pointcloud, Point *h_pointcloud, int numPoints) {
    cudaMalloc((void **)d_pointcloud, sizeof(Point) * numPoints);

    cudaMemcpy(*d_pointcloud, h_pointcloud, sizeof(Point) * numPoints, cudaMemcpyHostToDevice);
}

void CudaGrid3D::freeDevicePointcloud(Point *d_pointcloud) {
    cudaFree(d_pointcloud);
}

__global__ void generateRandomPcKernel(CudaGrid3D::Point *pointcloud, int n, curandState *state, int dimX, int dimY, int dimZ, float cellSize) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    float dimX_meters = dimX * cellSize;
    float dimY_meters = dimY * cellSize;
    float dimZ_meters = dimZ * cellSize;

    if (tid < n) {
        if (tid == n - 1) {
            printf("Generating random pointcloud in kernel\n");
        }

        curand_init(clock(), tid, 0, &state[tid]);

        float x = (curand_uniform(&(state[tid])) * dimX_meters) - (dimX_meters / 2.0);
        float y = (curand_uniform(&(state[tid])) * dimY_meters) - (dimY_meters / 2.0);
        float z = (curand_uniform(&(state[tid])) * dimZ_meters);

        CudaGrid3D::Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;

        if (x >= dimX_meters / 2 || y >= dimY_meters / 2 || z >= dimZ_meters || x <= -dimX_meters / 2 || y <= -dimY_meters / 2 || z <= 0) {
            // printf("***************ERROR, the generated point doesn't have valid coordinates!***************\n");
            // printf("generated point (%f, %f, %f)\n", pt.x, pt.y, pt.z);
        } else {
            // printf("generated point (%f, %f, %f)\n", pt.x, pt.y, pt.z);
            pointcloud[tid] = pt;
        }
    }
}

void CudaGrid3D::generateRandomPointcloud(Map *h_map, Point **d_pointcloud, int numPoints) {
    int numBlocks = (numPoints + 256) / 256;

    // state for the generation of random numbers in kernel code
    curandState *d_state;

    // allocate the space for state on the gpu
    cudaMalloc((void **)&d_state, sizeof(curandState) * numPoints);

    // allocate and initialize the pointcloud on the device memory
    initDevicePointcloud(d_pointcloud, numPoints);

    generateRandomPcKernel<<<numBlocks, 256>>>(*d_pointcloud, numPoints, d_state, h_map->dimX, h_map->dimY, h_map->dimZ, h_map->cellSize);

    cudaFree(d_state);

    // cudaDeviceSynchronize();
}

__global__ void insertPointcloudKernel(char *d_grid_2D, char *d_grid_3D, CudaGrid3D::Point *pointcloud, int n, int dimX, int dimY, int dimZ, float cellSize, int floorVoxelsMargin, int robotVoxelsHeight) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    // check if point is within the pointcloud vector of lenght n
    if (tid < n) {
        CudaGrid3D::Point pt = pointcloud[tid];
        // check if point is within bounds (correct coordinates)

        float numCellRawX = pt.x / cellSize;
        float numCellRawY = pt.y / cellSize;
        float numCellRawZ = pt.z / cellSize;

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

        // int x = floor(pt.x / cellSize) + (dimX / 2);
        // int y = floor(pt.y / cellSize) + (dimY / 2);
        // int z = floor(pt.z / cellSize);

        // printf("Floored point (%f, %f, %f): %d, %d, %d\n", pt.x, pt.y, pt.z, x, y, z);

        if (x < dimX && y < dimY && z < dimZ && x >= 0 && y >= 0 && z >= 0) {
            int idx3D = getIdx3D(dimX, dimY, dimZ, x, y, z);

            // int idx3D = y + x * dimY + z * dimX * dimY;

            // printf("Adding Point (%f, %f, %f)\n", pt.x, pt.y, pt.z);
            // printf("The point is floored to (%d, %d, %d) and added at idx %d\n", x, y, z, idx);

            // always check if idx3D is valid (getIdx3D returns -1 if the point is out of bound)
            if (idx3D >= 0) {
                d_grid_3D[idx3D] = OCCUPIED_CELL;
            }

        } else {
            // point out of bound
            // printf("Point (%d, %d, %d) out of bound!\n", x, y, z);
        }
    }
}

// insert a pointcloud (array of points (x,y,z)) in the 3D grid
void CudaGrid3D::insertPointcloud(Map *h_map, Point *d_pointcloud, int numPoints) {
    if (numPoints <= 0) {
        // printf("insertPointcloud() ERROR! numPoints is not valid!\n");
        return;
    }

    int numBlocks = (numPoints + 256) / 256;
    // printf("Size of pointcloud: %d\n", numPoints);

    insertPointcloudKernel<<<numBlocks, 256>>>(h_map->d_grid_2D, h_map->d_grid_3D, d_pointcloud, numPoints, h_map->dimX, h_map->dimY, h_map->dimZ, h_map->cellSize, h_map->floorVoxelsMargin, h_map->robotVoxelsHeight);
    // cudaDeviceSynchronize();
}

__global__ void checkDuplicatesKernel(CudaGrid3D::Point *pointcloud, int *pointcloudIntIdx, int numPoints, int dimX, int dimY, int dimZ, float cellSize) {
    for (int i = 0; i < numPoints; i++) {
        CudaGrid3D::Point pt = pointcloud[i];
        int x = floor(pt.x / cellSize);
        int y = floor(pt.y / cellSize);
        int z = floor(pt.z / cellSize);

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
        // printf("WARNING! There are duplicates in the pointcloud!\n");
    } else {
        // printf("NO Duplicates in the pointcloud\n");
    }
}

// check if there are duplicate points in the pointcloud (points with the same x,y,z coordinates)
void CudaGrid3D::checkDuplicates(Map *h_map, Point *pointcloud, int numPoints) {
    int *d_pointcloudIntIdx;

    cudaMalloc((void **)&d_pointcloudIntIdx, sizeof(int) * numPoints);
    checkDuplicatesKernel<<<1, 1>>>(pointcloud, d_pointcloudIntIdx, numPoints, h_map->dimX, h_map->dimY, h_map->dimZ, h_map->cellSize);
    cudaDeviceSynchronize();
    cudaFree(d_pointcloudIntIdx);
}

__global__ void arrToPointcloudKernel(CudaGrid3D::Point *d_pointcloud, float *d_arr, int length, bool enableRototranslation, CudaGrid3D::CudaTransform3D tf) {
    int tid = (blockIdx.x * blockDim.x + threadIdx.x) * 4;

    if (tid < length) {
        float x, y, z;

        x = d_arr[tid];
        y = d_arr[tid + 1];
        z = d_arr[tid + 2];

        if (!isnan(x) && !isnan(y) && !isnan(z) && !isinf(x) && !isinf(y) && !isinf(z)) {
            CudaGrid3D::Point point;

            point.x = x;
            point.y = y;
            point.z = z;

            CudaGrid3D::Point res;

            if (enableRototranslation) {
                // rototranslate point
                res.x = tf.tra[0] + tf.rot[0][0] * point.x + tf.rot[0][1] * point.y + tf.rot[0][2] * point.z;
                res.y = tf.tra[1] + tf.rot[1][0] * point.x + tf.rot[1][1] * point.y + tf.rot[1][2] * point.z;
                res.z = tf.tra[2] + tf.rot[2][0] * point.x + tf.rot[2][1] * point.y + tf.rot[2][2] * point.z;

            } else {
                res.x = x;
                res.y = y;
                res.z = z;
            }

            // remove floating point division error
            res.x = approxFloatKernel(res.x * 100000.0) / 100000.0;
            res.y = approxFloatKernel(res.y * 100000.0) / 100000.0;
            res.z = approxFloatKernel(res.z * 100000.0) / 100000.0;

            // printf("rt_point in kernel: %f, %f, %f\n", res.x, res.y, res.z);

            d_pointcloud[tid / 4] = res;
        }
    }
}

void CudaGrid3D::arrayToPointcloud(float *h_cvmat_arr, int length, Point **d_pointcloud) {
    float *d_cvmat_arr;
    cudaMalloc(&d_cvmat_arr, sizeof(float) * length);

    // copy the content of host cvmat_arr to device
    cudaMemcpy(d_cvmat_arr, h_cvmat_arr, sizeof(float) * length, cudaMemcpyHostToDevice);

    // initialize the pointcloud that holds the points found in the input matrix
    int numPoints = length / 4;
    initDevicePointcloud(d_pointcloud, numPoints);

    // call kernel function
    int numBlocks = (numPoints + 256) / 256;
    CudaTransform3D tf;

    // just to remove a warning
    tf.tra[0] = 0.0;

    arrToPointcloudKernel<<<numBlocks, 256>>>(*d_pointcloud, d_cvmat_arr, length, false, tf);

    // free device memory
    cudaFree(d_cvmat_arr);

    // cudaDeviceSynchronize();
}

void CudaGrid3D::arrayToPointcloud(float *h_cvmat_arr, int length, Point **d_pointcloud, CudaTransform3D tf) {
    float *d_cvmat_arr;
    cudaMalloc(&d_cvmat_arr, sizeof(float) * length);

    // copy the content of host cvmat_arr to device
    cudaMemcpy(d_cvmat_arr, h_cvmat_arr, sizeof(float) * length, cudaMemcpyHostToDevice);

    // initialize the pointcloud that holds the points found in the input matrix
    int numPoints = length / 4;
    initDevicePointcloud(d_pointcloud, numPoints);

    // call kernel function
    int numBlocks = (numPoints + 256) / 256;
    arrToPointcloudKernel<<<numBlocks, 256>>>(*d_pointcloud, d_cvmat_arr, length, true, tf);

    // free device memory
    cudaFree(d_cvmat_arr);

    // cudaDeviceSynchronize();
}

void CudaGrid3D::printHostGrid2D(char *h_grid_2D, int dimx, int dimy) {
    if (dimx * dimy <= 0) {
        printf("ERROR! Empty Grid\n");
        return;
    }

    printf("\n\nHost 2D Grid Output\n\n");

    for (int x = 0; x < dimx; x++) {
        for (int y = 0; y < dimy; y++) {
            int idx = y + x * dimy;

            if (y == 0)
                printf("| %d |", h_grid_2D[idx]);
            else
                printf(" %d |", h_grid_2D[idx]);
        }

        printf("\n");
    }
}

void CudaGrid3D::printHostGrid3D(char *h_grid_3D, int dimx, int dimy, int dimz) {
    if (dimx * dimy * dimz <= 0) {
        printf("ERROR! Empty Grid\n");
        return;
    }

    printf("\n\nHost 3D Grid Output\n");

    for (int z = 0; z < dimz; z++) {
        printf("\nLayer %d\n\n", z);
        for (int x = 0; x < dimx; x++) {
            for (int y = 0; y < dimy; y++) {
                int idx = y + x * dimy + z * dimx * dimy;

                if (y == 0)
                    printf("| %d |", h_grid_3D[idx]);
                else
                    printf(" %d |", h_grid_3D[idx]);
            }

            printf("\n");
        }
    }
}

__global__ void printDeviceGrid2DKernel(char *d_grid_2D, int dimX, int dimY) {
    printf("\n\nDevice 2D Grid Output\n\n");

    for (int x = 0; x < dimX; x++) {
        for (int y = 0; y < dimY; y++) {
            int idx = y + x * dimY;

            if (y == 0)
                printf("| %d |", d_grid_2D[idx]);
            else
                printf(" %d |", d_grid_2D[idx]);
        }

        printf("\n");
    }
}

// print the 2D grid
void CudaGrid3D::printDeviceGrid2D(char *d_grid_2D, int dimx, int dimy) {
    if (dimx * dimy <= 0) {
        printf("ERROR! Empty matrix\n");
        return;
    }

    printDeviceGrid2DKernel<<<1, 1>>>(d_grid_2D, dimx, dimy);
    cudaDeviceSynchronize();
}

// print the map 2D grid
void CudaGrid3D::printDeviceGrid2D(Map *h_map) {
    printDeviceGrid2D(h_map->d_grid_2D, h_map->dimX, h_map->dimY);
}

__global__ void printDeviceGrid3DKernel(char *d_grid_3D, int dimX, int dimY, int dimZ) {
    printf("\n\nDevice 3D Grid Output\n\n");

    for (int z = 0; z < dimZ; z++) {
        printf("\nLayer %d\n\n", z);
        for (int x = 0; x < dimX; x++) {
            for (int y = 0; y < dimY; y++) {
                int idx = y + x * dimY + z * dimX * dimY;

                if (y == 0)
                    printf("| %d |", d_grid_3D[idx]);
                else
                    printf(" %d |", d_grid_3D[idx]);
            }

            printf("\n");
        }
    }
}

// print device 3D Grid
void CudaGrid3D::printDeviceGrid3D(char *d_grid_3D, int dimx, int dimy, int dimz) {
    if (dimx * dimy * dimz <= 0) {
        printf("ERROR! Empty matrix\n");
        return;
    }

    printDeviceGrid3DKernel<<<1, 1>>>(d_grid_3D, dimx, dimy, dimz);
    cudaDeviceSynchronize();
}

// print the map 3D grid
void CudaGrid3D::printDeviceGrid3D(Map *h_map) {
    printDeviceGrid3D(h_map->d_grid_3D, h_map->dimX, h_map->dimY, h_map->dimZ);
}

__global__ void printDeviceLinearGrid2DKernel(char *d_grid_2D, int numCells) {
    printf("\n\nDevice Linear Grid 2D Output\n\n");

    for (int i = 0; i < numCells; i++) {
        if (i == 0)
            printf("| %d |", d_grid_2D[i]);
        else
            printf(" %d |", d_grid_2D[i]);
    }

    printf("\n");
}

void CudaGrid3D::printDeviceLinearGrid2D(char *d_grid_2D, int dimx, int dimy) {
    int numCells = dimx * dimy;

    if (numCells <= 0) {
        printf("Invalid Vector Size! (num. cells: %d)\n", numCells);
    }

    printDeviceLinearGrid2DKernel<<<1, 1>>>(d_grid_2D, numCells);
    cudaDeviceSynchronize();
}

// print the map 2D Grid as a 1D array
void CudaGrid3D::printDeviceLinearGrid2D(Map *h_map) {
    printDeviceLinearGrid2D(h_map->d_grid_2D, h_map->dimX, h_map->dimY);
}

__global__ void printDeviceLinearGrid3DKernel(char *d_grid_3D, int numCells) {
    printf("\n\nDevice Linear Grid 3D Output\n\n");

    for (int i = 0; i < numCells; i++) {
        if (i == 0)
            printf("| %d |", d_grid_3D[i]);
        else
            printf(" %d |", d_grid_3D[i]);
    }

    printf("\n");
}

void CudaGrid3D::printDeviceLinearGrid3D(char *d_grid_3D, int dimx, int dimy, int dimz) {
    int numCells = dimx * dimy * dimz;

    if (numCells <= 0) {
        printf("Invalid Vector Size! (num. cells: %d)\n", numCells);
    }

    printDeviceLinearGrid3DKernel<<<1, 1>>>(d_grid_3D, numCells);
    cudaDeviceSynchronize();
}

// print the map 3D Grid as a 1D array
void CudaGrid3D::printDeviceLinearGrid3D(Map *h_map) {
    printDeviceLinearGrid3D(h_map->d_grid_3D, h_map->dimX, h_map->dimY, h_map->dimZ);
}

__global__ void printPcKernel(CudaGrid3D::Point *pointcloud, int numPoints) {
    for (int i = 0; i < numPoints; i++) {
        CudaGrid3D::Point pt = pointcloud[i];
        printf("Point %d: (%f,%f,%f)\n", i, pt.x, pt.y, pt.z);
    }
}

// print the list of points inside the pointcloud
void CudaGrid3D::printPointcloud(Point *pointcloud, int numPoints) {
    printf("\n\nPointcloud Output (DEVICE)\n");
    printPcKernel<<<1, 1>>>(pointcloud, numPoints);
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

void CudaGrid3D::generateMesh2D(Map *h_map, const char *path) {
    int dimX = h_map->dimX;
    int dimY = h_map->dimY;

    float cellSize = h_map->cellSize;

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

            x = (x * cellSize) - (dimX * cellSize / 2);
            y = (y * cellSize) - (dimY * cellSize / 2);

            squareVertices(fptr, cellSize, x, y);
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

void CudaGrid3D::generateMesh3D(Map *h_map, const char *path) {
    int dimX = h_map->dimX;
    int dimY = h_map->dimY;
    int dimZ = h_map->dimZ;
    float cellSize = h_map->cellSize;

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

            x = (x * cellSize) - (dimX * cellSize / 2);
            y = (y * cellSize) - (dimY * cellSize / 2);
            z = (z * cellSize);

            cubeVertex(fptr, cellSize, x, y, z);
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

void CudaGrid3D::generateSimpleMesh3D(Map *h_map, const char *path, MeshType meshType) {
    float dimX = h_map->dimX;
    float dimY = h_map->dimY;
    float dimZ = h_map->dimZ;
    float cellSize = h_map->cellSize;

    int numCells = dimX * dimY * dimZ;

    char *h_grid = (char *)malloc(sizeof(char) * numCells);

    cudaMemcpy(h_grid, h_map->d_grid_3D, sizeof(char) * numCells, cudaMemcpyDeviceToHost);

    FILE *fptr;

    fptr = fopen(path, "w");

    char checkStatus;

    if (meshType == OCCUPANCY_MAP) {
        // if isOccupationMesh is true build a mesh where obstacles are drawn
        checkStatus = OCCUPIED_CELL;
    } else if (meshType == FREE_MAP) {
        // otherwise draw the free space
        checkStatus = FREE_CELL;
    } else if (meshType == FRONTIER_MAP) {
        checkStatus = FRONTIER_CELL;
    }

    for (int i = 0; i < numCells; i++) {
        if (h_grid[i] == checkStatus) {
            int *result = (int *)malloc(sizeof(int) * 3);
            getCoordinatesInv3D(h_map, i, result);

            float x = result[0];
            float y = result[1];
            float z = result[2];

            x = (x * cellSize) - (dimX * cellSize / 2);
            y = (y * cellSize) - (dimY * cellSize / 2);
            z = (z * cellSize);

            vertex(fptr, x, y, z);
            free(result);
        }
    }

    fclose(fptr);

    free(h_grid);
}

__device__ void printVisitedVoxelsRT(CudaGrid3D::Point *arr, int start, int len) {
    for (int i = 0; i < len; i++) {
        printf("> %f, %f, %f\n", arr[start + i].x, arr[start + i].y, arr[start + i].z);
    }
    printf("\n-------------------\n");
}

__device__ bool checkPointInGridBounds(int dimX, int dimY, int dimZ, CudaGrid3D::Point pt) {
    // point with positive integer coordinates (coordinates of the cell, not the point in meters)
    if (pt.x < dimX && pt.y < dimY && pt.z < dimZ && pt.x >= 0 && pt.y >= 0 && pt.z >= 0) {
        return true;
    }
    return false;
}

__global__ void rayTracingKernel(char *d_grid_3D, CudaGrid3D::Point *d_pointcloud, int numPoints, int dimX, int dimY, int dimZ, float cellSize, CudaGrid3D::Point ray_start, bool freeObstacles) {
    // thread id
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid < numPoints) {
        float _bin_size = 1.0;
        CudaGrid3D::Point ray_end = d_pointcloud[tid];

        if (ray_end.x == 0.0 && ray_end.y == 0.0 && ray_end.z == 0.0) {
            return;
        }

        // This id of the first/current voxel hit by the ray.
        CudaGrid3D::Point current_voxel;
        current_voxel.x = approxFloatKernel(ray_start.x / cellSize);
        current_voxel.y = approxFloatKernel(ray_start.y / cellSize);
        current_voxel.z = approxFloatKernel(ray_start.z / cellSize);

        // printf("current voxel: %f, %f, %f\n", current_voxel.x, current_voxel.y, current_voxel.z);

        CudaGrid3D::Point last_voxel;
        last_voxel.x = approxFloatKernel(ray_end.x / cellSize);
        last_voxel.y = approxFloatKernel(ray_end.y / cellSize);
        last_voxel.z = approxFloatKernel(ray_end.z / cellSize);

        // printf("last voxel: %f, %f, %f\n", last_voxel.x, last_voxel.y, last_voxel.z);

        current_voxel.x += (dimX / 2);
        current_voxel.y += (dimY / 2);
        // current_voxel.z += (dimZ / 10);

        last_voxel.x += (dimX / 2);
        last_voxel.y += (dimY / 2);
        // last_voxel.z += (dimZ / 10);

        // end the ray a diagonal cell before if freeObstacles is active (true)

        if (freeObstacles) {
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

            if (last_voxel.z - current_voxel.z > 0 && last_voxel.z > 0) {
                last_voxel.z -= 1;
            } else if (last_voxel.z - current_voxel.z < 0) {
                last_voxel.z += 1;
            }
        }

        if (checkPointInGridBounds(dimX, dimY, dimZ, current_voxel)) {
            // printf("current voxel floored: %f, %f, %f\n", current_voxel.x, current_voxel.y, current_voxel.z);
        } else {
            return;
            // printf("current voxel out of bounds! pt (%f,%f,%f)\n", current_voxel.x, current_voxel.y, current_voxel.z);
        }

        if (checkPointInGridBounds(dimX, dimY, dimZ, last_voxel)) {
            // printf("last voxel floored: %f, %f, %f\n", last_voxel.x, last_voxel.y, last_voxel.z);
        } else {
            return;
            // printf("last voxel out of bounds! pt (%f,%f,%f)\n", last_voxel.x, last_voxel.y, last_voxel.z);
        }

        // Compute normalized ray direction.
        CudaGrid3D::Point ray;

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

            int idx3D = getIdx3D(dimX, dimY, dimZ, current_voxel.x, current_voxel.y, current_voxel.z);

            if (idx3D >= 0) {
                // if freeObstacles is true we must know if we are freeing the last pixel of the ray (the second big condition checks if we are at the last iteration of the while cycle)
                if (freeObstacles && (current_voxel.x != last_voxel.x || current_voxel.y != last_voxel.y || current_voxel.z != last_voxel.z)) {
                    // do not check if it's an obstacles, set it as free anyway
                    d_grid_3D[idx3D] = FREE_CELL;
                } else if (!freeObstacles && d_grid_3D[idx3D] != OCCUPIED_CELL) {
                    // the cell is not an obstacle
                    d_grid_3D[idx3D] = FREE_CELL;
                } else if (!freeObstacles && d_grid_3D[idx3D == OCCUPIED_CELL]) {
                    // interrupt the ray, we encountered an obstacle before its end
                    break;
                }
            }
        }
    }
}

void CudaGrid3D::pointcloudRayTracing(Map *h_map, CudaGrid3D::Point *d_pointcloud, int numPoints, CudaGrid3D::Point origin, bool freeObstacles) {
    int numBlocks = (numPoints + 256) / 256;
    rayTracingKernel<<<numBlocks, 256>>>(h_map->d_grid_3D, d_pointcloud, numPoints, h_map->dimX, h_map->dimY, h_map->dimZ, h_map->cellSize, origin, freeObstacles);
    cudaDeviceSynchronize();
}

__global__ void findFrontiers3DKernel(char *d_grid_3D, int dimX, int dimY, int dimZ) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid < dimX * dimY * dimZ) {
        if (d_grid_3D[tid] != FREE_CELL) {
            return;
        }

        int z = tid / (dimX * dimY);
        int x = (tid - (z * dimX * dimY)) / dimY;
        int y = tid - (x * dimY + z * dimX * dimY);

        if (x == 0 || x >= dimX - 1 || y == 0 || y == dimY - 1 || z == 0 || z == dimZ - 1) {
            return;
        }

        // array of 1D indices of the 8 neighbours on the horizontal plane (same z) of the point ("o" in the grid below)
        //  |o|o|o|
        //  |o|x|o|
        //  |o|o|o|

        int neighbourCellsIndices[8];

        neighbourCellsIndices[0] = getIdx3D(dimX, dimY, dimZ, x - 1, y - 1, z);
        neighbourCellsIndices[1] = getIdx3D(dimX, dimY, dimZ, x, y - 1, z);
        neighbourCellsIndices[2] = getIdx3D(dimX, dimY, dimZ, x + 1, y - 1, z);
        neighbourCellsIndices[3] = getIdx3D(dimX, dimY, dimZ, x - 1, y, z);
        neighbourCellsIndices[4] = getIdx3D(dimX, dimY, dimZ, x + 1, y, z);
        neighbourCellsIndices[5] = getIdx3D(dimX, dimY, dimZ, x - 1, y + 1, z);
        neighbourCellsIndices[6] = getIdx3D(dimX, dimY, dimZ, x, y + 1, z);
        neighbourCellsIndices[7] = getIdx3D(dimX, dimY, dimZ, x + 1, y + 1, z);

        // int index = 0;

        // int neighbourCellsIndices[26];

        // for (int i = -1; i <= 1; i++) {
        //     for (int j = -1; j <= 1; j++) {
        //         for (int k = -1; k <= 1; k++) {
        //             if (i != 0 || j != 0 || k != 0) {
        //                 neighbourCellsIndices[index] = getIdx3D(dimX, dimY, dimZ, x + i, y + i, z + i);
        //                 index++;
        //             }
        //         }
        //     }
        // }

        int nUnknown = 0;
        bool foundOccupied = false;

        for (int i = 0; i < 8; i++) {
            int idx3D = neighbourCellsIndices[i];

            if (idx3D >= 0 && idx3D < dimX * dimY * dimZ) {
                // check if the neighbour is unknown
                if (d_grid_3D[idx3D] == UNKNOWN_CELL) {
                    nUnknown++;
                } else if (d_grid_3D[idx3D] == OCCUPIED_CELL) {
                    foundOccupied = true;
                    break;
                }
            }
        }

        if (!foundOccupied && nUnknown >= 3) {
            d_grid_3D[tid] = FRONTIER_CELL;
        }
    }
}

void CudaGrid3D::findFrontiers3D(Map *h_map) {
    int numPoints = h_map->dimX * h_map->dimY * h_map->dimZ;
    int numBlocks = (numPoints + 256) / 256;
    findFrontiers3DKernel<<<numBlocks, 256>>>(h_map->d_grid_3D, h_map->dimX, h_map->dimY, h_map->dimZ);
    cudaDeviceSynchronize();
}

__global__ void updateGrid2DKernel(char *d_grid_2D, char *d_grid_3D, int dimX, int dimY, int dimZ, int floorVoxelsMargin, int robotVoxelsHeight, int maxUnknownConfidence, int minOccupiedConfidence) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    int numPlaneCells = dimX * dimY;

    if (tid < numPlaneCells) {
        int countUnknown = 0;
        int countOccupied = 0;

        // height of the robot after floor margin remotion
        int scanHeight = robotVoxelsHeight - floorVoxelsMargin;

        int x = (tid / dimY);
        int y = tid - (x * dimY);

        for (int z = floorVoxelsMargin; z < robotVoxelsHeight; z++) {
            // int idx3D = y + x * dimY + z * dimX * dimY;
            int idx3D = getIdx3D(dimX, dimY, dimZ, x, y, z);

            // always check if idx3D is valid (getIdx3D returns -1 if the point is out of bound)
            if (idx3D >= 0 && d_grid_3D[idx3D] == OCCUPIED_CELL) {
                countOccupied++;

                // 10% extension
                // int extendedScan = (scanHeight + 10) / 10;
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

            } else if (idx3D >= 0 && d_grid_3D[idx3D] == UNKNOWN_CELL) {
                countUnknown++;
            }
        }

        float unknownRatio = countUnknown / (float)scanHeight;

        // unknownConfidence is capped at maxUnknownConfidence
        int unknownConfidence = unknownRatio * maxUnknownConfidence;
        d_grid_2D[tid] = unknownConfidence;

        if (countOccupied == 1) {
            // only one voxel is occupied
            d_grid_2D[tid] = minOccupiedConfidence;
        } else if (countOccupied == 2) {
            // 2 voxels are occipied: set the confidence at the mid of minOccupiedConfidence
            // and maxOccupiedConfidence (100)
            d_grid_2D[tid] = minOccupiedConfidence + (100 - minOccupiedConfidence) / 2;
        } else if (countOccupied > 2) {
            // more than 2 voxels are occupied, set maximum confidence (100)
            d_grid_2D[tid] = 100;
        }
    }
}

__global__ void findFrontiers2DKernel(char *d_grid_2D, int dimX, int dimY, int freeThreshold, int occupiedThreshold) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    // exit if the cell is not free
    if (d_grid_2D[tid] > freeThreshold) {
        return;
    }

    int x = (tid / dimY);
    int y = tid - (x * dimY);

    if (x == 0 || x >= dimX - 1 || y == 0 || y == dimY - 1) {
        return;
    }

    // array of 1D indices of the 8 neighbours ("o" in the grid below)
    //  |o|o|o|
    //  |o|x|o|
    //  |o|o|o|
    int neighbourCellsIndices[8];

    neighbourCellsIndices[0] = (y - 1) + (x - 1) * dimY;
    neighbourCellsIndices[1] = (y - 1) + (x)*dimY;
    neighbourCellsIndices[2] = (y - 1) + (x + 1) * dimY;
    neighbourCellsIndices[3] = (y) + (x - 1) * dimY;
    neighbourCellsIndices[4] = (y) + (x + 1) * dimY;
    neighbourCellsIndices[5] = (y + 1) + (x - 1) * dimY;
    neighbourCellsIndices[6] = (y + 1) + (x)*dimY;
    neighbourCellsIndices[7] = (y + 1) + (x + 1) * dimY;

    for (int i = 0; i < 8; i++) {
        int idx2D = neighbourCellsIndices[i];
        // check if the neighbour is unknown
        if (d_grid_2D[idx2D] > freeThreshold && d_grid_2D[idx2D] < occupiedThreshold) {
            d_grid_2D[tid] = FRONTIER_CELL;
            break;
        }
    }
}

void CudaGrid3D::updateGrid2D(Map *h_map, int freeThreshold, int maxUnknownConfidence, int minOccupiedConfidence) {
    int dimX = h_map->dimX;
    int dimY = h_map->dimY;
    int dimZ = h_map->dimZ;
    int minZ = h_map->floorVoxelsMargin;
    int maxZ = h_map->robotVoxelsHeight;

    // maxUnknownConfidence must be lower than minOccupiedConfidence
    if (maxUnknownConfidence >= minOccupiedConfidence) {
        printf("ERROR updateGrid2D(): maxUnknownConfidence can't be higher or equal than minOccupiedConfidence");
        return;
    }

    int numCellsPlane = dimX * dimY;

    int numBlocks = (numCellsPlane + 256) / 256;

    updateGrid2DKernel<<<numBlocks, 256>>>(h_map->d_grid_2D, h_map->d_grid_3D, dimX, dimY, dimZ, minZ, maxZ, maxUnknownConfidence, minOccupiedConfidence);

    findFrontiers2DKernel<<<numBlocks, 256>>>(h_map->d_grid_2D, h_map->dimX, h_map->dimY, freeThreshold, minOccupiedConfidence);
    //  cudaDeviceSynchronize();
}

__global__ void binarizeGrid2DKernel(char *d_grid_2D, char *d_grid_2D_bin, int numCells, int freeThreshold, int occupiedThreshold) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid < numCells) {
        char cellValue = d_grid_2D[tid];

        if (cellValue <= freeThreshold) {
            d_grid_2D_bin[tid] = FREE_CELL;
        } else if (cellValue >= occupiedThreshold) {
            d_grid_2D_bin[tid] = OCCUPIED_CELL;
        } else {
            d_grid_2D_bin[tid] = UNKNOWN_CELL;
        }
    }
}

__global__ void grid2DBinningKernel(char *d_grid_2d_binarized, int *d_grid_2d_binned, int dimX, int dimY, int dimX_bin, int dimY_bin, int bin_size) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid < dimX_bin * dimY_bin) {
        d_grid_2d_binned[tid] = 0;
        int x_binned = tid / dimY_bin;               // x coord of the cell in the binned grid
        int y_binned = tid - (x_binned * dimY_bin);  // y coord of the cell in the binned grid

        int x_binarized = x_binned * bin_size;
        int y_binarized = y_binned * bin_size;

        // top-left corner of the (bin_size x bin_size) subgrid of the binarized 2D grid

        for (int i = 0; i < bin_size; i++) {
            int x = x_binarized + i;
            for (int j = 0; j < bin_size; j++) {
                int y = y_binarized + j;

                int idxGrid2D = y + x * dimY;
                if (d_grid_2d_binarized[idxGrid2D] == UNKNOWN_CELL) {
                    d_grid_2d_binned[tid] += 1;
                }
            }
        }
    }
}

void CudaGrid3D::getUnknownDensityGrid2D(Map *h_map, int bin_size, int freeThreshold, int occupiedThreshold, int *&output_grid_2d_binned, int &dimX, int &dimY) {
    if (freeThreshold >= occupiedThreshold) {
        printf("ERROR, freeThreshold must be lower than occupiedThreshold\n");
        return;
    }

    if (bin_size > h_map->dimX || bin_size > h_map->dimY) {
        printf("ERROR, bin_size must be lower or equal than the smallest dimension\n");
        return;
    }

    // binarize grid 2D
    char *d_grid_2D_binarized;
    int numCells = h_map->dimX * h_map->dimY;
    cudaMalloc(&d_grid_2D_binarized, sizeof(char) * numCells);

    int numBlocks = (numCells + 256) / 256;
    binarizeGrid2DKernel<<<numBlocks, 256>>>(h_map->d_grid_2D, d_grid_2D_binarized, numCells, freeThreshold, occupiedThreshold);

    int dimX_bin = h_map->dimX / bin_size;
    int dimY_bin = h_map->dimY / bin_size;

    // grid 2D binning

    int *d_grid_2D_binned;
    int numCellsBinned = dimX_bin * dimY_bin;
    cudaMalloc(&d_grid_2D_binned, sizeof(int) * numCellsBinned);

    int numBlocksBinning = (numCellsBinned + 256) / 256;
    grid2DBinningKernel<<<numBlocksBinning, 256>>>(d_grid_2D_binarized, d_grid_2D_binned, h_map->dimX, h_map->dimY, dimX_bin, dimY_bin, bin_size);

    int *h_grid_2D_binned = (int *)malloc(sizeof(int) * numCellsBinned);
    cudaMemcpy(h_grid_2D_binned, d_grid_2D_binned, sizeof(int) * numCellsBinned, cudaMemcpyDeviceToHost);

    // free grids allocated on the device (GPU)
    cudaFree(d_grid_2D_binarized);
    cudaFree(d_grid_2D_binned);

    // free(h_grid_2D_bin);

    output_grid_2d_binned = h_grid_2D_binned;
    dimX = dimX_bin;
    dimY = dimY_bin;
}

Mat getGrid2DTask(CudaGrid3D::Map *h_map, int freeThreshold, int warningThreshold, int occupiedThreshold, bool displayRobotPosition, CudaGrid3D::CudaTransform3D *robotPosition, int markerRadius) {
    Mat error_img;

    if (freeThreshold >= warningThreshold || freeThreshold >= occupiedThreshold) {
        printf("ERROR, freeThreshold must be lower than both warningThreshold and occupiedThreshold\n");
        return error_img;
    }

    if (freeThreshold >= warningThreshold || warningThreshold >= occupiedThreshold) {
        printf("ERROR, warningThreshold must be lower than occupiedThreshold and higher than freeThreshold \n");
        return error_img;
    }

    int dimX = h_map->dimX;
    int dimY = h_map->dimY;

    int numCells = dimX * dimY;

    // allocate the space on the host for the 2D grid
    char *h_grid = (char *)malloc(sizeof(char) * numCells);

    // to read the image on the host I first need to transfer the 2D Grid on the host

    // transfer the grid from the device to the host
    cudaMemcpy(h_grid, h_map->d_grid_2D, sizeof(char) * numCells, cudaMemcpyDeviceToHost);

    Vec3b free_cv(0, 255, 0);
    Vec3b unknown_cv(124, 129, 138);
    Vec3b warning_cv(0, 160, 255);
    Vec3b occupied_cv(0, 0, 155);
    Vec3b frontier_cv(255, 0, 0);

    Mat data(dimX, dimY, CV_8UC3, unknown_cv);

    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            int idx = j + i * dimY;
            // use a different color based on the confidence value found in the cell
            if (h_grid[idx] <= freeThreshold) {
                // use green
                data.at<Vec3b>(i, j) = free_cv;

            } else if (h_grid[idx] > freeThreshold && h_grid[idx] < warningThreshold) {
                // use grey
                data.at<Vec3b>(i, j) = unknown_cv;

            } else if (h_grid[idx] >= warningThreshold && h_grid[idx] < occupiedThreshold) {
                // use orange
                data.at<Vec3b>(i, j) = warning_cv;
            } else if (h_grid[idx] == FRONTIER_CELL) {
                // use orange
                data.at<Vec3b>(i, j) = frontier_cv;
            } else {
                // use red
                data.at<Vec3b>(i, j) = occupied_cv;
            }
        }
    }

    if (displayRobotPosition) {
        int robot_pos_x = approxFloat(robotPosition->tra[0] / h_map->cellSize) + (h_map->dimX / 2);
        int robot_pos_y = approxFloat(robotPosition->tra[1] / h_map->cellSize) + (h_map->dimY / 2);

        // opencv inverts row and columns for points
        cv::Point robot_circle_center(robot_pos_y, robot_pos_x);

        Scalar robot_position_color(153, 0, 153);

        circle(data, robot_circle_center, markerRadius, robot_position_color, FILLED);
    }

    return data;
}

Mat CudaGrid3D::getGrid2D(Map *h_map, int freeThreshold, int warningThreshold, int occupiedThreshold) {
    CudaGrid3D::CudaTransform3D tf;
    return getGrid2DTask(h_map, freeThreshold, warningThreshold, occupiedThreshold, false, &tf, 0);
}

Mat CudaGrid3D::getGrid2D(Map *h_map, int freeThreshold, int warningThreshold, int occupiedThreshold, CudaTransform3D *robotPosition, int markerRadius) {
    return getGrid2DTask(h_map, freeThreshold, warningThreshold, occupiedThreshold, true, robotPosition, markerRadius);
}

__global__ void initFrontierToCluster(int *arr, int length) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid < length) {
        arr[tid] = -1;
    }
}

__device__ double euclideanDistance(CudaGrid3D::IntPoint p1, CudaGrid3D::IntPoint p2) {
    int diff_x = p1.x - p2.x;
    int diff_y = p1.y - p2.y;
    int diff_z = p1.z - p2.z;
    return sqrt(pow(diff_x, 2) + pow(diff_y, 2) + pow(diff_z, 2));
}

__global__ void clusterFrontiers3DKernel(char *d_grid_3D, int dimX, int dimY, int dimZ, CudaGrid3D::IntPoint center1, CudaGrid3D::IntPoint center2, int *d_frontierToCluster) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid < dimX * dimY * dimZ) {
        if (d_grid_3D[tid] == FRONTIER_CELL) {
            int z = tid / (dimX * dimY);
            int x = (tid - (z * dimX * dimY)) / dimY;
            int y = tid - (x * dimY + z * dimX * dimY);

            printf("frontier idx %d (%d, %d, %d)\n", tid, x, y, z);

            CudaGrid3D::IntPoint pt;
            pt.x = x;
            pt.y = y;
            pt.z = z;

            double distance1 = euclideanDistance(pt, center1);
            double distance2 = euclideanDistance(pt, center2);

            printf("point n. %d, distance1: %f, distance2: %f\n", tid, distance1, distance2);

            if (distance1 <= distance2) {
                d_frontierToCluster[tid] = 1;
            } else {
                d_frontierToCluster[tid] = 2;
            }
        }
    }
}

void CudaGrid3D::clusterFrontiers3D(Map *h_map) {
    int numCells = h_map->dimX * h_map->dimY * h_map->dimZ;
    int numBlocks = (numCells + 256) / 256;

    srand(time(NULL));

    IntPoint center1;

    center1.x = rand() % h_map->dimX;
    center1.y = rand() % h_map->dimY;
    center1.z = rand() % h_map->dimZ;

    IntPoint center2;

    center2.x = rand() % h_map->dimX;
    center2.y = rand() % h_map->dimY;
    center2.z = rand() % h_map->dimZ;

    printf("centroid 1: (%d, %d, %d)\n", center1.x, center1.y, center1.z);
    printf("centroid 2: (%d, %d, %d)\n", center2.x, center2.y, center2.z);

    int *d_frontierToCluster;

    cudaMalloc((void **)&d_frontierToCluster, numCells * sizeof(int));

    initFrontierToCluster<<<numBlocks, 256>>>(d_frontierToCluster, numCells);

    clusterFrontiers3DKernel<<<numBlocks, 256>>>(h_map->d_grid_3D, h_map->dimX, h_map->dimY, h_map->dimZ, center1, center2, d_frontierToCluster);
    cudaDeviceSynchronize();
}