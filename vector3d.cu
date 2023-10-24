/*

BASIC GUIDE

host variable/code/function... -> variable/code/function... allocated on CPU + RAM
device variable/code/function... -> variable/code/function... allocated on GPU + VRAM

d_variable -> variable allocated on the device (it is not mandatory, just to remember where a variable is allocated)
h_variable -> variable allocated on the host

kernel functions (with the __global__ void before the name of the function) are not commented because are called by wrapper host functions (which are commented) to simplify
the signature of the function calls inside the main()

*/

#include <assert.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

#include <opencv2/core/cuda.hpp>

#include "vector3d.h"

__global__ void initGridKernel(bool *d_grid, int numCells) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    // if (tid == numCells - 1) {
    //     printf("initializing vector3D in kernel\n");
    // }

    if (tid < numCells) {
        d_grid[tid] = false;
    }
}

// initialize 3d grid, make every cell false
void initVector(Vector3D *h_vector, int dimX, int dimY, int dimZ, float resolution) {
    h_vector->dimX = dimX;
    h_vector->dimY = dimY;
    h_vector->dimZ = dimZ;
    h_vector->resolution = resolution;

    int numCells = h_vector->dimX * h_vector->dimY * h_vector->dimZ;
    int numBlocks = (numCells + 256) / 256;

    // grid on the device (GPU)
    bool *d_grid;

    // allocate the grid on the device (GPU)
    cudaMalloc((void **)&d_grid, numCells * sizeof(bool));

    h_vector->d_grid = d_grid;

    initGridKernel<<<numBlocks, 256>>>(h_vector->d_grid, numCells);
    cudaDeviceSynchronize();
}

// free vector space
void freeVector(Vector3D *h_vector) {
    cudaFree(h_vector->d_grid);
    free(h_vector);
}

void getCoordinatesInv(Vector3D *h_vector, int index, int **result) {
    *result = (int *)malloc(sizeof(int) * 3);

    int dimX = h_vector->dimX;
    int dimY = h_vector->dimY;
    int dimZ = h_vector->dimZ;

    if (index >= dimX * dimY * dimZ) {
        printf("getCoordinatesInv() ERROR! Index out of bounds!\n");
        (*result)[0] = -1;
        (*result)[1] = -1;
        (*result)[2] = -1;
        return;
    }

    int z = floor(index / (dimX * dimY));

    int x = floor((index - (z * dimX * dimY)) / dimY);

    int y = index - (x * dimY + z * dimX * dimY);

    (*result)[0] = x;
    (*result)[1] = y;
    (*result)[2] = z;

    return;
}

// input 3 coordinates (x,y,z) and get the linear index
int getLinearIndex(Vector3D *h_vector, int x, int y, int z) {
    if (x < h_vector->dimX && y < h_vector->dimY && z < h_vector->dimZ) {
        int result = y + x * h_vector->dimY + z * h_vector->dimX * h_vector->dimY;
        return result;

    } else {
        printf("Error! Input coordinates are not valid!\n");
        return -1;
    }
}

__global__ void insertPointcloudKernel(bool *d_grid, Point *pointcloud, int n, int dimX, int dimY, int dimZ, float resolution) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    // check if point is within the pointcloud vector of lenght n
    if (tid < n) {
        Point pt = pointcloud[tid];
        // check if point is within bounds (correct coordinates)

        int x = floor(pt.x / resolution);
        int y = floor(pt.y / resolution);
        int z = floor(pt.z / resolution);

        if (pt.x < float(dimX) && pt.y < float(dimY) && pt.z < float(dimZ)) {
            int idx = y + x * dimY + z * dimX * dimY;

            // printf("Adding Point (%f, %f, %f)\n", pt.x, pt.y, pt.z);
            // printf("The point is floored to (%d, %d, %d) and added at idx %d\n", x, y, z, idx);
            d_grid[idx] = true;
        } else {
            // point out of bound
            printf("Point (%f, %f, %f) out of bound!\n", pt.x, pt.y, pt.z);
        }
    }
}

// insert a pointcloud (array of points (x,y,z)) in the 3D grid
void insertPointcloud(Vector3D *h_vector, Point *d_pointcloud, int sizePointcloud) {
    if (sizePointcloud <= 0) {
        printf("insertPointcloud() ERROR! sizePointcloud is not valid!\n");
        return;
    }

    int numBlocks = (sizePointcloud + 256) / 256;
    // printf("Size of pointcloud: %d\n", sizePointcloud);

    insertPointcloudKernel<<<numBlocks, 256>>>(h_vector->d_grid, d_pointcloud, sizePointcloud, h_vector->dimX, h_vector->dimY, h_vector->dimZ, h_vector->resolution);
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

    if (tid < n) {
        if (tid == n - 1) {
            printf("Generating random pointcloud in kernel\n");
        }

        curand_init(clock(), tid, 0, &state[tid]);

        float x = curand_uniform(&(state[tid])) * (dimX * resolution - 1.0 + 0.999999);
        float y = curand_uniform(&(state[tid])) * (dimY * resolution - 1.0 + 0.999999);
        float z = curand_uniform(&(state[tid])) * (dimZ * resolution - 1.0 + 0.999999);

        Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;

        if (x >= dimX || y >= dimY || z >= dimZ) {
            printf("***************ERROR, the generated point doesn't have valid coordinates!***************\n");
        } else {
            // printf("generated point (%f, %f, %f)\n", pt.x, pt.y, pt.z);
            pointcloud[tid] = pt;
        }
    }
}

void generateRandomPointcloud(Vector3D *h_vector, Point *pointcloud, int sizePointcloud) {
    int numBlocks = (sizePointcloud + 256) / 256;

    // state for the generation of random numbers in kernel code
    curandState *d_state;

    // allocate the space for state on the gpu
    cudaMalloc((void **)&d_state, sizeof(curandState) * sizePointcloud);

    generateRandomPcKernel<<<numBlocks, 256>>>(pointcloud, sizePointcloud, d_state, h_vector->dimX, h_vector->dimY, h_vector->dimZ, h_vector->resolution);

    cudaFree(d_state);

    cudaDeviceSynchronize();
}

__global__ void checkGridKernel(bool *d_grid, int numCells) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid < numCells) {
        if (tid == numCells - 1) printf("checking grid in kernel\n");
        if (d_grid[tid] != true && d_grid[tid] != false) {
            printf("Check Failed  ");
        }
    }
}

// check if the 3D grid contains illegal values
void checkGrid(Vector3D *h_vector) {
    int numCells = h_vector->dimX * h_vector->dimY * h_vector->dimZ;
    int numBlocks = (numCells + 256) / 256;
    checkGridKernel<<<numBlocks, 256>>>(h_vector->d_grid, numCells);
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
void checkDuplicates(Vector3D *h_vector, Point *pointcloud, int sizePointcloud) {
    int *d_pointcloudIntIdx;

    cudaMalloc((void **)&d_pointcloudIntIdx, sizeof(int) * sizePointcloud);
    checkDuplicatesKernel<<<1, 1>>>(pointcloud, d_pointcloudIntIdx, sizePointcloud, h_vector->dimX, h_vector->dimY, h_vector->dimZ, h_vector->resolution);
    cudaDeviceSynchronize();
    cudaFree(d_pointcloudIntIdx);
}

// same as printGrid but for grids allocated on the host
void printGridHost(bool *h_grid, int dimx, int dimy, int dimz) {
    if (dimx * dimy * dimz <= 0) {
        printf("ERROR! Empty matrix\n");
        return;
    }

    printf("\n\n3D Matrix Output\n");

    for (int z = 0; z < dimz; z++) {
        printf("Layer %d\n", z);
        for (int x = 0; x < dimx; x++) {
            for (int y = 0; y < dimy; y++) {
                int idx = y + x * dimy + z * dimx * dimy;

                if (y == dimy - 1) {
                    printf("%d", h_grid[idx]);
                } else {
                    printf("%d | ", h_grid[idx]);
                }
            }

            printf("\n");
        }
    }
}

__global__ void printLinearGridKernel(bool *d_grid, int numCells) {
    printf("\n\nLinear Matrix Output (DEVICE)\n");

    for (int i = 0; i < numCells; i++) {
        printf("%d | ", d_grid[i]);
    }

    printf("\n");
}

// print the grid on a line (as it really is in the memory)
void printLinearGrid(Vector3D *h_vector) {
    int numCells = h_vector->dimX * h_vector->dimY * h_vector->dimZ;

    if (numCells <= 0) {
        printf("Invalid Vector Size! (num. cells: %d)\n", numCells);
    }

    printLinearGridKernel<<<1, 1>>>(h_vector->d_grid, numCells);
    cudaDeviceSynchronize();
}

__global__ void printGridKernel(bool *d_grid, int dimX, int dimY, int dimZ) {
    if (dimX * dimY * dimZ <= 0) {
        printf("ERROR! Empty matrix\n");
        return;
    }

    printf("\n\n3D Matrix Output (DEVICE)\n");

    for (int z = 0; z < dimZ; z++) {
        printf("Layer %d\n", z);
        for (int x = 0; x < dimX; x++) {
            for (int y = 0; y < dimY; y++) {
                int idx = y + x * dimY + z * dimX * dimY;

                if (y == dimY - 1) {
                    printf("%d", d_grid[idx]);
                } else {
                    printf("%d | ", d_grid[idx]);
                }
            }

            printf("\n");
        }
    }
}

// print the grid in many 2D planes (dimZ 2D planes of size dimX x dimY)
void printGrid(Vector3D *h_vector) {
    printGridKernel<<<1, 1>>>(h_vector->d_grid, h_vector->dimX, h_vector->dimY, h_vector->dimZ);
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

void vertex(FILE *file, double x, double y, double z) {
    fprintf(file, "v %f %f %f\n", x, y, z);
}

void face(FILE *file, int v1, int v2, int v3) {
    fprintf(file, "f %d %d %d\n", v1, v2, v3);
}

void cubeVertex(FILE *file, double res, double x, double y, double z) {
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

void generateMesh(Vector3D *h_vector, const char *path) {
    double dimX = h_vector->dimX;
    double dimY = h_vector->dimY;
    double dimZ = h_vector->dimZ;
    double resolution = h_vector->resolution;

    int numCells = dimX * dimY * dimZ;

    bool *h_grid = (bool *)malloc(sizeof(bool) * numCells);

    cudaMemcpy(h_grid, h_vector->d_grid, sizeof(bool) * numCells, cudaMemcpyDeviceToHost);

    FILE *fptr;

    fptr = fopen(path, "w");

    int nCube = 0;

    for (int i = 0; i < numCells; i++) {
        if (h_grid[i] == true) {
            double z = floor(i / (dimX * dimY));

            double x = floor((i - (z * dimX * dimY)) / dimY);

            double y = i - (x * dimY + z * dimX * dimY);

            cubeVertex(fptr, resolution, x * resolution, y * resolution, z * resolution);
        }
    }

    for (int i = 0; i < numCells; i++) {
        if (h_grid[i] == true) {
            cubeFace(fptr, nCube);
            nCube++;
        }
    }

    fclose(fptr);

    free(h_grid);
}

__global__ void arrToPointcloudKernel(Point *d_pointcloud, float *d_arr, int length) {
    int tid = (blockIdx.x * blockDim.x + threadIdx.x) * 4;

    if (tid < length) {
        float x, y, z;

        x = d_arr[tid];
        y = d_arr[tid + 1];
        z = d_arr[tid + 2];

        Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;

        d_pointcloud[tid / 4] = pt;
    }
}

void insertCvMatToPointcloud(cv::Mat *h_cv_mat, Point **d_pointcloud, Transform3D tf) {
    // convert cv::Mat to classic C array
    float *mat_arr = h_cv_mat->isContinuous() ? (float *)h_cv_mat->data : (float *)h_cv_mat->clone().data;
    uint length = h_cv_mat->total() * h_cv_mat->channels();

    printf("Converted array size: %d\n", length);

    for (int i = 0; i < length; i++) {
        printf("%f |", mat_arr[i]);
    }
    printf("\n");

    float *d_arr;

    cudaMalloc(&d_arr, sizeof(float) * length);
    cudaMemcpy(d_arr, mat_arr, sizeof(float) * length, cudaMemcpyHostToDevice);

    int numPoints = length / 4;

    cudaMalloc(d_pointcloud, sizeof(Point) * numPoints);

    initDevicePointcloud(d_pointcloud, numPoints);

    int numBlocks = (numPoints + 256) / 256;

    arrToPointcloudKernel<<<numBlocks, 256>>>(*d_pointcloud, d_arr, length);
}

int test(int dimx, int dimy, int dimz, float res, int numPoints) {
    // PARAMETERS

    bool enableCheckDuplicates = false;

    // size of the 3D Matrix
    int dimX = dimx;  // 10cm per unit (20 x 10 x 5 m) (200u x 100u x 50u)
    int dimY = dimy;
    int dimZ = dimz;

    float resolution = res;  // length of the edges of a cell (in meters)

    // number of points to randomly generate for the pointcloud
    int numPointsToGenerate = numPoints;

    // END PARAMETERS
    // ---------------------------------------------------------------------------------

    // number of cells contained in the 3D matrix
    int numCells = dimX * dimY * dimZ;

    // for time measurement
    struct timeval start, end;

    // vector on the host (ONLY ON THE HOST, because we can't access variables allocated on the gpu outside of kernel functions)
    Vector3D *h_vector = (Vector3D *)malloc(sizeof(Vector3D));

    // grid (or matrix) on the host
    bool *h_grid = (bool *)malloc(sizeof(bool) * numCells);

    // start timer
    gettimeofday(&start, NULL);

    // initialize 3d grid
    initVector(h_vector, dimX, dimY, dimZ, resolution);

    printf("Size of grid (cells): %d x %d x %d\n", h_vector->dimX, h_vector->dimY, h_vector->dimZ);
    printf("Size of grid: %f m x %f m x %f m\n", h_vector->dimX * resolution, h_vector->dimY * resolution, h_vector->dimZ * resolution);

    // check if there are illegal values inside the grid
    checkGrid(h_vector);

    // allocate pointcloud on device
    Point *d_pointcloud;

    initDevicePointcloud(&d_pointcloud, numPointsToGenerate);

    // fill the pointcloud with n (numPointsToGenerate) random points
    generateRandomPointcloud(h_vector, d_pointcloud, numPointsToGenerate);

    Point *h_pointcloud = (Point *)malloc(sizeof(Point) * numPointsToGenerate);

    // check for duplicates
    if (enableCheckDuplicates) {
        checkDuplicates(h_vector, d_pointcloud, numPointsToGenerate);
    }

    // // print the list of points in the pointcloud
    printPointcloud(d_pointcloud, numPointsToGenerate);

    // fill the 3d space with the pointcloud points
    insertPointcloud(h_vector, d_pointcloud, numPointsToGenerate);

    // check if the grid contains illegal values (e.g. numbers that are not 0 or 1)
    checkGrid(h_vector);

    printGrid(h_vector);

    generateMesh(h_vector, "test.obj");

    // stop the timer
    gettimeofday(&end, NULL);

    // calculate the computation time
    printf("computation took %f ms\n", ((end.tv_sec - start.tv_sec) * 1000000 + end.tv_usec - start.tv_usec) / 1000.0);

    freeDevicePointcloud(d_pointcloud);
    freeVector(h_vector);

    // free heap memory
    free(h_grid);
    free(h_pointcloud);

    return 0;
}