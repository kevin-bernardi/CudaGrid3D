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

class Vector3D {
   public:
    bool *grid;
    int dimX;
    int dimY;
    int dimZ;
};

class Point {
   public:
    int x;
    int y;
    int z;
};

__global__ void initGridKernel(bool *grid, int numCells) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid == numCells - 1) {
        printf("initializing vector3D in kernel\n");
    }

    if (tid < numCells) {
        grid[tid] = false;
    }
}

// initialize 3d grid, make every cell false
void initGrid(Vector3D *vector) {
    int numCells = vector->dimX * vector->dimY * vector->dimZ;
    int numBlocks = (numCells + 256) / 256;

    initGridKernel<<<numBlocks, 256>>>(vector->grid, numCells);
    cudaDeviceSynchronize();
}

//__global__ void computeMidPointKernel(bool *grid, int n, int dimX, int dimY,
//                                       int dimZ) {
//     int tid = blockIdx.x * blockDim.x + threadIdx.x;

//     float sumx, sumy, sumz, count;
//     sumx = sumy = sumz = count = 0.0;

//     for (int i = 0; i < n; i++) {
//         if (grid[i] == 1) {
//             // get coordinates (x,y,z) from linear index

//             int z = floor(i / (dimX * dimY));

//             int x = floor((i - (z * dimX * dimY)) / dimY);

//             int y = i - (x * dimY + z * dimX * dimY);

//             sumx += x;
//             sumy += y;
//             sumz += z;
//             count++;
//         }
//     }

// }

void getCoordinatesInv(int i, int dimX, int dimY, int dimZ) {
    int z = floor(i / (dimX * dimY));

    int x = floor((i - (z * dimX * dimY)) / dimY);

    int y = i - (x * dimY + z * dimX * dimY);

    printf("Idx %d -> (%d, %d, %d)\n", i, x, y, z);
}

// input 3 coordinates (x,y,z) and get the linear index
int getLinearIndex(Vector3D *vector, int x, int y, int z) {
    if (x < vector->dimX && y < vector->dimY && z < vector->dimZ) {
        int result = y + x * vector->dimY + z * vector->dimX * vector->dimY;
        return result;

    } else {
        printf("Error! Input coordinates are not valid!\n");
        return -1;
    }
}

__global__ void insertPtKernel(bool *grid, int idx) {
    grid[idx] = true;

    printf("inserted point at idx %d\n", idx);
}

// insert a single point in the 3D grid
void insertPt(Vector3D *vector, int x, int y, int z) {
    int idx = getLinearIndex(vector, x, y, z);
    insertPtKernel<<<1, 1>>>(vector->grid, idx);
    cudaDeviceSynchronize();
}

__global__ void insertPointCloudKernel(bool *grid, Point *pointcloud, int n, int dimX, int dimY, int dimZ) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    // check if point is within the pointcloud vector of lenght n
    if (tid < n) {
        if (tid == 0) printf("Length: %d\n", n);
        Point pt = pointcloud[tid];
        // check if point is within bounds (correct coordinates)
        if (pt.x < dimX && pt.y < dimY && pt.z < dimZ) {
            int idx = pt.y + pt.x * dimY + pt.z * dimX * dimY;
            // printf("Adding Point (%d, %d, %d) at idx %d\n", pt.x, pt.y, pt.z, idx);
            grid[idx] = true;
        } else {
            // point out of bound
            printf("Point (%d, %d, %d) out of bound!\n", pt.x, pt.y, pt.z);
        }
    }
}

// insert a pointcloud (array of points (x,y,z)) in the 3D grid
void insertPointCloud(Vector3D *vector, Point *pointcloud, int sizePointcloud) {
    int numBlocks = (sizePointcloud + 256) / 256;
    printf("Num blocks: %d\n", numBlocks);
    printf("Size of grid: %d x %d x %d\n", vector->dimX, vector->dimY, vector->dimZ);
    printf("Size of pointcloud: %d\n", sizePointcloud);
    insertPointCloudKernel<<<numBlocks, 256>>>(vector->grid, pointcloud, sizePointcloud, vector->dimX, vector->dimY, vector->dimZ);
    cudaDeviceSynchronize();
}

__global__ void generateRandomPcKernel(Point *pointcloud, int n, curandState *state, int dimX, int dimY, int dimZ) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid < n) {
        if (tid == n - 1) {
            printf("generating random pointcloud in kernel\n");
        }

        curand_init(clock(), tid, 0, &state[tid]);

        int x = (int)truncf(curand_uniform(&(state[tid])) * (dimX - 1.0 + 0.999999));
        int y = (int)truncf(curand_uniform(&(state[tid])) * (dimY - 1.0 + 0.999999));
        int z = (int)truncf(curand_uniform(&(state[tid])) * (dimZ - 1.0 + 0.999999));

        Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;

        if (x >= dimX || y >= dimY || z >= dimZ) {
            printf("***************ERROR, the generated point doesn't have valid coordinates!***************\n");
        } else {
            // printf("generated point (%d, %d, %d)\n", pt.x, pt.y, pt.z);
        }

        pointcloud[tid] = pt;
    }
}

// generate a (sizePointcloud) amount of points and put them in the pointcloud
void generateRandomPc(Vector3D *vector, Point *pointcloud, int sizePointcloud, curandState *state) {
    int numBlocks = (sizePointcloud + 256) / 256;
    generateRandomPcKernel<<<numBlocks, 256>>>(pointcloud, sizePointcloud, state, vector->dimX, vector->dimY, vector->dimZ);
    cudaDeviceSynchronize();
}

__global__ void checkGridKernel(bool *grid, int numCells) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid < numCells) {
        if (tid == numCells - 1) printf("checking grid in kernel\n");
        if (grid[tid] != true && grid[tid] != false) {
            printf("Check Failed  ");
        }
    }
}

// check if the 3D grid contains illegal values
void checkGrid(Vector3D *vector) {
    int numCells = vector->dimX * vector->dimY * vector->dimZ;
    int numBlocks = (numCells + 256) / 256;
    checkGridKernel<<<numBlocks, 256>>>(vector->grid, numCells);
    cudaDeviceSynchronize();
}

__global__ void checkDuplicatesKernel(Point *pointcloud, int *pointcloudIntIdx, int numPoints, int dimX, int dimY, int dimZ) {
    for (int i = 0; i < numPoints; i++) {
        Point pt = pointcloud[i];
        int idx = pt.y + pt.x * dimY + pt.z * dimX * dimY;

        pointcloudIntIdx[i] = idx;
    }

    bool duplicates = false;

    for (int i = 0; i < numPoints - 1; i++) {
        for (int j = i + 1; j < numPoints; j++) {
            if (pointcloudIntIdx[j] == pointcloudIntIdx[i]) {
                printf("Found duplicate! Point idx %d\n", pointcloudIntIdx[j]);
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
void checkDuplicates(Vector3D *vector, Point *pointcloud, int *pointcloudIntIdx, int sizePointcloud) {
    checkDuplicatesKernel<<<1, 1>>>(pointcloud, pointcloudIntIdx, sizePointcloud, vector->dimX, vector->dimY, vector->dimZ);
    cudaDeviceSynchronize();
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

__global__ void printLinearGridKernel(bool *grid, int numCells) {
    for (int i = 0; i < numCells; i++) {
        printf("%d | ", grid[i]);
    }

    printf("\n");
}

// print the grid on a line (as it really is in the memory)
void printLinearGrid(Vector3D *vector) {
    int numCells = vector->dimX * vector->dimY * vector->dimZ;
    printLinearGridKernel<<<1, 1>>>(vector->grid, numCells);
    cudaDeviceSynchronize();
}

__global__ void printGridKernel(bool *grid, int dimX, int dimY, int dimZ) {
    if (dimX * dimY * dimZ <= 0) {
        printf("ERROR! Empty matrix\n");
        return;
    }

    printf("\n\n3D Matrix Output (KERNEL)\n");

    for (int z = 0; z < dimZ; z++) {
        printf("Layer %d\n", z);
        for (int x = 0; x < dimX; x++) {
            for (int y = 0; y < dimY; y++) {
                int idx = y + x * dimY + z * dimX * dimY;

                if (y == dimY - 1) {
                    printf("%d", grid[idx]);
                } else {
                    printf("%d | ", grid[idx]);
                }
            }

            printf("\n");
        }
    }
}

// print the grid in many 2D planes (dimZ 2D planes of size dimX x dimY)
void printGrid(Vector3D *vector) {
    printGridKernel<<<1, 1>>>(vector->grid, vector->dimX, vector->dimY, vector->dimZ);
    cudaDeviceSynchronize();
}

__global__ void printPcKernel(Point *pointcloud, int sizePointcloud) {
    for (int i = 0; i < sizePointcloud; i++) {
        Point pt = pointcloud[i];
        printf("Point %d: (%d,%d,%d)\n", i, pt.x, pt.y, pt.z);
    }
}

// print the list of points inside the pointcloud
void printPc(Point *pointcloud, int sizePointcloud) {
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

void generateMesh(Vector3D *vector) {
    double dimX = vector->dimX;
    double dimY = vector->dimY;
    double dimZ = vector->dimZ;

    double resolution = 0.1;

    int numCells = dimX * dimY * dimZ;

    FILE *fptr;

    fptr = fopen("example.obj", "w");

    int nCube = 0;

    for (int i = 0; i < numCells; i++) {
        if (vector->grid[i] == true) {
            double z = floor(i / (dimX * dimY));

            double x = floor((i - (z * dimX * dimY)) / dimY);

            double y = i - (x * dimY + z * dimX * dimY);

            cubeVertex(fptr, resolution, x * resolution, y * resolution, z * resolution);
        }
    }

    for (int i = 0; i < numCells; i++) {
        if (vector->grid[i] == true) {
            cubeFace(fptr, nCube);
            nCube++;
        }
    }

    fclose(fptr);
}

int main() {
    // PARAMETERS

    // size of the 3D Matrix
    int dimX = 200;  // 10cm per unit (20 x 10 x 5 m)
    int dimY = 100;
    int dimZ = 50;

    // number of points to randomly generate for the pointcloud
    int numPointsToGenerate = 100000;

    // END PARAMETERS
    // ---------------------------------------------------------------------------------

    // state for the generation of random numbers in kernel code
    curandState *d_state;

    // allocate the space for state on the gpu
    cudaError_t err = cudaMalloc((void **)&d_state, sizeof(curandState) * numPointsToGenerate);
    printf("cudaMalloc curandState error code: %d\n", err);  // 0: ok | not 0: not ok

    // number of cells contained in the 3D matrix
    int numCells = dimX * dimY * dimZ;

    // for time measurement
    struct timeval start, end;

    // vector on the host (ONLY ON THE HOST, because we can't access variables allocated on the gpu outside of kernel functions)
    Vector3D *h_vector = (Vector3D *)malloc(sizeof(Vector3D));

    // grid (or matrix) on the host
    bool *h_grid = (bool *)malloc(sizeof(bool) * numCells);

    // grid on the device (GPU)
    bool *d_grid;

    // allocate the grid on the device (GPU)
    err = cudaMalloc((void **)&d_grid, numCells * sizeof(bool));
    printf("cudaMalloc grid error code: %d\n", err);  // 0: ok | not 0: not ok

    // initialize vector
    h_vector->dimX = dimX;
    h_vector->dimY = dimY;
    h_vector->dimZ = dimZ;

    // NOTE: the host vector MUST contain the pointer to the grid on the DEVICE
    h_vector->grid = d_grid;

    // start timer
    gettimeofday(&start, NULL);

    // initialize 3d grid
    initGrid(h_vector);

    // check if there are illegal values inside the grid
    checkGrid(h_vector);

    // allocate pointcloud on device
    Point *d_pointcloud;

    err = cudaMalloc((void **)&d_pointcloud, sizeof(Point) * numPointsToGenerate);
    printf("cudaMalloc pointcloud error code: %d\n", err);  // 0: ok | not 0: not ok

    // fill the pointcloud with n (numPointsToGenerate) random points
    generateRandomPc(h_vector, d_pointcloud, numPointsToGenerate, d_state);

    // // check for duplicates
    // int *d_pointCloudIntIdx;
    // err = cudaMalloc((void **)&d_pointCloudIntIdx, sizeof(int) * numPointsToGenerate);
    // printf("cudaMalloc pointCloudIntIdx error code: %d\n", err);  // 0: ok | not 0: not ok
    // checkDuplicates(h_vector, d_pointcloud, d_pointCloudIntIdx, numPointsToGenerate);

    // // print the list of points in the pointcloud
    // printPc(d_pointcloud, numPointsToGenerate);

    // fill the 3d space with the pointcloud points
    insertPointCloud(h_vector, d_pointcloud, numPointsToGenerate);

    // check if the grid contains illegal values (e.g. numbers that are not 0 or 1)
    checkGrid(h_vector);

    cudaMemcpy(h_grid, d_grid, sizeof(bool) * numCells, cudaMemcpyDeviceToHost);

    h_vector->grid = h_grid;

    generateMesh(h_vector);

    // stop the timer
    gettimeofday(&end, NULL);

    // calculate the computation time
    printf("computation took %f ms\n", ((end.tv_sec - start.tv_sec) * 1000000 + end.tv_usec - start.tv_usec) / 1000.0);

    // free gpu memory
    cudaFree(d_pointcloud);
    // cudaFree(d_pointCloudIntIdx);
    cudaFree(d_state);
    cudaFree(d_grid);

    // free heap memory
    free(h_grid);
    free(h_vector);
}