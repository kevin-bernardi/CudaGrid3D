#ifndef VECTOR3D_H
#define VECTOR3D_H

// #include <opencv2/core/mat.hpp>

// #include "star/robotics/math/Transform3D.hpp"

class Vector3D {
   public:
    char *d_grid;
    int dimX = 0;
    int dimY = 0;
    int dimZ = 0;
    float resolution = 0.0;
};

class Point {
   public:
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
};

class Coord3D {
   public:
    int x;
    int y;
    int z;
};

class CudaTransform3D {
   public:
    float tra[3];
    float rot[3][3];
};

// initialize the 3D Grid on the device
void initVector(Vector3D *h_vector, int dimX, int dimY, int dimZ, float resolution);

// free vector (allocated on the host) and grid (allocated on the device) space
void freeVector(Vector3D *h_vector);

// get the 3D coordinates with the 1D index and the Grid size
// if index is not valid the result will be (-1, -1, -1)
// result is a pointer to a pointer because the allocation of the required space
// for result is done in inside the function
void getCoordinatesInv(Vector3D *h_vector, int index, int **result);

// calculate the 1D index with the 3D coordinates (x,y,z)
int getLinearIndex(Vector3D *h_vector, int x, int y, int z);

// insert the points of the pointcloud in the 3D Grid
void insertPointcloud(Vector3D *h_vector, Point *d_pointcloud, int sizePointcloud);

// initialize a pointcloud on the device (GPU)
// the address of the pointer to the pointcloud must be passed because the allocation of the pointcloud
// is done on the device through a kernel function
void initDevicePointcloud(Point **d_pointcloud, int length);

// initialize a pointcloud on the device (GPU) with the points of another pointcloud allocated on the host
// the address of the pointer to the pointcloud must be passed because the allocation of the pointcloud
// is done on the device through a kernel function
void initDevicePointcloud(Point **d_pointcloud, Point *h_pointcloud, int length);

// free allocated space by pointcloud on the device
void freeDevicePointcloud(Point *d_pointcloud);

// generates a pointcloud with n random points
void generateRandomPointcloud(Vector3D *h_vector, Point *d_pointcloud, int sizePointcloud);

// checks if the grid is legal
// void checkGrid(Vector3D *h_vector);

// checks if there are duplicates in the pointcloud (points with the same x,y,z coordinates)
void checkDuplicates(Vector3D *h_vector, Point *d_pointcloud, int sizePointcloud);

// Insert the points found in the cv:Mat matrix acquired by the zed camera.
// This function accepts a converted
void insertCvMatToPointcloud(float *h_array, int length, Point **d_pointcloud, CudaTransform3D tf);

// print functions
void printGridHost(bool *h_grid, int dimx, int dimy, int dimz);
void printLinearGrid(Vector3D *h_vector);
void printGrid(Vector3D *h_vector);
void printPointcloud(Point *d_pointcloud, int sizePointcloud);

// functions for 3D mesh generation (.obj)
void vertex(FILE *file, float x, float y, float z);
void face(FILE *file, int v1, int v2, int v3);
void cubeVertex(FILE *file, float res, float x, float y, float z);
void cubeFace(FILE *file, int nCube);
void generateMesh(Vector3D *h_vector, const char *path);
void generateSimpleMesh(Vector3D *h_vector, const char *path);

// ray tracing
void pointcloudRayTracing(Vector3D *h_vector, Point *pointcloud, int sizePointcloud, Point origin);

// test function (it's better to use the functions above in a cpp file and keep the .cu as simple as possible)
int test(int dimX, int dimY, int dimZ, float resolution, int numPoints);

#endif