#ifndef GRID3D_H
#define GRID3D_H

// #include <opencv2/core/mat.hpp>
// #include "star/robotics/math/Transform3D.hpp"

namespace CudaGrid3D {

class Map {
   public:
    char *d_grid_3D;
    char *d_grid_2D;
    int dimX = 0;
    int dimY = 0;
    int dimZ = 0;
    float resolution = 0.0;
    int freeVoxelsMargin = 0;
    int robotVoxelsHeight = 0;
};

class Point {
   public:
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

class CudaTransform3D {
   public:
    double tra[3];
    double rot[3][3];
};

// initialize the 3D Grid on the device
void initMap(Map *h_map, int dimX, int dimY, int dimZ, float resolution, int freeVoxelsMargin, int robotVoxelsHeight);

// free vector (allocated on the host) and grid (allocated on the device) space
void freeMap(Map *h_map);

void getCoordinatesInv2D(Map *h_map, int index, int *result);

// get the 3D coordinates with the 1D index and the Grid size
// if index is not valid the result will be (-1, -1, -1)
// result contains the 3 coordinates (x,y,z)
void getCoordinatesInv3D(Map *h_map, int index, int *result);

// calculate the 1D index of the 2D grid with the 2D coordinates (x,y)
int getLinearIndex2D(Map *h_map, int x, int y);

// calculate the 1D index of the 3D grid with the 3D coordinates (x,y,z)
int getLinearIndex3D(Map *h_map, int x, int y, int z);

// insert the points of the pointcloud in the 3D Grid
void insertPointcloud(Map *h_map, Point *d_pointcloud, int sizePointcloud);

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
void generateRandomPointcloud(Map *h_map, Point *d_pointcloud, int sizePointcloud);

// checks if the grid is legal
// void checkGrid(Map *h_map);

// checks if there are duplicates in the pointcloud (points with the same x,y,z coordinates)
void checkDuplicates(Map *h_map, Point *d_pointcloud, int sizePointcloud);

// Insert the points found in the cv:Mat matrix acquired by the zed camera.
// This function accepts a converted
void cvMatToPointcloud(float *h_array, int length, Point **d_pointcloud, CudaTransform3D tf);

// ----- print functions -----

// print 2D Grid
void printGrid2DHost(char *h_grid_2D, int dimx, int dimy);

// print the 2D Grid on a line (as it really is in the memory)
void printLinearGrid2D(Map *h_map);

// print the 2D Grid
void printGrid2D(Map *h_map);

// print host 3D Grid
void printGrid3DHost(char *h_grid_3D, int dimx, int dimy, int dimz);

// print device 3D Grid on a line (as it really is in the memory)
void printLinearGrid3D(Map *h_map);

// print device 3D Grid as many 2D planes
void printGrid3D(Map *h_map);

// print device pointcloud
void printPointcloud(Point *d_pointcloud, int sizePointcloud);

// ------------------------

// functions for 3D mesh generation (.obj)
// void vertex(FILE *file, float x, float y, float z);
// void face(FILE *file, int v1, int v2, int v3);
// void cubeVertex(FILE *file, float res, float x, float y, float z);
// void cubeFace(FILE *file, int nCube);

void generateMeshGrid2D(Map *h_map, const char *path);
void generateMesh(Map *h_map, const char *path);
void generateSimpleMesh(Map *h_map, const char *path);

// ------------------------

// ray tracing to find free cells
// rays starts at origin and ends at the first encountered obstacle in its own direction
void pointcloudRayTracing(Map *h_map, Point *pointcloud, int sizePointcloud, Point origin);

// update occupied, free and unknown cells of the 2D grid using the data from 3D grid
// maxUnknownConfidence is the confidence set if every voxel inside the height interval of the robot
// is unknown
// minOccupiedConfidence is the confidence set if only one voxel is found in the height interval of the robot
void updateGrid2D(Map *h_map, int maxUnknownConfidence, int minOccupiedConfidence);

// TODO:
// grid 2s binning
void gridBinning(Map *h_map, int bin_size, int freeThreshold, int occupiedThreshold);

// create an image based on the data from the 2D Grid, save it at the specified path
// if show is true also visualize the image
void visualizeAndSaveGrid2D(Map *h_map, const char *path, bool show, int freeThreshold, int warningThreshold, int occupiedThreshold);

// test function (it's better to use the functions above in a cpp file and keep the .cu as simple as possible)
int test(int dimX, int dimY, int dimZ, float resolution, int numPoints);

}  // namespace CudaGrid3D

#endif