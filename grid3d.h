#ifndef GRID3D_H
#define GRID3D_H

#include <opencv2/highgui/highgui.hpp>

namespace CudaGrid3D {

class Map {
   public:
    char *d_grid_2D;
    char *d_grid_3D;
    int dimX = 0;
    int dimY = 0;
    int dimZ = 0;
    float cellSize = 0.0;
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
void initMap(Map *h_map, int dimX, int dimY, int dimZ, float cellSize, int freeVoxelsMargin, int robotVoxelsHeight);

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
void generateRandomPointcloud(Map *h_map, Point **d_pointcloud, int sizePointcloud);

// checks if the grid is legal
// void checkGrid(Map *h_map);

// checks if there are duplicates in the pointcloud (points with the same x,y,z coordinates)
void checkDuplicates(Map *h_map, Point *d_pointcloud, int sizePointcloud);

// Converts an array of points in the format [point0_x, point0_y, point0_z, point0_w, point1_x, point1_y, point1_z, point1_w, ...]
// in a device pointcloud
void arrayToPointcloud(float *h_array, int length, Point **d_pointcloud, bool enableRototranslation, CudaTransform3D tf);

// ----- print functions -----

// print 2D Grid
void printHostGrid2D(char *h_grid_2D, int dimx, int dimy);

// print 2D Grid with int values
void printHostGrid2D(int *h_grid_2D, int dimx, int dimy);

// print host 3D Grid
void printHostGrid3D(char *h_grid_3D, int dimx, int dimy, int dimz);

// print device 2D Grid
void printDeviceGrid2D(char *d_grid_2D, int dimx, int dimy);

// print device 3D Grid
void printDeviceGrid3D(char *d_grid_3D, int dimx, int dimy, int dimz);

// print the 2D Grid on a line (as it really is in the memory)
void printDeviceLinearGrid2D(char *d_grid_2D, int dimx, int dimy);

// print device 3D Grid on a line (as it really is in the memory)
void printDeviceLinearGrid3D(char *d_grid_3D, int dimx, int dimy, int dimz);

// print Map Grids

// print the 2D Grid
void printMapGrid2D(Map *h_map);

// print device 3D Grid as many 2D planes
void printMapGrid3D(Map *h_map);

// print the 2D Grid on a line (as it really is in the memory)
void printLinearMapGrid2D(Map *h_map);

// print device 3D Grid on a line (as it really is in the memory)
void printLinearMapGrid3D(Map *h_map);

// --------

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
void generateSimpleMesh(Map *h_map, const char *path, bool isOccupationMesh);

// ------------------------

// ray tracing to find free cells
// rays starts at origin and ends at the first encountered obstacle in its own direction
void pointcloudRayTracing(Map *h_map, Point *pointcloud, int sizePointcloud, Point origin, bool freeObstacles);

// update occupied, free and unknown cells of the 2D grid using the data from 3D grid
// maxUnknownConfidence is the confidence set if every voxel inside the height interval of the robot
// is unknown
// minOccupiedConfidence is the confidence set if only one voxel is found in the height interval of the robot
void updateGrid2D(Map *h_map, int freeThreshold, int maxUnknownConfidence, int minOccupiedConfidence);

// grid 2d binning: used to calculate the density of unknown cells
void getUnknownDensityGrid2D(Map *h_map, int bin_size, int freeThreshold, int occupiedThreshold, int *&output_grid_2d_binned, int &dimX, int &dimY);

// returns the cv::Mat of the 2D colored grid
cv::Mat getGrid2D(Map *h_map, int freeThreshold, int warningThreshold, int occupiedThreshold, CudaTransform3D *robotPosition);

}  // namespace CudaGrid3D

#endif