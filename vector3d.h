#ifndef VECTOR3D_H
#define VECTOR3D_H

class Vector3D {
   public:
    bool *grid;
    int dimX;
    int dimY;
    int dimZ;
};

class Point {
   public:
    float x;
    float y;
    float z;
};

// initialize the 3D Grid on the GPU memory
void initVector(Vector3D *h_vector, int dimX, int dimY, int dimZ);

// get the 3D coordinates with the 1D index and the Grid size
void getCoordinatesInv(int i, int dimX, int dimY, int dimZ);

// calculate the 1D index with the 3D coordinates (x,y,z)
int getLinearIndex(Vector3D *h_vector, int x, int y, int z);

// insert the points of the pointcloud in the 3D Grid
void insertPointCloud(Vector3D *h_vector, Point *pointcloud, int sizePointcloud, float resolution);

// initialize a pointcloud on the GPU memory
void initPointcloud(Point *d_pointcloud, int length);

// generates a pointcloud with n random points
void generateRandomPc(Vector3D *h_vector, Point *d_pointcloud, int sizePointcloud, float resolution);

// checks if the grid is legal
void checkGrid(Vector3D *h_vector);

// checks if there are duplicates in the pointcloud (points with the same x,y,z coordinates)
void checkDuplicates(Vector3D *h_vector, Point *d_pointcloud, int *pointcloudIntIdx, int sizePointcloud);

// print functions
void printGridHost(bool *h_grid, int dimx, int dimy, int dimz);
void printLinearGrid(Vector3D *h_vector);
void printGrid(Vector3D *h_vector);
void printPointcloud(Point *d_pointcloud, int sizePointcloud);

void copyPCToHost(Point *h_pc, Point *d_pc, int lenght);

// functions for 3D mesh generation
void vertex(FILE *file, double x, double y, double z);
void face(FILE *file, int v1, int v2, int v3);
void cubeVertex(FILE *file, double res, double x, double y, double z);
void cubeFace(FILE *file, int nCube);
void generateMesh(Vector3D *h_vector);

// test function (it's better to use the functions above in a cpp file and keep the .cu as simple as possible)
int test(int dimX, int dimY, int dimZ, float resolution, int numPoints, bool memoryDebug);

#endif