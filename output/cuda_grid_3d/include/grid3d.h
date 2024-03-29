#ifndef GRID3D_H
#define GRID3D_H

#include <opencv2/highgui/highgui.hpp>

namespace CudaGrid3D {

/// @brief 3D Point struct with integer coordinates (x,y,z)
class IntPoint {
   public:
    unsigned int x = 0;
    unsigned int y = 0;
    unsigned int z = 0;
};

/// @brief  3D Point struct (x,y,z coordinates)
class Point {
   public:
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/// @brief Map structs that holds the 2D and the 3D Grids and grid parameters
class Map {
   public:
    char *d_grid_2D;
    char *d_grid_2D_detailed;
    char *d_grid_3D;
    int dimX = 0;
    int dimY = 0;
    int dimZ = 0;
    int ox = 0;
    int oy = 0;
    int oz = 0;
    float cellSize = 0.0;
    int floorMargin = 0;
    int robotHeight = 0;
};

/// @brief Struct that contains the translation vector and the rotation matrix
class CudaTransform3D {
   public:
    double tra[3];
    double rot[3][3];
};

class BestObservation {
   public:
    Point point;
    double pitch;
    double yaw;
};

/// @brief Mesh type: OCCUPANCY_MAP contains occupied points, FREE_MAP contains free points
/// and FRONTIER_MAP contains frontier cells
enum MeshType {
    OCCUPANCY_MAP,
    FREE_MAP,
    FRONTIER_MAP
};

/// @brief Initialize the Map struct on the host and the grids (2D and 3D) on the device
/// @param h_map Map struct
/// @param dimX Length of the x-axis of the grid (meters)
/// @param dimY Length of the y-axis of the grid (meters)
/// @param dimZ Length of the z-axis of the grid (meters)
/// @param ox Position of the origin on x (meters)
/// @param oy Position of the origin on y (meters)
/// @param cellSize Voxel edge length (meters)
/// @param floorMargin Bottom margin for 2D projection of the 3D Grid (meters)
/// @param robotHeight Top margin for 2D projection of the 3D Grid (meters)
void initMap(Map *h_map, float dimX, float dimY, float dimZ, float ox, float oy, float oz, float cellSize, float floorMargin, float robotHeight);

/// @brief Free struct (allocated on the host) and the grids (allocated on the device)
/// @param h_map Map struct to be freed
void freeMap(Map *h_map);

/// @brief Get the (x,y) coordinates with the 1D index of the 2D Grid
/// if index is not valid (index >= dimX * dimY) result will be (-1, -1)
/// @param h_map Map struct
/// @param index 1D index
/// @param result (x,y) coordinates
void getCoordinatesInv2D(Map *h_map, int index, int *result);

/// @brief Get the 3D coordinates with the 1D index and the Grid size
/// if index is not valid (index >= dimX * dimY * dimZ) result will be (-1, -1, -1)
/// @param h_map Map struct
/// @param index 1D index
/// @param result (x,y,z) coordinates
void getCoordinatesInv3D(Map *h_map, int index, int *result);

/// @brief Calculate the 1D index of the 2D grid with the 2D coordinates (x,y)
/// @param h_map Map struct
/// @param x x coordinate
/// @param y y coordinate
/// @return 1D index of the (x,y) cell
int getLinearIndex2D(Map *h_map, int x, int y);

/// @brief Calculate the 1D index of the 3D grid with the 3D coordinates (x,y,z)
/// @param h_map Map struct
/// @param x x coordinate
/// @param y y coordinate
/// @param z z coordinate
/// @return 1D index of the (x,y,z) cell
int getLinearIndex3D(Map *h_map, int x, int y, int z);

/// @brief Initialize a pointcloud on the device (GPU).
/// The address of the pointer to the pointcloud must be passed.
/// @param d_pointcloud pointcloud
/// @param length capacity of the pointcloud (max number of points)
void initDevicePointcloud(Point **d_pointcloud, int length);

/// @brief Initialize a pointcloud on the device (GPU) with the points of another pointcloud allocated on the host
/// @param d_pointcloud device pointcloud (destination of the points)
/// @param h_pointcloud host pointcloud (source of the points)
/// @param numPoints number of points to be transferred
void initDevicePointcloud(Point **d_pointcloud, Point *h_pointcloud, int numPoints);

/// @brief Free the pointcloud space allocated in the device memory
/// @param d_pointcloud pointcloud to be freed
void freeDevicePointcloud(Point *d_pointcloud);

/// @brief Generate a pointcloud with n random points
/// @param h_map map
/// @param d_pointcloud pointcloud
/// @param numPoints number of points
void generateRandomPointcloud(Map *h_map, Point **d_pointcloud, int numPoints);

/// @brief Insert the points of the pointcloud in the 3D Grid
/// @param h_map map
/// @param d_pointcloud pointcloud (device)
/// @param numPoints number of points inside the pointcloud
void insertPointcloud(Map *h_map, Point *d_pointcloud, int numPoints);

// check if the grid is legal
// void checkGrid(Map *h_map);

/// @brief Check if there are duplicates in the pointcloud (points with the same x,y,z coordinates)
/// @param h_map map
/// @param d_pointcloud pointcloud (device)
/// @param numPoints number of points
void checkDuplicates(Map *h_map, Point *d_pointcloud, int numPoints);

/// @brief Converts an array of points in the format [point0_x, point0_y, point0_z, point0_w, point1_x, point1_y, point1_z, point1_w, ...]
/// in a device pointcloud
/// @param h_pc_array array of points
/// @param length h_pc_array length (not the number of points)
/// @param d_pointcloud pointcloud (device)
void arrayToPointcloud(float *h_pc_array, int length, Point **d_pointcloud);

/// @brief Converts an array of points in the format [point0_x, point0_y, point0_z, point0_w, point1_x, point1_y, point1_z, point1_w, ...]
/// in a device pointcloud
/// @param h_pc_array array of points
/// @param length h_pc_array length (not the number of points)
/// @param d_pointcloud pointcloud (device)
/// @param tf struct that contains the translation vector and the rotation matrix for rototranslations
void arrayToPointcloud(float *h_pc_array, int length, Point **d_pointcloud, CudaTransform3D tf);

// ----- print functions -----

/// @brief Print a 2D Grid allocated on the host
/// @param h_grid_2D 2D grid
/// @param dimx Number of cells on the x-axis
/// @param dimy Number of cells on the y-axis
void printHostGrid2D(char *h_grid_2D, int dimx, int dimy);

/// @brief Print a 3D Grid allocated on the host
/// @param h_grid_3D 3D grid
/// @param dimx number of cells on the x-axis
/// @param dimy number of cells on the y-axis
/// @param dimz number of cells on the z-axis
void printHostGrid3D(char *h_grid_3D, int dimx, int dimy, int dimz);

/// @brief Print device 2D Grid
/// @param d_grid_2D 2D grid
/// @param dimx number of cells on the x-axis
/// @param dimy number of cells on the y-axis
void printDeviceGrid2D(char *d_grid_2D, int dimx, int dimy);

/// @brief Print device 2D grid
/// @param h_map map
void printDeviceGrid2D(Map *h_map);

/// @brief Print device 3D Grid as many 2D planes
/// @param d_grid_3D 3D grid
/// @param dimx number of cells on the x-axis
/// @param dimy number of cells on the y-axis
/// @param dimz number of cells on the z-axis
void printDeviceGrid3D(char *d_grid_3D, int dimx, int dimy, int dimz);

/// @brief Print device 3D Grid as many 2D planes
/// @param h_map map
void printDeviceGrid3D(Map *h_map);

/// @brief Print the 2D Grid as a 1D vector (as it really is in the memory)
/// @param d_grid_2D 2D grid
/// @param dimx number of cells on the x-axis
/// @param dimy number of cells on the y-axis
void printDeviceLinearGrid2D(char *d_grid_2D, int dimx, int dimy);

/// @brief Print the 2D Grid as a 1D vector (as it really is in the memory)
/// @param h_map map
void printDeviceLinearGrid2D(Map *h_map);

/// @brief Print device 3D Grid on a line (as it really is in the memory)
/// @param d_grid_3D 3D grid
/// @param dimx number of cells on the x-axis
/// @param dimy number of cells on the y-axis
/// @param dimz number of cells on the z-axis
void printDeviceLinearGrid3D(char *d_grid_3D, int dimx, int dimy, int dimz);

/// @brief Print device 3D Grid on a line (as it really is in the memory)
/// @param h_map map
void printDeviceLinearGrid3D(Map *h_map);

/// @brief Print device pointcloud
/// @param d_pointcloud pointcloud (device)
/// @param numPoints number of points in the pointcloud
void printPointcloud(Point *d_pointcloud, int numPoints);

// ------------------------

// functions for 3D mesh generation (.obj)

/// @brief Generate a mesh of the 2D grid
/// @param h_map map
/// @param path save path of the mesh
void generateMesh2D(Map *h_map, const char *path, bool localMesh);

/// @brief Generate a mesh of the 3D grid (a cube for each occupied cell)
/// @param h_map map
/// @param path save path of the mesh
void generateMesh3D(Map *h_map, const char *path, bool localMesh);

/// @brief Generate a simple mesh of the 3D grid (a vertex for each selected cell)
/// @param h_map map
/// @param path save path of the mesh
/// @param isOccupationMesh if true generates a mesh of the occupied space, otherwise a mesh of the free space is created
void generateSimpleMesh3D(Map *h_map, const char *path, MeshType meshType, bool localMesh);

// ------------------------

/// @brief Find the free cells with a ray tracing algorithm.
/// Each ray starts at origin and ends at the first encountered obstacle in its path
/// @param h_map map
/// @param d_pointcloud pointcloud (device)
/// @param numPoints number of point in the pointcloud
/// @param origin coordinates of the camera (origin point of the traced rays)
/// @param freeObstacles mark occupied cells as free if it is traversed by a ray
void pointcloudRayTracing(Map *h_map, Point *d_pointcloud, int numPoints, Point origin, bool freeObstacles);

/// @brief Find frontier points in the 3D grid
/// @param h_map map
void findFrontiers3D(Map *h_map);

/// @brief Update occupied, free and unknown cells of the 2D grid using the data from 3D grid
/// @param h_map map
/// @param freeThreshold max confidence for a free cell
/// @param warningThreshold min confidence for a warning cell
/// @param occupiedThreshold min confidence for an occupied cell
/// @param maxUnknownConfidence the confidence set if every voxel inside the height interval of the robot
/// is unknown
/// @param minOccupiedConfidence the confidence set if only one voxel is found in the height interval of the robot
/// @param inflationRadius obstacles inflation radius
void updateGrid2D(Map *h_map, int freeThreshold, int warningThreshold, int occupiedThreshold, int maxUnknownConfidence, int minOccupiedConfidence, double inflationRadius);

/// @brief Get a binned (reduced in resolution) density 2D grid of the unknown space
/// @param h_map map
/// @param bin_size binning size (bin_size x bin_size)
/// @param freeThreshold max confidence of a free cell
/// @param occupiedThreshold min confidence of an occupied cell
/// @param output_grid_2d_binned result: binned grid
/// @param dimX number of cells on the x-axis in the output binned grid
/// @param dimY number of cells on the y-axis in the output binned grid
void getUnknownDensityGrid2D(Map *h_map, int bin_size, int freeThreshold, int occupiedThreshold, int *&output_grid_2d_binned, int &dimX, int &dimY);

/// @brief returns the cv::Mat of the 2D colored grid
/// @param h_map map
/// @param detailedGrid if true get the grid with obstacle inflation and frontiers
/// @param freeThreshold max confidence for a free pixel
/// @param warningThreshold min confidence for a warning pixel
/// @param occupiedThreshold min confidence for an occupied pixel
/// @return 2D occupancy map
cv::Mat getGrid2D(Map *h_map, bool detailedGrid, int freeThreshold, int warningThreshold, int occupiedThreshold);

/// @brief Get the 2D occupancy map
/// @param h_map map
/// @param detailedGrid if true get the grid with obstacle inflation and frontiers
/// @param freeThreshold max confidence for a free pixel
/// @param warningThreshold min confidence for a warning pixel
/// @param occupiedThreshold min confidence for an occupied pixel
/// @param robotPosition position of the robot (x,y,z coordinates)
/// @param markerRadius radius of the robot position marker
/// @return 2D occupancy map
cv::Mat getGrid2D(Map *h_map, bool detailedGrid, int freeThreshold, int warningThreshold, int occupiedThreshold, CudaTransform3D *robotPosition, int markerRadius);

void getHostGrid(Map *h_map, char **h_grid_2D);
void getHostGridDetailed(Map *h_map, char **h_grid_2D_detailed);

/// @brief Cluster the frontier cells of the 3D Grid
/// @param h_map map
/// @param maxClusterRadiusMeters max distance of a point from its centroid in a cluster
/// @param origin robot position
/// @param bestCentroid centroid in meters of the best cluster (result)
/// @param cluster list of points in the best cluster (result)
/// @param sizeCluster number of points in the best cluster (result)
void clusterFrontiers3D(Map *h_map, double maxClusterRadiusMeters, CudaGrid3D::Point origin, CudaGrid3D::Point *bestCentroid, CudaGrid3D::IntPoint **cluster, int *sizeCluster);

/// @brief Compute the best point in the map to observe the cluster. The points are searched on the circumference on the 2D map
/// with the centre as the projected cluster centroid on the 2D map and with a specified radius
/// @param h_map map
/// @param clusterCenterMeters centroid of the cluster to observe (coordinates in meters)
/// @param cluster cluster to observe
/// @param sizeCluster number of points in the cluster to observe
/// @param radiusMeters distance from the projected cluster centroid on the 2D map where to look for the best observation point
/// @param angleIntervalDeg search for a point every x degrees on the circumference
/// @param freeThreshold max confidence for a free cell
/// @param cameraHeightMeters camera z-position
/// @return best observation point (coordinates in meters)
BestObservation bestObservationPoint(Map *h_map, CudaGrid3D::Point clusterCenterMeters, CudaGrid3D::IntPoint *cluster, int sizeCluster, double clusterDistanceMeters, double angleIntervalDeg, int freeThreshold, double z_min, double z_max, double z_interval);

}  // namespace CudaGrid3D

#endif