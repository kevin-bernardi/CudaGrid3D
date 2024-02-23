# Functions Reference

- [Functions Reference](#functions-reference)
  - [initMap](#initmap)
  - [freeMap](#freemap)
  - [initDevicePointcloud](#initdevicepointcloud)
  - [freeDevicePointcloud](#freedevicepointcloud)
  - [insertPointcloud](#insertpointcloud)
  - [arrayToPointcloud](#arraytopointcloud)
  - [printPointcloud](#printpointcloud)
  - [generateMesh2D](#generatemesh2d)
  - [generateMesh3D](#generatemesh3d)
  - [generateSimpleMesh3D](#generatesimplemesh3d)
  - [pointcloudRayTracing](#pointcloudraytracing)
  - [findFrontiers3D](#findfrontiers3d)
  - [updateGrid2D](#updategrid2d)
  - [getUnknownDensityGrid2D](#getunknowndensitygrid2d)
  - [getGrid2D](#getgrid2d)
  - [clusterFrontiers3D](#clusterfrontiers3d)
  - [inflateObstacles2D](#inflateobstacles2d)
  - [bestObservationPoint2D](#bestobservationpoint2d)


## initMap

`void initMap(Map *h_map, float dimX, float dimY, float dimZ, float ox, float oy, float cellSize, float floorMargin, float robotHeight)`

Initialize the Map on the host and the grids (both 2D and 3D) on the device.

| Parameter   | Description                                              |
| ----------- | -------------------------------------------------------- |
| h_map       | Map struct                                               |
| dimX        | Length of the x-axis of the grid (meters)                |
| dimY        | Length of the y-axis of the grid (meters)                |
| dimZ        | Length of the z-axis of the grid (meters)                |
| ox          | Position of the origin on the x-axis (meters)            |
| oy          | Position of the origin on the y-axis (meters)            |
| cellSize    | Edge length of each cell (meters)                        |
| floorMargin | Floor margin to not consider in 2D grid updates (meters) |
| robotHeight | Height of the robot (meters)                             |

## freeMap

`void freeMap(Map *h_map)`

Free the map struct (allocated on the host) and the grids (allocated on the device).

| Parameter | Description |
| --------- | ----------- |
| h_map     | Map struct  |

## initDevicePointcloud

`void initDevicePointcloud(Point **d_pointcloud, int numPoints)`

Initialize a pointcloud in the device memory.

| Parameter    | Description                                     |
| ------------ | ----------------------------------------------- |
| d_pointcloud | Pointcloud pointer address (pointer to pointer) |
| numPoints    | Max number of points the pointcloud can hold    |

`initDevicePointcloud(Point **d_pointcloud, Point *h_pointcloud, int numPoints)`

Initialize a pointcloud in the device memory and copy the host pointcloud in the device pointcloud just allocated.

| Parameter    | Description                                     |
| ------------ | ----------------------------------------------- |
| d_pointcloud | Pointcloud pointer address (pointer to pointer) |
| h_pointcloud | Host pointcloud                                 |
| numPoints    | Number of points in the pointcloud              |

## freeDevicePointcloud

`void freeDevicePointcloud(Point *d_pointcloud);`

Free the pointcloud space allocated in the device memory.

| Parameter    | Description |
| ------------ | ----------- |
| d_pointcloud | Pointcloud  |

## insertPointcloud

`void insertPointcloud(Map *h_map, Point *d_pointcloud, int numPoints)`

Inserts the points of the pointcloud in the 3D grid.

| Parameter    | Description                        |
| ------------ | ---------------------------------- |
| h_map        | Map struct                         |
| d_pointcloud | Pointcloud allocated on the device |
| numPoints    | Number of points in the pointcloud |

## arrayToPointcloud

`void arrayToPointcloud(float *h_pc_array, int length, Point **d_pointcloud)`

Converts an array of points in the format [point0_x, point0_y, point0_z, point0_w, point1_x, point1_y, point1_z, point1_w, ...] in a pointcloud already allocated in the device memory. Rototranslation disabled.

| Parameter    | Description                                 |
| ------------ | ------------------------------------------- |
| h_pc_array   | Array of points [x1, y1, z1, w1, x2, y2...] |
| length       | Pointcloud allocated on the device          |
| d_pointcloud | Number of points in the pointcloud          |

`void arrayToPointcloud(float *h_pc_array, int length, Point **d_pointcloud, CudaTransform3D tf)`

Converts an array of points in the format [point0_x, point0_y, point0_z, point0_w, point1_x, point1_y, point1_z, point1_w, ...] in a pointcloud already allocated in the device memory. Rototranslation enabled based on the CudaTransform3D object.

| Parameter    | Description                                                        |
| ------------ | ------------------------------------------------------------------ |
| h_pc_array   | Array of points [x1, y1, z1, w1, x2, y2...]                        |
| length       | Pointcloud allocated on the device                                 |
| d_pointcloud | Number of points in the pointcloud                                 |
| tf           | CudaTransform3D object with translation vector and rotation matrix |

## printPointcloud

`void printPointcloud(Point *d_pointcloud, int numPoints)`

Print the device pointcloud.

| Parameter    | Description                        |
| ------------ | ---------------------------------- |
| d_pointcloud | Pointcloud allocated on the device |
| numPoints    | Number of points in the pointcloud |

## generateMesh2D

`void generateMesh2D(Map *h_map, const char *path)`

Generate a mesh of the 2D occupancy grid.

| Parameter | Description                     |
| --------- | ------------------------------- |
| h_map     | Map struct                      |
| path      | Save path of the generated mesh |

## generateMesh3D

`void generateMesh3D(Map *h_map, const char *path)`

Generate a mesh of the 3D grid.

| Parameter | Description                     |
| --------- | ------------------------------- |
| h_map     | Map struct                      |
| path      | Save path of the generated mesh |

## generateSimpleMesh3D

`void generateSimpleMesh(Map *h_map, const char *path, MeshType meshType)`

Generate a simple mesh of the 3D grid. If `meshType` is set to `OCCUPANCY_MAP` a vertex is inserted in the mesh for each occupied cell. If `meshType` is set to `FREE_MAP` there will be a vertex for each free cell and if `meshType` is set to `FRONTIER_MAP` there will be a vertex for each frontier cell.

| Parameter | Description                                                                           |
| --------- | ------------------------------------------------------------------------------------- |
| h_map     | Map struct                                                                            |
| path      | Save path of the generated mesh                                                       |
| meshType  | Mesh type to be created. Possible values: `OCCUPANCY_MAP`, `FREE_MAP`, `FRONTIER_MAP` |

## pointcloudRayTracing

`void pointcloudRayTracing(Map *h_map, Point *d_pointcloud, int numPoints, Point origin, bool freeObstacles)`

Run the ray tracing algorithm to find free cells in the 3D grid.

| Parameter     | Description                                      |
| ------------- | ------------------------------------------------ |
| h_map         | Map struct                                       |
| d_pointcloud  | Pointcloud allocated on the device               |
| numPoints     | Number of points in the pointcloud               |
| origin        | Origin point (camera coordinates)                |
| freeObstacles | If true the occupied cells can be marked as free |

## findFrontiers3D

`void findFrontiers3D(Map *h_map)`

Find the frontier cells in the 3D grid. A frontier cell is a free voxel in the 3D grid with at least one unknown neighbour on the same height.

| Parameter | Description |
| --------- | ----------- |
| h_map     | Map struct  |


## updateGrid2D

`void updateGrid2D(Map *h_map, int freeThreshold, int maxUnknownConfidence, int minOccupiedConfidence)`

Update the 2D grid using the data contained in the 3D grid (projection).

| Parameter             | Description                                             |
| --------------------- | ------------------------------------------------------- |
| h_map                 | Map struct                                              |
| freeThreshold         | Max confidence to mark a cell as free                   |
| maxUnknownConfidence  | Max confidence if every cell in the column is unknown   |
| minOccupiedConfidence | Min confidence if only 1 cell is occupied in the column |


## getUnknownDensityGrid2D

`void getUnknownDensityGrid2D(Map *h_map, int bin_size, int freeThreshold, int occupiedThreshold, int *&output_grid_2d_binned, int &dimX, int &dimY)`

Get a binned (reduced in resolution) density 2D grid of the unknown space

| Parameter             | Description                        |
| --------------------- | ---------------------------------- |
| h_map                 | Map struct                         |
| bin_size              | Binning size (bin_size x bin_size) |
| freeThreshold         | Max confidence of a free cell      |
| occupiedThreshold     | Min confidence of an occupied cell |
| output_grid_2d_binned | The output binned grid             |
| dimX                  | dimX of the output density grid    |
| dimY                  | dimY of the output density grid    |

## getGrid2D

`cv::Mat getGrid2D(Map *h_map, int freeThreshold, int warningThreshold, int occupiedThreshold)`

Get a displayable 2D occupancy grid

| Parameter         | Description                          |
| ----------------- | ------------------------------------ |
| h_map             | Map struct                           |
| freeThreshold     | Max confidence for a free pixel      |
| warningThreshold  | Min confidence for a warning pixel   |
| occupiedThreshold | Min confidence for an occupied pixel |

`cv::Mat getGrid2D(Map *h_map, int freeThreshold, int warningThreshold, int occupiedThreshold, CudaTransform3D *robotPosition, int markerRadius)`

Get a displayable 2D occupancy grid with an indicator of the robot position

| Parameter         | Description                               |
| ----------------- | ----------------------------------------- |
| h_map             | Map struct                                |
| freeThreshold     | Max confidence for a free pixel           |
| warningThreshold  | Min confidence for a warning pixel        |
| occupiedThreshold | Min confidence for an occupied pixel      |
| robotPosition     | Position of the robot (x,y,z coordinates) |
| markerRadius      | Radius of the robot position marker       |

## clusterFrontiers3D

`void clusterFrontiers3D(Map *h_map, double maxClusterRadiusMeters, CudaGrid3D::Point origin, CudaGrid3D::Point *bestCentroid, CudaGrid3D::IntPoint **cluster, int *sizeCluster)`

Cluster the frontier cells of the 3D Grid using a combination of k-means (with k=2) and hierarchical divisive clustering

| Parameter              | Description                                                                                                                      |
| ---------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| h_map                  | Map struct                                                                                                                       |
| maxClusterRadiusMeters | Maximum distance of a point from its centroid in a cluster. If the distance is bigger the cluster is splitted in 2 sub clusters. |
| origin                 | Inflation margin                                                                                                                 |
| bestCentroid           | Centroid of the best cluster (result)                                                                                            |
| cluster                | List of points in the best cluster (result)                                                                                      |
| sizeCluster            | Number of points in the best cluster (result)                                                                                    |

## inflateObstacles2D

`void inflateObstacles2D(Map *h_map, double radius, int freeThreshold, int warningThreshold, int occupiedThreshold)`

Inflate the obstacles in the 2D occupancy map

| Parameter         | Description                         |
| ----------------- | ----------------------------------- |
| h_map             | Map struct                          |
| radius            | Inflation radius                    |
| freeThreshold     | Max confidence for a free cell      |
| warningThreshold  | Min confidence for a warning cell   |
| occupiedThreshold | Min confidence for an occupied cell |

## bestObservationPoint2D

`CudaGrid3D::Point bestObservationPoint2D(Map *h_map, CudaGrid3D::Point clusterCenterMeters, CudaGrid3D::IntPoint *cluster, int sizeCluster, double radiusMeters, double angleIntervalDeg, int freeThreshold, double cameraHeightMeters)`

Compute the best point in the map to observe the cluster. The points are searched on the circumference on the 2D map with the centre as the projected cluster centroid on the 2D map and with a specified radius

| Parameter           | Description                                                                                             |
| ------------------- | ------------------------------------------------------------------------------------------------------- |
| h_map               | Map struct                                                                                              |
| clusterCenterMeters | Centroid of the cluster to observe (coordinates in meters)                                              |
| cluster             | Array of points of the cluster to observe                                                               |
| sizeCluster         | Number of points in the cluster to observe                                                              |
| radiusMeters        | Distance from the projected cluster centroid on the 2D map where to look for the best observation point |
| angleIntervalDeg    | Search for a point every x degrees on the circumference                                                 |
| freeThreshold       | Max confidence for a free cell                                                                          |
| cameraHeightMeters  | Camera z-position (meters)                                                                              |













