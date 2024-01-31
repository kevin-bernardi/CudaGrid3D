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
  - [updateGrid2D](#updategrid2d)
  - [getUnknownDensityGrid2D](#getunknowndensitygrid2d)
  - [getGrid2D](#getgrid2d)


## initMap

`void initMap(Map *h_map, int dimX, int dimY, int dimZ, float cellSize, int floorVoxelsMargin, int robotVoxelsHeight)`

Initialize the Map on the host and the grids (both 2D and 3D) on the device.

| Parameter         | Description                                                                          |
| ----------------- | ------------------------------------------------------------------------------------ |
| h_map             | Map struct                                                                           |
| dimX              | Number of cells on the x-axis                                                        |
| dimY              | Number of cells on the y-axis                                                        |
| dimZ              | Number of cells on the z-axis                                                        |
| cellSize          | Edge length of each cell in meters                                                   |
| floorVoxelsMargin | Number of cells from the ground to not consider in 2D grid updates using the 3D grid |
| robotVoxelsHeight | Height of the robot as number of cells                                               |

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

`void generateSimpleMesh(Map *h_map, const char *path, bool isOccupationMesh)`

Generate a simple mesh of the 3D grid (only a vertex for each occupied cell). If `isOccupationMesh` is false then a mesh of the free space is generated instead.

| Parameter        | Description                                                                                                |
| ---------------- | ---------------------------------------------------------------------------------------------------------- |
| h_map            | Map struct                                                                                                 |
| path             | Save path of the generated mesh                                                                            |
| isOccupationMesh | If true the function generates a mesh of the occupied space, otherwise it creates a mesh of the free space |

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













