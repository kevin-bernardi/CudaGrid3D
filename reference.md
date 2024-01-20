# Functions Reference

- [Functions Reference](#functions-reference)
  - [initMap](#initmap)
  - [freeMap](#freemap)
  - [initDevicePointcloud](#initdevicepointcloud)
  - [insertPointcloud](#insertpointcloud)
  - [arrayToPointcloud](#arraytopointcloud)
  - [printPointcloud](#printpointcloud)
  - [generateMesh](#generatemesh)
  - [generateSimpleMesh](#generatesimplemesh)
  - [pointcloudRayTracing](#pointcloudraytracing)
  - [updateGrid2D](#updategrid2d)
  - [getUnknownDensityGrid2D](#getunknowndensitygrid2d)
  - [getGrid2D](#getgrid2d)


## initMap

`void initMap(Map *h_map, int dimX, int dimY, int dimZ, float cellSize, int freeVoxelsMargin, int robotVoxelsHeight)`

Initialize the Map on the host and the grids (both 2D and 3D) on the device.

| Parameter         | Description                                                                          |
| ----------------- | ------------------------------------------------------------------------------------ |
| h_map             | Map struct                                                                           |
| dimX              | Number of cells on the x-axis                                                        |
| dimY              | Number of cells on the y-axis                                                        |
| dimZ              | Number of cells on the z-axis                                                        |
| cellSize          | Edge length of each cell in meters                                                   |
| freeVoxelsMargin  | Number of cells from the ground to not consider in 2D grid updates using the 3D grid |
| robotVoxelsHeight | Height of the robot as number of cells                                               |

## freeMap

`void freeMap(Map *h_map)`

Free the map struct (allocated on the host) and the grids (allocated on the device).

| Parameter | Description |
| --------- | ----------- |
| h_map     | Map struct  |

## initDevicePointcloud

`void initDevicePointcloud(Point **d_pointcloud, int sizePointcloud)`

Initialize a pointcloud in the device memory.

| Parameter      | Description                                     |
| -------------- | ----------------------------------------------- |
| d_pointcloud   | Pointcloud pointer address (pointer to pointer) |
| sizePointcloud | Max number of points the pointcloud can hold    |

`initDevicePointcloud(Point **d_pointcloud, Point *h_pointcloud, int sizePointcloud)`

Initialize a pointcloud in the device memory and copy the host pointcloud in the device pointcloud just allocated.

| Parameter      | Description                                     |
| -------------- | ----------------------------------------------- |
| d_pointcloud   | Pointcloud pointer address (pointer to pointer) |
| h_pointcloud   | Host pointcloud                                 |
| sizePointcloud | Number of points in the pointcloud              |


## insertPointcloud

`void insertPointcloud(Map *h_map, Point *d_pointcloud, int sizePointcloud)`

Inserts the points of the pointcloud in the 3D grid

| Parameter      | Description                        |
| -------------- | ---------------------------------- |
| h_map          | Map struct                         |
| d_pointcloud   | Pointcloud allocated on the device |
| sizePointcloud | Number of points in the pointcloud |

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



`void printPointcloud(Point *d_pointcloud, int sizePointcloud)`

Print the device pointcloud.

| Parameter      | Description                        |
| -------------- | ---------------------------------- |
| d_pointcloud   | Pointcloud allocated on the device |
| sizePointcloud | Number of points in the pointcloud |

## generateMesh

`void generateMesh(Map *h_map, const char *path)`

Generate a mesh of the 3D grid.

| Parameter | Description                     |
| --------- | ------------------------------- |
| h_map     | Map struct                      |
| path      | Save path of the generated mesh |

## generateSimpleMesh

`void generateSimpleMesh(Map *h_map, const char *path)`

Generate a simple mesh of the 3D grid (only a vertex for each occupied cell).

| Parameter | Description                     |
| --------- | ------------------------------- |
| h_map     | Map struct                      |
| path      | Save path of the generated mesh |

## pointcloudRayTracing

`void pointcloudRayTracing(Map *h_map, Point *d_pointcloud, int sizePointcloud, Point origin, bool freeObstacles)`

Run the ray tracing algorithm to find free cells in the 3D grid.

| Parameter      | Description                                      |
| -------------- | ------------------------------------------------ |
| h_map          | Map struct                                       |
| d_pointcloud   | Pointcloud allocated on the device               |
| sizePointcloud | Number of points in the pointcloud               |
| origin         | Origin point (camera coordinates)                |
| freeObstacles  | If true the occupied cells can be marked as free |

## updateGrid2D

`void updateGrid2D(Map *h_map, int freeThreshold, int maxUnknownConfidence, int minOccupiedConfidence)`

Update the 2D grid using the data contained in the 3D grid (projection).

| Parameter             | Description                                             |
| --------------------- | ------------------------------------------------------- |
| h_map                 | Map struct                                              |
| freeThreshold         | Max confidence to be marked free                        |
| maxUnknownConfidence  | Max confidence if every cell in the column is unknown   |
| minOccupiedConfidence | Min confidence if only 1 cell is occupied in the column |


## getUnknownDensityGrid2D

`void getUnknownDensityGrid2D(Map *h_map, int bin_size, int freeThreshold, int occupiedThreshold, int *&output_grid_2d_binned, int &dimX, int &dimY)`

Get a binned (reduced in resolution) density 2D grid

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

`cv::Mat getGrid2D(Map *h_map, int freeThreshold, int warningThreshold, int occupiedThreshold, CudaTransform3D *robotPosition)`

Get a displayable 2D occupancy grid with an indicator of the robot position

| Parameter         | Description                          |
| ----------------- | ------------------------------------ |
| h_map             | Map struct                           |
| freeThreshold     | Max confidence for a free pixel      |
| warningThreshold  | Min confidence for a warning pixel   |
| occupiedThreshold | Min confidence for an occupied pixel |
| robotPosition     | Coordinate position of the robot     |













