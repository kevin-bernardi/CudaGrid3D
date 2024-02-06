# CudaGrid3D

[Detailed Reference Page](https://github.com/kevin-bernardi/CudaGrid3D/blob/main/reference.md)

## Table of Contents

- [CudaGrid3D](#cudagrid3d)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Why CudaGrid3D?](#why-cudagrid3d)
  - [Functionalities](#functionalities)
  - [Build](#build)
  - [How does the 3D Grid works?](#how-does-the-3d-grid-works)
- [Guide](#guide)
  - [Conventions](#conventions)
  - [Map Initialization](#map-initialization)
  - [Insert Pointclouds](#insert-pointclouds)
    - [Rototranslate the Pointcloud](#rototranslate-the-pointcloud)
  - [Create a Random Pointcloud](#create-a-random-pointcloud)
  - [Free Volume Computation](#free-volume-computation)
  - [2D Grid Generation](#2d-grid-generation)
    - [Frontier Detection](#frontier-detection)
  - [2D Occupancy Map Generation](#2d-occupancy-map-generation)
  - [3D Mesh Generation](#3d-mesh-generation)

## Introduction

CudaGrid3D is a C library that implements a 3D grid for pointcloud insertion, a ray-tracing algorithm for free space calculations and a 2D occupation map updated dynamically. The library uses CUDA and the extremely large parallelization capacity of GPUs for handling very high data flows without lags.

> **Warning**: CudaGrid3D is written using CUDA so only systems equipped with a NVIDIA GPU can use this library.

## Why CudaGrid3D?

There are a lot of libraries that help developers in the task of inserting pointclouds in a 3D space but they usually do all the computation only on one single core and therefore not allowing the handling of high data flows which are very common in 3D scanning.

CudaGrid3D uses GPUs to divide the work load on thousands of cores and allowing the handling of a much higher flow of data.

## Functionalities

This library exposes many functions that can be easily integrated in many applications to solve a variety of problems:

- 2D and 3D map management;
- pointcloud insertion;
- pointcloud rototranslation;
- pointcloud random generation (for testing purposes);
- 2D and 3D map print;
- pointcloud print;
- 2D and 3D mesh generation;
- free space computing with ray-tracing;
- 2D occupation grid display with runtime updates.

## Build

In order to use the CudaGrid3D in your application you have to build the library for your target system.

Run the `make` command in the library folder to build the shared library (.so file).

Then you can link the library using for example

```shell
g++ -o program application.cpp -L{cudaGrid3D .so file path} -lgrid3d
```

or if you also use OpenCV based functions

```shell
g++ -o program application.cpp -L{cudaGrid3D .so file path} -lgrid3d -lopencv_highgui -lopencv_imgproc -lopencv_core -I{OpenCV path}
```

## How does the 3D Grid works?

The 3D grid is a data structure allocated in the GPU memory where you can insert the 3D points with (x,y,z) coordinates.
The grid has a size of `dimX` x `dimY` x `dimZ`. These 3 dimensions are specified upfront by the developer at grid initialization.
This representation of space is discrete and each cell is a cube and every cell has the same size.

Each cell can have 4 possible values:
- `UNKNOWN_CELL`: unknown space (default value);
- `OCCUPIED_CELL`: obstacle;
- `FREE_CELL`: free space;
- `FRONTIER_CELL`: free cell near unknown space.

A smaller cell size results in a more precise 2D/3D map but at the cost of more computational work.

The 2D grid is a projection of the 3D grid and the user cannot insert the points directly in the 2D grid.

# Guide

This guide explains the general workflow needed for pointcloud insertion, free space computation, occupancy grid update and 3D mesh generation.
The guide does not explain the complete list of arguments for each function.
Each function is documented in detail in the [functions reference page](https://github.com/kevin-bernardi/CudaGrid3D/blob/main/reference.md).


A complete example of the library usage can be found [here](https://github.com/kevin-bernardi/CudaGrid3D/blob/main/example.cpp).

## Conventions

CudaGrid3D use both RAM and VRAM (RAM mounted on the GPU).
Pointers pointing to an allocated space in the RAM (called host memory) are marked with `h_` at the beginning of the pointer's name. Pointers pointing to an allocated space in the VRAM (called device memory) are instead marked with `d_`.

>E.g.: a pointer pointing to a grid in the host memory (RAM) must be called `h_grid`. Otherwise, if the grid is allocated in the device memory (VRAM), the pointer must be called `d_grid`.

## Map Initialization

The first step to do before inserting pointclouds in the 3D Grid is to initialize the Map, a struct that holds the 2D Grid, the 3D Grid and all the parameters that are needed to make them work as expected.
To initialize the map just call the function `initMap`. Below you can see a Map initialization of a 3D grid of size 200 x 100 x 50 cells and a 2D grid of size 200 x 100 cells. Both grids have a cell size of 10 centimeters.

The last 2 arguments (floorVoxelsMargin and robotVoxelsHeight) are used to compute the 2D Grid from the 3D grid and can be overlooked at the moment until the occupation map is explained later.

```c++
#include "grid3d.h"

[...]

int dimX = 200;
int dimY = 100;
int dimZ = 50;
float cellSize = 0.10;
int floorVoxelsMargin = 1;
int robotVoxelsHeight = 5;

CudaGrid3D::Map* h_map = new CudaGrid3D::Map;  
// the map is allocated on the host memory

initMap(h_map, dimX, dimY, dimZ, cellSize, floorVoxelsMargin, robotVoxelsHeight);
```

> WARNING: The grids contained in the `Map` can't be directly accessed because are allocated on the device memory. The grids can be accessed or manipulated using only the functions provided by the library.

## Insert Pointclouds

A pointcloud is an array of points representing obstacles in the environment. Each point of the pointcloud is inserted in the correct cell of the 3D grid and marked the cell as occupied.

A pointcloud can be inserted in the 3D Grid using the function `insertPointcloud`.
This function accepts an array of `CudaGrid3D::Point` allocated on the device memory. Therefore, the points acquired must be converted to the type the library uses and then copy the data on a pointcloud allocated on the device memory using `initDevicePointcloud`.

The coordinates of the points are expressed in meters.

If you have an array of points in this format [point0_x, point0_y, point0_z, point0_w, point1_x, point1_y, point1_z, point1_w, ...] you can pass it to the `arrayToPointcloud` function that converts it to the pointcloud type accepted by the library. This function already initializes a pointcloud on the device memory so you don't have to use `initDevicePointcloud` like before.

The `pointX_w` is not used by the function so if your array that has only 3 coordinates for each point without additional data then you can put a random number in the fourth "coordinate".

> Some robotic applications save the points captured in a 4 channel OpenCV Mat object (cv::Mat).
> This object can be easily converted to a classic array and then passed in the `arrayToPointcloud` function explained before.
> More informations about cv::Mat to array conversion can be found here: https://stackoverflow.com/questions/26681713/convert-mat-to-array-vector-in-opencv

Here are shown two examples about pointcloud creation and conversion.

Example 1: the pointcloud has already the right type:


```c++
CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
initMap(h_map, dimX, dimY, dimZ, cellSize, floorVoxelsMargin, robotVoxelsHeight);

CudaGrid3D::Point* d_pointcloud;

int numPoints = 3;
CudaGrid3D::Point* h_pointcloud = (CudaGrid3D::Point*)malloc(sizeof(CudaGrid3D::Point) * numPoints);

CudaGrid3D::Point pt;
pt.x = 2.32; // meters
pt.y = 1.26; // meters
pt.z = 0.81; // meters

h_pointcloud[0] = pt;
h_pointcloud[1] = pt;
h_pointcloud[2] = pt;
// to shorten the example the pointcloud consists of 3 identical points

// the host pointcloud must be converted to a device pointcloud
// d_pointcloud must be a pointer to a pointer 
// (this is why we pass the address of the pointer)
initDevicePointcloud(&d_pointcloud, h_pointcloud, numPoints);

// insert the points in the 3D Grid
insertPointcloud(h_map, d_pointcloud, numPoints);
```

Example 2: the camera gives a cv::Mat containing the points:

```c++
// convert a cv::Mat to a classic array (easier to manipulate)
void cvMatToArray(cv::Mat* cv_matrix, float** result, int* result_length) {

    // convert cv::Mat to a classic C array
    *result = cv_matrix->isContinuous() ? (float*)cv_matrix->data : (float*)cv_matrix->clone().data;

    // lenth of the array result
    *result_length = cv_matrix->total() * cv_matrix->channels();
}

int main(){

    [...]


    CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
    initMap(h_map, dimX, dimY, dimZ, cellSize, floorVoxelsMargin, robotVoxelsHeight);

    // pointcloud_raw is the data received by a sensor / camera etc.
    cv::Mat pointcloud_raw;

    CudaGrid3D::Point* d_pointcloud;

    float* h_arr;

    int length;
    
    
    cvMatToArray(&pointcloud_raw, &h_arr, &length);

    // convert an array of coordinates into a pointcloud without rototranslations
    arrayToPointcloud(h_arr, length, &d_pointcloud);

    // insert the points in the 3D Grid
    // the number of points is length / 4 because the pointcloud is an array of Points while
    // h_arr (of size "length") has the format [pt0_x, pt0_y, pt0_z, pt0_w, pt1_x...]
    // so the number of points is the length of the array divided by 4

    int numPoints = length / 4;
    insertPointcloud(h_map, d_pointcloud, numPoints);

    [...]
}
```

### Rototranslate the Pointcloud

The function `arrayToPointcloud` can also rototranslate all the points based on the CudaTransform3D object passed to the function. The CudaTransform3D is a struct that contains the vector of translation and the matrix of rotation. These informations are needed to rotate and translate the pointcloud.
To rototranslate the pointcloud just pass the CudaTransform3D as the last argument of the function.
This function in GPU accelerated. If your applications requires pointcloud rototranslation, doing it using this function will result in better performance.


## Create a Random Pointcloud

For testing purposes the library also provide the function `generateRandomPointcloud` that generates a pointcloud allocated on the device memory with N random points within the bounds of the 3D grid.

This pointcloud can be directly inserted in the 3D grid by using the `insertPointcloud` function.

The `generateRandomPointcloud` function doesn't require the pointcloud initialization.


```c++

    [...]


    CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
    initMap(h_map, dimX, dimY, dimZ, cellSize, floorVoxelsMargin, robotVoxelsHeight);

    CudaGrid3D::Point* d_pointcloud;

    generateRandomPointcloud(h_map, &d_pointcloud, numPoints);
    insertPointcloud(h_map, d_pointcloud, numPoints);


    [...]
```

## Free Volume Computation

The free volume computation is done with a simplified ray-tracing algorithm running in the 3D grid.
A ray is traced for each point in the pointcloud. The ray starts at the coordinates of the camera and ends at the coordinates of the point in the pointcloud.
The rays pass through some cells that are marked as free. The ray stops before reaching the destination (the point of the pointcloud) if it encounters an obstacle.

For free volume computation the library provide the function `pointcloudRayTracing`.
The last argument of the function (`freeObstacles`) takes a boolean. If this argument is true then the algorithm can mark as free also occupied cells. Otherwise occupied cells cannot be marked as free.

The function only marks and inserts the free cells in the 3D grid and does not insert the pointcloud itself (the obstacles).
For this reason this function is usually called just after `insertPointcloud`.

Here is a short example:

```c++

    [...]


    CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
    initMap(h_map, dimX, dimY, dimZ, cellSize, floorVoxelsMargin, robotVoxelsHeight);

    CudaGrid3D::Point* d_pointcloud;
    int pointcloud_length;

    // ...
    // pointcloud conversions or generation
    // ...

    insertPointcloud(h_map, d_pointcloud, pointcloud_length);

    // coordinates of the camera when it acquired the pointcloud
    CudaGrid3D::Point camera_coords;
    camera_coords.x = 0.0;
    camera_coords.y = 0.0;
    camera_coords.z = 0.0;

    bool freeObstacles = false;

    pointcloudRayTracing(h_map, d_pointcloud, pointcloud_length, camera_coords, freeObstacles);

    [...]
```

## 2D Grid Generation

The 2D occupancy grid is generated as a projection of the 3D grid. Only the cells inside the height interval of the robot are taken into account.
To avoid the detection of the floor as an obstacle only the cells above a certain threshold are considered.
Both the height of the robot and the floor threshold are defined as two variables inside the `Map` structure with the `initMap` function.
The height of the robot is set with the `robotVoxelsHeight` variable and the floor threshold is set with the `floorVoxelsMargin` variable.
Both variables must not use meters but number of cells (voxels).

The `updateGrid2D` function generate the 2D occupancy grid.
This function takes 3 thresholds as arguments:

- freeThreshold (used for frontier detection)
- maxUnknownConfidence
- minOccupiedConfidence


For each cell `(x,y, z>= floorVoxelsMargin and z < robotVoxelsHeight)` in the 2D grid a occupied confidence value (grid2DCellValue) between 0 and 100 is calculated:

- if there are no occupied cells in the column (fixed (x,y), variable z):
    1. the ratio of unknown cells in the height of the robot (floor margin excluded) is calculated:
    
        `unknownRatio = numUnknownCells / (robotVoxelsHeight - floorVoxelsMargin)`

    2. if every cell is unknown then the `grid2DCellValue` will be equal to `maxUnknownConfidence`, if every cell is free then the `grid2DCellValue` will be 0 
        
        `grid2DCellValue = unknownRatio * maxUnknownConfidence`
  
- if there are occupied cells in the column (fixed (x,y), variable z):
    1. only 1 occupied cell in the column (above floor margin):

        `grid2DCellValue = minOccupiedConfidence`

    2. 2 occupied cells:

        `grid2DCellValue = (100 + minOccupiedConfidence) / 2`

    3. 3 or more occupied cells:

        `grid2DCellValue = 100`

### Frontier Detection

When the function `updateGrid2D` is called, after the 2D grid update an algorithm for frontier detection is runned. The frontier detection algorithm marks as frontier all the free cells on the 2D grid that have at least a common side to an unknown cell.

## 2D Occupancy Map Generation

The 2D occupancy map generation is done based on the 2D grid (updated with the `updateGrid2D` function we seen in the previous paragraph).

This function doesn't display anything nor generates or save any image because in the 2D grid we have saved in each cell the probability that the cell represents an obstacle (occupied cell).

To generate an image, the function `getGrid2D` has to be called.
The function scans the entire 2D grid and compares each value with three new thresholds to paint the pixel image with a different color:
- free threshold (green)
- warning threshold (orange)
- occcupied threshold (red)

> The free threshold must be lower than the warning threshold.
>
> The warning threshold must be lower than the occupied threshold.

> Frontier cells are not compared to any threshold.

The image has the same size as the 2D grid and the cell with coordinates (x1, y1) is represented by the image pixel with the same coordinates (x1, y1). The first cell in the 2D grid corresponds to the first pixel in the occupancy map image.

The algorithm scans the 2D grid and paints the pixels of the occupancy map in the following way:
- If `grid2DCellValue <= freeThreshold` the pixel image is painted green (free cell);
- if `grid2DCellValue > freeThreshold` and `grid2DCellValue < warningThreshold` the pixel image is painted grey (unknown cell);
- if `grid2DCellValue >= warningThreshold` and `grid2DCellValue < occupiedThreshold` the pixel image is painted orange (maybe occupied cell);
- if `grid2DCellValue >= occupiedThreshold` the pixel image is painted red (occupied cell);
- if the `grid2DCell` is a frontier cell then the pixel image is painted blue.


The function then returns the image as a OpenCV `cv::Mat` object that can be displayed using for example the `cv::imshow` function (OpenCV).

The function can also print a marker (circle) in the occupancy map showing the robot position at the moment of the last grid update. To enable this feature just pass the robot position (CudaGrid3D::CudaTransform3D object) and the marker radius as the last two arguments of the `getGrid2D` function call.

Here is an example on how to use both `updateGrid2D` and `getGrid2D`:

```c++

    [...]


    CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
    initMap(h_map, dimX, dimY, dimZ, cellSize, floorVoxelsMargin, robotVoxelsHeight);

    // ...
    // pointcloud insertion in the 3D grid
    // ...

    // ...
    // free volume computation with ray tracing
    // ...

    // update the 2D grid using the 3D grid (projection)
    updateGrid2D(h_map, freeThreshold, maxUnknownConfidence, minOccupiedConfidence);

    
    // occupancy grid without the robot position marker
    cv::Mat occupancyMap = getGrid2D(h_map, freeThreshold, warningThreshold, occupiedThreshold);


    // --- ALTERNATIVELY ---
    // occupancy grid with the robot position marker

    // robot position
    CudaGrid3D::CudaTransform3D robot_position;
    int marker_radius = 3;

    cv::Mat occupancyMap = getGrid2D(h_map, freeThreshold, warningThreshold, occupiedThreshold, &robot_position, marker_radius);
    // ---------------------

    // display the occupancy 2D map
    cv::imshow("Occupancy Map", occupancyMap);
    cv::waitKey(10);

    [...]
```

## 3D Mesh Generation

The library has a function to generate a 3D mesh in Wavefront format (.obj).

Call the `generateMesh3D` function to generate a mesh of the entire 3D grid at the specified path.
This function creates a cube for each cell marked as occupied and can quicly increase in size for big 3D grids (> 1 GB).

For this reason the library also provides the function `generateSimpleMesh3D` which generates a simple mesh where a vertex (and not a cube) is created for each cell marked as occupied.
This function can also generate a mesh of the free space if the last argument (`isOccupationMesh`) is false.

The mesh file can be later imported in any 3D viewer application.

> The simple mesh file support is very limited because many viewers can't display meshes without faces. Blender is a great and free 3D manipulation tool that has the support for these meshes.

Here is a simple example for 3D mesh generation:

```c++

    [...]


    CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
    initMap(h_map, dimX, dimY, dimZ, cellSize, floorVoxelsMargin, robotVoxelsHeight);

    // ...
    // pointcloud insertion in the 3D grid
    // ...

    // ...
    // free volume computation with ray tracing
    // ...

    // complete mesh generation
    generateMesh3D(h_map, "./mesh.obj");

    // simple mesh generation

    MeshType meshType = OCCUPANCY_MAP;
    generateSimpleMesh3D(h_map, "./simple_mesh.obj", meshType);

    [...]
```






