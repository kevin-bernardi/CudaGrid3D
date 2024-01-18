# CudaGrid3D

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
  - [2D Occupancy Grid Generation](#2d-occupancy-grid-generation)

## Introduction

CudaGrid3D is a C library that implements a 3D grid for pointcloud insertion, a ray-tracing algorithm for free space calculations and an efficient 2D occupation map updated dynamically. The library uses CUDA and the extremely large parallelization capacity of GPUs for handling very high data flows without lags.

> **Warning**: CudaGrid3D is written using CUDA so only systems equipped with a NVIDIA GPU can use this library.

## Why CudaGrid3D?

There are a lot of libraries that help developers in the task of inserting pointclouds in a 3D space but they usually only do all the computation on one single core and therefore not allowing the handling of high data flows which are very common in 3D scanning of an environments.

CudaGrid3D uses GPUs to divide the work load on thousands of cores and allowing the handling of a much higher flow of data.

## Functionalities

This library exposes many functions that you can easily integrate in your application to solve many problems:

- 2D and 3D map managements;
- pointcloud insertion;
- pointcloud rototranslation;
- pointcloud random generation (for testing purposes);
- check duplicates in the pointcloud;
- 2D and 3D map print;
- pointcloud print;
- 2D and 3D mesh generation;
- free space computing with ray-tracing;
- 2D occupation grid display with runtime updates.

## Build

In order to use the CudaGrid3D in your application you have to build the library for your target system.

Run the `make` command in the library folder to create the shared library (.so file).

Then you can link the library using for example:

```shell
g++ -o program application.cpp -L{cudaGrid3D .so file path} -lgrid3d
```

or if you also use openCV based functions:

```shell
g++ -o program application.cpp -L{cudaGrid3D .so file path} -lgrid3d -lopencv_core -lopencv_highgui -I{opencv path}
```

## How does the 3D Grid works?

The 3D Grid is a place in memory where you can insert the 3D points with (x,y,z) coordinates.
The grid has a size of `dimX` x `dimY` x `dimZ`. These 3 dimensions are specified upfront by the developer.
This representation of space is discrete and every cell is a cube of the same size.
Each cell can have 4 possible values:
- `UNKNOWN_CELL`: unknown space (default value);
- `OCCUPIED_CELL`: obstacle;
- `FREE_CELL`: free space;
- `FRONTIER_CELL`: free cell near unknown space.

A smaller cell size results in a more precise 2D/3D map but at the cost of more computational work.
The 2D Grid is simply a projection of the 3D Grid and the user cannot insert the points directly in the 2D Grid.

# Guide

## Conventions

CudaGrid3D use both RAM and VRAM (RAM mounted on the GPU).
Pointers pointing to an allocated space in the RAM (called host memory) are marked with `h_` at the beginning of the pointer name. Pointers pointing to an allocated space in the VRAM (called device memory) are instead marked with `d_`.

>E.g.: a pointer pointing to a grid in the host memory (RAM) must be called `h_grid`. Insted, if the grid is on the device memory (VRAM), the pointer must be called `d_grid`.

## Map Initialization

The first step to do before inserting pointclouds in the 3D Grid is to initialize the Map, a struct that holds the 2D Grid, the 3D Grid and all the parameters that are needed to make them work as expected.
To initialize the map just call the function `initMap`. Below you can see a Map initialization of a 3D grid of size 200 x 100 x 50 cells and a 2D grid of size 200 x 100 cells. Both grids have a cell size of 10 centimeters.

The last 2 arguments (freeVoxelsMargin and robotVoxelsHeight) are used to compute the 2D Grid from the 3D Grid and can be overlooked until the occupation map is explained later.

```c++
#include "grid3d.h"

....

int dimX = 200;
int dimY = 100;
int dimZ = 50;
float cellSize = 0.10;
int freeVoxelsMargin = 1;
int robotVoxelsHeight = 5;

CudaGrid3D::Map* h_map = new CudaGrid3D::Map;  // the map is allocated on the host memory
initMap(h_map, dimX, dimY, dimZ, cellSize, freeVoxelsMargin, robotVoxelsHeight);
```

> WARNING: The grids contained in the `Map` can't be directly accessed because are allocated on the device memory. The grids can be accessed or manipulated using only the functions provided by the library.

## Insert Pointclouds

A pointcloud is an array of points representing obstacles in the environment. Each point of the pointcloud is inserted in the correct cell of the 3D Grid and marked the cell as occupied.

A pointcloud can be inserted in the 3D Grid using the `insertPointcloud`.
This function accepts an array of `CudaGrid3D::Point` allocated on the device memory. The points acquired must be converted to the type the library uses and then initialize a pointcloud allocated on the device memory using `initDevicePointcloud`.

The coordinates of the points are expressed in meters.

If you have an array of points in this format [point0_x, point0_y, point0_z, point0_w, point1_x, point1_y, point1_z, point1_w, ...] you can pass it to the `arrayToPointcloud` function that converts it to the pointcloud type accepted by the library. This function already initializes a pointcloud on the device memory so you don't have to use `initDevicePointcloud` like before.

The `pointX_w` is not used by the function so if your array has only 3 coordinates for each point without additional data you can  put 0 or a random number.

> Some robotic applications save the points captured in a 4 channel OpenCV Mat object (cv::Mat).
> This object can be easily be converted to a classic array and then passed in the `arrayToPointcloud` function explained before.
> More informations about cv::Mat to array conversion can be found here: https://stackoverflow.com/questions/26681713/convert-mat-to-array-vector-in-opencv


Here are shown two examples that should be useful in understanding better how all this works.

Example 1: the pointcloud has already the right type


```c++
CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
initMap(h_map, dimX, dimY, dimZ, cellSize, freeVoxelsMargin, robotVoxelsHeight);

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
// d_pointcloud must be a pointer to a pointer (this is why we pass the address of the pointer)
initDevicePointcloud(&d_pointcloud, h_pointcloud, numPoints);

// insert the points in the 3D Grid
insertPointcloud(h_map, d_pointcloud, numPoints);
```

Example 2: the camera gives a cv::Mat containing the points

```c++

// convert a cv::Mat to a classic array (easier to manipulate)
void cvMatToArray(cv::Mat* cv_matrix, float** result, int* result_length) {

    // convert cv::Mat to a classic C array
    *result = cv_matrix->isContinuous() ? (float*)cv_matrix->data : (float*)cv_matrix->clone().data;

    // lenth of the array result
    *result_length = cv_matrix->total() * cv_matrix->channels();
}

int main(){

    ...


    CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
    initMap(h_map, dimX, dimY, dimZ, cellSize, freeVoxelsMargin, robotVoxelsHeight);

    // pointcloud_raw is the data received by a sensor / camera etc.
    cv::Mat pointcloud_raw;

    CudaGrid3D::Point* d_pointcloud;

    float* h_arr;

    int length;
    
    
    cvMatToArray(&pointcloud_raw, &h_arr, &length);


    CudaTransform3D tf;

    // convert an array of coordinates into a pointcloud without doing rototranslation
    CudaGrid3D::arrayToPointcloud(h_arr, length, &d_pointcloud, false, tf);

    // insert the points in the 3D Grid
    // the number of points is length / 4 because the pointcloud is an array of Points while
    // h_arr (of size "length") has the format [pt0_x, pt0_y, pt0_z, pt0_w, pt1_x...]
    // so the number of points is the length of the array divided by 4

    int numPoints = length / 4;
    insertPointcloud(h_map, d_pointcloud, numPoints);

    ...
}
```

### Rototranslate the Pointcloud

The function `arrayToPointcloud` can also rototranslate all the points based on the CudaTransform3D object passed to the function. The CudaTransform3D is a struct that contains the vector of translation and the matrix of rotation. These informations are needed to rotate and translate the pointcloud.
In the example above we disabled the rototranslation by setting the fourth argument to false and passing a default CudaTransform3D as the last argument of the function.

## Create a Random Pointcloud

For testing purposes the library also provide the function `generateRandomPointcloud` that generated a pointcloud allocated on the device memory with N random points within the bounds of the 3D Grid.
This pointcloud can be directly inserted in the 3D Grid by using the `insertPointcloud` function.

The `generateRandomPointcloud` function doesn't require the pointcloud initialization to avoid memory size mismatches (like `arrayToPointcloud`).


```c++

    ...


    CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
    initMap(h_map, dimX, dimY, dimZ, cellSize, freeVoxelsMargin, robotVoxelsHeight);

    CudaGrid3D::Point* d_pointcloud;

    generateRandomPointcloud(h_map, &d_pointcloud, numPoints);
    insertPointcloud(h_map, d_pointcloud, numPoints);

    ...
}
```

## Free Volume Computation

The free volume computation is done with ray-tracing.
A ray is casted for each point in the pointcloud. The ray starts at the coordinates of the camera and ends at the coordinates of the point in the pointcloud.
The rays pass through some cells that are marked as free. The ray stops before reaching the destination (the point of the pointcloud) if it encounters an obstacle.

For free volume computation the library provide the function `pointcloudRayTracing`.
The last argument of the function takes a boolean. If this argument is true then the algorithm can mark as free also occupied cells. Otherwise occupied cells cannot be marked as free.

The function only marks free cells and does not insert the pointcloud in the 3D grid.
For this reason the function is usually called just after the `insertPointcloud` call.

Here is a short example:

```c++

    ...


    CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
    initMap(h_map, dimX, dimY, dimZ, cellSize, freeVoxelsMargin, robotVoxelsHeight);

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

    ...
```

## 2D Occupancy Grid Generation

The 2D occupancy grid is generated as a projection of the 3D grid. Only the cells inside the height interval of the robot are taken into account.
To avoid the detection of the floor as an obstacle, only the cells above a certain threshold are considered.
Both the height of the robot and the floor threshold are defined as two variables inside the `Map` structure.
The height of the robot is set with the `robotVoxelsHeight` variable and the floor threshold is set with the `floorVoxelsMargin` variable.
Both variables must not use meters but number of cells (voxels).

The `updateGrid2D` function generate the 2D occupancy grid.
This function takes 3 thresholds as arguments:

- freeThreshold 
- maxUnknownConfidence
- minOccupiedConfidence


For each cell (x,y) in the 2D grid a number (grid2DCellValue) between 0 and 100 is computed:

- if there are no occupied cells in the column (x,y):
    1. the ratio of unknown cells in the height of the robot (floor margin excluded) is calculated:
    
        `unknownRatio = numUnknownCells / (robotVoxelsHeight - floorVoxelsMargin)`

    2. if every cell is unknown then the `grid2DCellValue` will be equal to `maxUnknownConfidence`, if every cell is free then the `grid2DCellValue` will be 0 
        
        `grid2DCellValue = unknownRatio * maxUnknownConfidence`
  
- if there are occupied cells in the column (x,y):
    1. only 1 occupied cell in the height of the robot (floor margin excluded):

        `grid2DCellValue = minOccupiedConfidence`

    2. 2 occupied cells:

        `grid2DCellValue = (100 + minOccupiedConfidence) / 2`

    3. 3 or more occupied cells:

        `grid2DCellValue = 100`






























