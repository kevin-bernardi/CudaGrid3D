# CudaGrid3D

CudaGrid3D is a C library that implements a 3D grid for pointcloud insertion, a ray-tracing algorithm for free space calculations and an efficient 2D occupation map updated dynamically. The library uses CUDA and the extremely large parallelization capacity of GPUs for handling very high data flows without lags.

> **Warning**: CudaGrid3D is written using CUDA so only computers equipped with a NVIDIA GPU can use the library.

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

## Insert pointclouds

A pointcloud (array of points) can be inserted in the 3D Grid using the `insertPointcloud()`.
This function accepts an array of `CudaGrid3D::Points` allocated on the device memory and the points captured must be converted to the type the library uses and then initialize a pointcloud allocated on the device memory using `initDevicePointcloud()`.

The coordinates of the points are expressed in meters.

If you have an array of points in this format [point0_x, point0_y, point0_z, point0_w, point1_x, point1_y, point1_z, point1_w, ...] you can pass it to the `arrayToPointcloud()` function that converts it to the pointcloud type accepted by the library. This function already initializes a pointcloud on the device memory so you don't have to use `initDevicePointcloud()` like before.

> Some robotic applications save the points captured in a 4 channel OpenCV Mat object (cv::Mat).
> This object can be easily be converted to a classic array and then passed in the `arrayToPointcloud()` function explained before.
> More informations about cv::Mat to array conversion can be found here: https://stackoverflow.com/questions/26681713/convert-mat-to-array-vector-in-opencv


Here are shown two examples that should be useful in understanding better how all this works.


```c++
CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
initMap(h_map, dimX, dimY, dimZ, cellSize, freeVoxelsMargin, robotVoxelsHeight);

CudaGrid3D::Point* d_pointcloud;

int numPoints = 3;
CudaGrid3D::Point* h_pointcloud = (CudaGrid3D::Point*)malloc(sizeof(CudaGrid3D::Point) * numPoints);

CudaGrid3D::Point pt;
pt.x = 2.32;
pt.y = 1.26;
pt.z = 0.81;

h_pointcloud[0] = pt;
h_pointcloud[1] = pt;
h_pointcloud[2] = pt;

// to shorten the example the pointcloud consists of 3 identical points

// the host pointcloud must be converted to a device pointcloud
// d_pointcloud must be a pointer to a pointer (this is why we pass the address)
initDevicePointcloud(&d_pointcloud, h_pointcloud, numPoints);

// insert the points in the 3D Grid
insertPointcloud(h_map, d_pointcloud, numPoints);
```

```c++

// convert a cv::Mat to a classic array (easier to manipulate)
void cvMatToArray(cv::Mat* cv_matrix, float** result, int* result_length) {

    // convert cv::Mat to classic C array
    *result = cv_matrix->isContinuous() ? (float*)cv_matrix->data : (float*)cv_matrix->clone().data;
    *result_length = cv_matrix->total() * cv_matrix->channels();
}

void example(){

    CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
    initMap(h_map, dimX, dimY, dimZ, cellSize, freeVoxelsMargin, robotVoxelsHeight);

    CudaGrid3D::Point* d_pointcloud;

    float* h_arr;

    int length;
    
    // pointcloud_raw is the data received by a sensor / camera etc.
    cvMatToArray(&pointcloud_raw, &h_arr, &length);

    CudaTransform3D tf;

    CudaGrid3D::arrayToPointcloud(h_arr, length, &d_pointcloud, false, tf);

    // insert the points in the 3D Grid
    insertPointcloud(h_map, d_pointcloud, length / 4);
}
```



















