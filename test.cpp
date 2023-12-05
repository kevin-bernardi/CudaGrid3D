#include <sys/time.h>

#include <iostream>

#include "grid3d.h"

float getTimeDifference(timeval t1, timeval t2) {
    float diff = ((t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec) / 1000.0;
    return diff;
}

int main(int argc, char* argv[]) {
    int dimX = 10;  // units
    int dimY = 10;
    int dimZ = 10;
    float resolution = 0.1;
    int freeVoxelsMargin = 1;
    int robotVoxelsHeight = 5;
    int numPoints = 1000;

    if (argc >= 8) {
        dimX = std::stoi(argv[1]);
        dimY = std::stoi(argv[2]);
        dimZ = std::stoi(argv[3]);
        resolution = std::stof(argv[4]);
        freeVoxelsMargin = std::stof(argv[5]);
        robotVoxelsHeight = std::stof(argv[6]);
        numPoints = std::stoi(argv[7]);

        std::cout << "started with arguments" << std::endl;
    }

    CudaGrid3D::Point ori;
    ori.x = 0.0;
    ori.y = 0.0;
    ori.z = 0.0;

    CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
    initMap(h_map, dimX, dimY, dimZ, resolution, freeVoxelsMargin, robotVoxelsHeight);

    CudaGrid3D::Point* d_pointcloud;

    initDevicePointcloud(&d_pointcloud, numPoints);
    generateRandomPointcloud(h_map, d_pointcloud, numPoints);
    insertPointcloud(h_map, d_pointcloud, numPoints);

    pointcloudRayTracing(h_map, d_pointcloud, numPoints, ori);
    updateGrid2D(h_map, 50, 75);

    // printGrid3D(h_map);
    printGrid2D(h_map);

    visualizeAndSaveGrid2D(h_map, "map2d.bmp", false, 10, 55, 85);
    CudaGrid3D::gridBinning(h_map, 4, 10, 85);

    // generateMeshGrid2D(h_map, "suino.obj");
}

// int cooInv[2];

// Point* h_pointcloud = new Point[3];
// Point pt1;
// pt1.x = -0.1;
// pt1.y = -0.1;
// pt1.z = 0.1;

// Point pt2;
// pt2.x = -0.20;
// pt2.y = -0.20;
// pt2.z = 0.1;

// Point pt3;
// pt3.x = 0.1;
// pt3.y = 0.1;
// pt3.z = 0.2;

// h_pointcloud[0] = pt1;
// h_pointcloud[1] = pt2;
// h_pointcloud[2] = pt3;
