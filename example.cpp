#include <sys/time.h>

#include <iostream>

#include "grid3d.h"

using namespace cv;
using namespace CudaGrid3D;

int main(int argc, char* argv[]) {
    float dimX = 10;  // meters
    float dimY = 10;  // meters
    float dimZ = 10;  // meters
    float ox = 5;     // meters
    float oy = 5;     // meters
    float cellSize = 0.1;
    float floorMargin = 0.2;
    float robotHeight = 0.4;
    int numPoints = 10000;

    if (argc == 8) {
        dimX = std::stoi(argv[1]);
        dimY = std::stoi(argv[2]);
        dimZ = std::stoi(argv[3]);
        cellSize = std::stof(argv[4]);
        floorMargin = std::stof(argv[5]);
        robotHeight = std::stof(argv[6]);
        numPoints = std::stoi(argv[7]);

        std::cout << "launched with arguments" << std::endl;
    }

    CudaGrid3D::Point ori;
    ori.x = 0.0;
    ori.y = 0.0;
    ori.z = 0.0;

    CudaGrid3D::Map* h_map = new CudaGrid3D::Map;
    initMap(h_map, dimX, dimY, dimZ, ox, oy, cellSize, floorMargin, robotHeight);

    std::cout << h_map->dimX << " " << h_map->dimY << " " << h_map->dimZ << std::endl;
    std::cout << h_map->floorMargin << " " << h_map->robotHeight << std::endl;

    CudaGrid3D::Point* d_pointcloud;

    generateRandomPointcloud(h_map, &d_pointcloud, numPoints);
    insertPointcloud(h_map, d_pointcloud, numPoints);

    pointcloudRayTracing(h_map, d_pointcloud, numPoints, ori, false);
    findFrontiers3D(h_map);
    updateGrid2D(h_map, 10, 50, 75);

    generateSimpleMesh3D(h_map, "./mesh.obj", FRONTIER_MAP);

    CudaGrid3D::Point centroid;
    IntPoint* cluster;
    int sizeCluster;

    clusterFrontiers3D(h_map, 0.5, ori, &centroid, &cluster, &sizeCluster);

    std::cout << "size best cluster: " << sizeCluster << std::endl;
    std::cout << "centroid best cluster: " << centroid.x << "m " << centroid.y << "m " << centroid.z << "m" << std::endl;

    inflateObstacles2D(h_map, 0.2, 10, 55, 85);

    CudaGrid3D::Point bestObsPoint = bestObservationPoint2D(h_map, centroid, cluster, sizeCluster, 1, 15, 10, 0.75);

    Mat res = getGrid2D(h_map, 10, 55, 85);
    namedWindow("Inflated 2D Map", WINDOW_AUTOSIZE);
    imshow("Inflated 2D Map", res);
    waitKey(0);

    freeMap(h_map);

    std::cout << "Best obs point in example.cpp: x:" << bestObsPoint.x << "m y:" << bestObsPoint.y << "m z:" << bestObsPoint.z << "m" << std::endl;
}
