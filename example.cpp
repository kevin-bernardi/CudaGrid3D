#include <sys/time.h>

#include <iostream>

#include "grid3d.h"

using namespace cv;
using namespace CudaGrid3D;

int main(int argc, char* argv[]) {
    float dimX = 1;  // meters
    float dimY = 1;  // meters
    float dimZ = 1;  // meters
    float ox = 0;    // meters
    float oy = 0;    // meters
    float oz = 0;    // meters
    float cellSize = 0.1;
    float floorMargin = 0.2;
    float robotHeight = 0.4;
    int numPoints = 40;

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
    initMap(h_map, dimX, dimY, dimZ, ox, oy, oz, cellSize, floorMargin, robotHeight);

    std::cout << h_map->dimX << " " << h_map->dimY << " " << h_map->dimZ << std::endl;
    std::cout << h_map->floorMargin << " " << h_map->robotHeight << std::endl;

    CudaGrid3D::Point* d_pointcloud;

    generateRandomPointcloud(h_map, &d_pointcloud, numPoints);
    insertPointcloud(h_map, d_pointcloud, numPoints);

    pointcloudRayTracing(h_map, d_pointcloud, numPoints, ori, false);
    findFrontiers3D(h_map);
    updateGrid2D(h_map, 10, 55, 85, 50, 75, 0);

    generateSimpleMesh3D(h_map, "./mesh.obj", OCCUPANCY_MAP);

    CudaGrid3D::Point centroid;
    IntPoint* cluster;
    int sizeCluster;

    clusterFrontiers3D(h_map, 0.5, ori, &centroid, &cluster, &sizeCluster);

    std::cout << "size best cluster: " << sizeCluster << std::endl;
    std::cout << "centroid best cluster: " << centroid.x << "m " << centroid.y << "m " << centroid.z << "m" << std::endl;

    // inflateObstacles2D(h_map, 0.2, 10, 55, 85);

    CudaGrid3D::BestObservation bestObs = bestObservationPoint(h_map, centroid, cluster, sizeCluster, 1, 15, 10, 0.5, 2, 0.1);

    // Mat res = getGrid2D(h_map, 10, 55, 85);
    // namedWindow("Inflated 2D Map", WINDOW_AUTOSIZE);
    // imshow("Inflated 2D Map", res);
    // waitKey(0);

    std::cout << "Best obs point in example.cpp: x:" << bestObs.point.x << "m y:" << bestObs.point.y << "m z:" << bestObs.point.z << "m" << std::endl;
    std::cout << "pitch: " << bestObs.pitch << " yaw: " << bestObs.yaw << std::endl;

    freeMap(h_map);
}
