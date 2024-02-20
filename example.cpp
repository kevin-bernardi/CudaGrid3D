#include <sys/time.h>

#include <iostream>

#include "grid3d.h"

using namespace cv;
using namespace CudaGrid3D;

int main(int argc, char* argv[]) {
    float dimX = 5;  // meters
    float dimY = 5;
    float dimZ = 5;
    float cellSize = 0.1;
    float floorMargin = 1;
    float robotHeight = 2;
    int numPoints = 300;

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
    initMap(h_map, dimX, dimY, dimZ, cellSize, floorMargin, robotHeight);

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

    clusterFrontiers3D(h_map, 1, ori, &centroid, &cluster, &sizeCluster);

    std::cout << "print in example" << std::endl;

    // for (int i = 0; i < 10; i++) {
    //     std::cout << "pt " << cluster[i].x << " " << cluster[i].y << " " << cluster[i].z << std::endl;
    // }

    // std::cout << "size cluster: " << sizeCluster << std::endl;
    // std::cout << "centroid: " << centroid.x << " " << centroid.y << " " << centroid.z << std::endl;

    // Mat res = getGrid2D(h_map, 10, 55, 85);
    // namedWindow("Image Test OpenCV", WINDOW_AUTOSIZE);
    // imshow("Image Test OpenCV", res);
    // waitKey(0);

    freeMap(h_map);
}
