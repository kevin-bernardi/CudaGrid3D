#include <sys/time.h>

#include <iostream>

#include "grid3d.h"

using namespace cv;
using namespace CudaGrid3D;

int main(int argc, char* argv[]) {
    int dimX = 1000;  // units
    int dimY = 1000;
    int dimZ = 1000;
    float cellSize = 0.1;
    int freeVoxelsMargin = 1;
    int robotVoxelsHeight = 2;
    int numPoints = 10000;

    if (argc >= 8) {
        dimX = std::stoi(argv[1]);
        dimY = std::stoi(argv[2]);
        dimZ = std::stoi(argv[3]);
        cellSize = std::stof(argv[4]);
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
    initMap(h_map, dimX, dimY, dimZ, cellSize, freeVoxelsMargin, robotVoxelsHeight);

    CudaGrid3D::Point* d_pointcloud;

    generateRandomPointcloud(h_map, &d_pointcloud, numPoints);
    insertPointcloud(h_map, d_pointcloud, numPoints);

    pointcloudRayTracing(h_map, d_pointcloud, numPoints, ori, false);
    findFrontiers3D(h_map);
    updateGrid2D(h_map, 10, 50, 75);

    // Mat res = getGrid2D(h_map, 10, 55, 85);
    // namedWindow("Image Test OpenCV", WINDOW_AUTOSIZE);
    // imshow("Image Test OpenCV", res);
    // waitKey(0);

    // generateSimpleMesh3D(h_map, "./mesh.obj", FRONTIER_MAP);
    // printDeviceGrid3D(h_map);

    std::cout << "Num frontiers: " << h_map->numFrontiers_3D << std::endl;
}
