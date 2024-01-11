#include <sys/time.h>

#include <iostream>

#include "grid3d.h"

using namespace cv;

float getTimeDifference(timeval t1, timeval t2) {
    float diff = ((t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec) / 1000.0;
    return diff;
}

// void showcvImage() {
//     Mat img(500, 500, CV_8UC3, Scalar(0, 0, 0));
//     namedWindow("Image Test OpenCV", WINDOW_AUTOSIZE);
//     imshow("Image Test OpenCV", img);
//     waitKey(0);
//     return;
// }

int main(int argc, char* argv[]) {
    int dimX = 500;  // units
    int dimY = 500;
    int dimZ = 500;
    float resolution = 0.1;
    int freeVoxelsMargin = 1;
    int robotVoxelsHeight = 5;
    int numPoints = 10000;

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
    updateGrid2D(h_map, 10, 50, 75);

    Mat res = getGrid2D(h_map, 10, 55, 85);
    namedWindow("Image Test OpenCV", WINDOW_AUTOSIZE);
    imshow("Image Test OpenCV", res);
    waitKey(0);

    generateMesh(h_map, "./compl_mesh.obj");

    // generateMeshGrid2D(h_map, "suino.obj");
}
