#include <sys/time.h>

#include <iostream>
#include <opencv2/core/mat.hpp>

#include "vector3d.h"

float getTimeDifference(timeval t1, timeval t2) {
    float diff = ((t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec) / 1000.0;
    return diff;
}

void cvMatToArray(cv::Mat* cv_matrix, float** result, int* result_length) {
    // convert cv::Mat to classic C array
    *result = cv_matrix->isContinuous() ? (float*)cv_matrix->data : (float*)cv_matrix->clone().data;
    *result_length = cv_matrix->total() * cv_matrix->channels();
}

int main(int argc, char* argv[]) {
    int dimX = 10;  // units
    int dimY = 10;
    int dimZ = 3;
    float resolution = 0.1;
    int freeVoxelsMargin = 1;
    int robotVoxelsHeight = 2;
    int numPoints = 10;

    if (argc >= 8) {
        dimX = std::stoi(argv[1]);
        dimY = std::stoi(argv[2]);
        dimZ = std::stoi(argv[3]);
        resolution = std::stof(argv[4]);
        freeVoxelsMargin = std::stof(argv[5]);
        robotVoxelsHeight = std::stof(argv[6]);
        numPoints = std::stoi(argv[7]);
    }

    Map* h_map = new Map;
    initMap(h_map, dimX, dimY, dimZ, resolution, freeVoxelsMargin, robotVoxelsHeight);

    int cooInv[2];

    Point* h_pointcloud = new Point[3];
    Point pt1;
    pt1.x = -0.1;
    pt1.y = -0.1;
    pt1.z = 0;

    Point pt2;
    pt2.x = -0.20;
    pt2.y = -0.20;
    pt2.z = 0.1;

    Point pt3;
    pt3.x = 0.1;
    pt3.y = 0.1;
    pt3.z = 0.2;

    h_pointcloud[0] = pt1;
    h_pointcloud[1] = pt2;
    h_pointcloud[2] = pt3;

    Point ori;
    ori.x = -0.2;
    ori.y = -0.2;
    ori.z = 0;

    Point* d_pointcloud;

    initDevicePointcloud(&d_pointcloud, h_pointcloud, 3);
    insertPointcloud(h_map, d_pointcloud, 3);

    // pointcloudRayTracing(h_map, d_pointcloud, 3, ori);

    printGrid3D(h_map);
    printGrid2D(h_map);

    generateMeshGrid2D(h_map, "suino.obj");
}

// Point* d_pointcloud;

//     initDevicePointcloud(&d_pointcloud, numPoints);
//     generateRandomPointcloud(h_map, d_pointcloud, numPoints);
//     insertPointcloud(h_map, d_pointcloud, numPoints);
//     printGrid3D(h_map);

//     Point ori;
//     ori.x = -0.5;
//     ori.y = -0.5;
//     ori.z = 0;

//     pointcloudRayTracing(h_map, d_pointcloud, numPoints, ori);
//     freeDevicePointcloud(d_pointcloud);

//     printGrid3D(h_map);

//     freeMap(h_map);