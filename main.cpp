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
    int dimZ = 1;
    float resolution = 0.1;
    int numPoints = 10;

    if (argc >= 6) {
        dimX = std::stoi(argv[1]);
        dimY = std::stoi(argv[2]);
        dimZ = std::stoi(argv[3]);
        resolution = std::stof(argv[4]);
        numPoints = std::stoi(argv[5]);
    }

    std::cout << dimX << " " << dimY << " " << dimZ << " " << resolution << " " << numPoints << std::endl;

    Vector3D* h_vector = new Vector3D;
    initVector(h_vector, dimX, dimY, dimZ, resolution);

    Point* d_pointcloud;

    initDevicePointcloud(&d_pointcloud, numPoints);
    generateRandomPointcloud(h_vector, d_pointcloud, numPoints);
    insertPointcloud(h_vector, d_pointcloud, numPoints);
    // printGrid(h_vector);

    Point ori;
    ori.x = 0;
    ori.y = 0;
    ori.z = 0;

    pointcloudRayTracing(h_vector, d_pointcloud, numPoints, ori);

    // printGrid(h_vector);
}