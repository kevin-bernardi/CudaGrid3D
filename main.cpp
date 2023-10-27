#include <sys/time.h>

#include <iostream>
#include <opencv2/core/mat.hpp>

#include "vector3d.h"

float getTimeDifference(timeval t1, timeval t2) {
    float diff = ((t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec) / 1000.0;
    return diff;
}

int main(int argc, char *argv[]) {
    int dimX = 200;  // units
    int dimY = 100;
    int dimZ = 50;
    float resolution = 0.1;
    int numPoints = 10000;

    if (argc >= 6) {
        dimX = std::stoi(argv[1]);
        dimY = std::stoi(argv[2]);
        dimZ = std::stoi(argv[3]);
        resolution = std::stof(argv[4]);
        numPoints = std::stoi(argv[5]);
    }

    std::cout << dimX << " " << dimY << " " << dimZ << " " << resolution << " " << numPoints << std::endl;

    std::cout << "Esecuzione Vector3D da file cpp" << std::endl;

    Point *d_pointcloud;

    Vector3D *h_vector = new Vector3D;

    initVector(h_vector, dimX, dimY, dimZ, resolution);

    initDevicePointcloud(&d_pointcloud, numPoints);
    generateRandomPointcloud(h_vector, d_pointcloud, numPoints);

    insertPointcloud(h_vector, d_pointcloud, numPoints);

    generateMesh(h_vector, "./mesh.obj");
    generateSimpleMesh(h_vector, "./simple_mesh.obj");

    // float diff = getTimeDifference(t1, t2);
    // std::cout << "Computed test.cpp in : " << diff << " ms" << std::endl;
}