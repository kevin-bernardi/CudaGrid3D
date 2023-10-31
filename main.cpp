#include <sys/time.h>

#include <iostream>
#include <opencv2/core/mat.hpp>

#include "vector3d.h"

float getTimeDifference(timeval t1, timeval t2) {
    float diff = ((t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec) / 1000.0;
    return diff;
}

int main(int argc, char *argv[]) {
    int dimX = 5;  // units
    int dimY = 5;
    int dimZ = 2;
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

    Point *h_pointcloud = new Point[2];
    Point *d_pointcloud;

    Point pt1;
    pt1.x = -0.29;
    pt1.y = -0.23;
    pt1.z = 0;

    Point pt2;
    pt2.x = 0.2;
    pt2.y = 0.2;
    pt2.z = 0;

    h_pointcloud[0] = pt1;
    h_pointcloud[1] = pt2;

    Vector3D *h_vector = new Vector3D;

    initVector(h_vector, dimX, dimY, dimZ, resolution);

    initDevicePointcloud(&d_pointcloud, h_pointcloud, 2);

    insertPointcloud(h_vector, d_pointcloud, 2);

    // generateMesh(h_vector, "./mesh.obj");
    // generateSimpleMesh(h_vector, "./simple_mesh.obj");
    printGrid(h_vector);

    // float diff = getTimeDifference(t1, t2);
    // std::cout << "Computed test.cpp in : " << diff << " ms" << std::endl;

    ray_tracing(h_vector, pt1, pt2);
    printGrid(h_vector);
}