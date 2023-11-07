#include <sys/time.h>

#include <iostream>
#include <opencv2/core/mat.hpp>

#include "vector3d.h"

float getTimeDifference(timeval t1, timeval t2) {
    float diff = ((t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec) / 1000.0;
    return diff;
}

int main(int argc, char *argv[]) {
    int dimX = 1000;  // units
    int dimY = 1000;
    int dimZ = 50;
    float resolution = 0.1;
    int numPoints = 1000;

    if (argc >= 6) {
        dimX = std::stoi(argv[1]);
        dimY = std::stoi(argv[2]);
        dimZ = std::stoi(argv[3]);
        resolution = std::stof(argv[4]);
        numPoints = std::stoi(argv[5]);
    }

    std::cout << dimX << " " << dimY << " " << dimZ << " " << resolution << " " << numPoints << std::endl;

    std::cout << "Esecuzione Vector3D da file cpp" << std::endl;

    Vector3D *h_vector = new Vector3D;

    initVector(h_vector, dimX, dimY, dimZ, resolution);

    Point *h_pointcloud = new Point[2];
    Point *d_pointcloud;

    initDevicePointcloud(&d_pointcloud, numPoints);
    // generateRandomPointcloud(h_vector, d_pointcloud, numPoints);

    Point ori;
    ori.x = -0.5;
    ori.y = -0.5;
    ori.z = 0;

    Point pt;
    pt.x = 0.1;
    pt.y = -0.1;
    pt.z = 0.1;

    h_pointcloud[0] = pt;

    initDevicePointcloud(&d_pointcloud, h_pointcloud, 1);

    Vector3D *vct_extra = new Vector3D;
    initVector(vct_extra, dimX / 2, dimY / 2, 2, resolution);

    insertPointcloud(h_vector, d_pointcloud, numPoints);

    int *result = new int[3];

    getCoordinatesInv(h_vector, 1501499, result);

    std::cout << "inv coords: " << result[0] << " " << result[1] << " " << result[2] << std::endl;

    generateMesh(h_vector, "./mesh.obj");
    generateSimpleMesh(h_vector, "./simple_mesh.obj");
    // printGrid(h_vector);

    // float diff = getTimeDifference(t1, t2);
    // std::cout << "Computed test.cpp in : " << diff << " ms" << std::endl;

    pointcloudRayTracing(h_vector, d_pointcloud, numPoints, ori);
    // generateMesh(h_vector, "./mesh.obj");

    // printGrid(h_vector);
}