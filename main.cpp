#include <sys/time.h>

#include <iostream>

#include "vector3d.h"

float getTimeDifference(timeval t1, timeval t2) {
    float diff = ((t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec) / 1000.0;
    return diff;
}

int main(int argc, char* argv[]) {
    int dimX = 100000;  // units
    int dimY = 100000;
    int dimZ = 100000;
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

    struct timeval t1, t2, t1_mesh, t2_mesh;

    gettimeofday(&t1, nullptr);

    Vector3D* vector = new Vector3D;

    Point* d_pointcloud;

    initVector(vector, dimX, dimY, dimZ, resolution);

    initDevicePointcloud(&d_pointcloud, numPoints);

    generateRandomPointcloud(vector, d_pointcloud, numPoints);

    insertPointcloud(vector, d_pointcloud, numPoints);

    gettimeofday(&t2, nullptr);

    float diff = getTimeDifference(t1, t2);

    std::cout << "Generated the grid in : " << diff << " ms" << std::endl;

    // gettimeofday(&t1_mesh, nullptr);
    // generateMesh(vector, "./mesh.obj");
    // gettimeofday(&t2_mesh, nullptr);

    // diff = getTimeDifference(t1_mesh, t2_mesh);
    // std::cout << "Generated the mesh in : " << diff << " ms" << std::endl;

    // free allocated memory

    freeVector(vector);
    freeDevicePointcloud(d_pointcloud);
}