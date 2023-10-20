#include <iostream>

#include "vector3d.h"

int main() {
    std::cout << "Esecuzione Vector3D da file cpp" << std::endl;

    int dimX = 2;  // units
    int dimY = 2;
    int dimZ = 2;
    float resolution = 0.1;
    int numPoints = 2;

    bool memoryDebug = false;

    Vector3D* vector = new Vector3D;

    Point* h_pointcloud = new Point[numPoints];
    Point* d_pointcloud;

    // test(dimX, dimY, dimZ, resolution, numPoints, memoryDebug);

    initVector(vector, dimX, dimY, dimZ, resolution);

    Point pt1;
    pt1.x = 0.02;
    pt1.y = 0.06;
    pt1.z = 0.09;

    Point pt2;
    pt2.x = 0.17;
    pt2.y = 0.18;
    pt2.z = 0.19;

    h_pointcloud[0] = pt1;
    h_pointcloud[1] = pt2;

    initDevicePointcloud(&d_pointcloud, h_pointcloud, numPoints);

    printPointcloud(d_pointcloud, numPoints);

    // generateRandomPointcloud(vector, d_pointcloud, numPoints);

    // checkDuplicates(vector, d_pointcloud, numPoints);

    insertPointcloud(vector, d_pointcloud, numPoints);

    generateMesh(vector, "./gino.obj");

    // free allocated memory

    freeVector(vector);
    freeDevicePointcloud(d_pointcloud);

    free(h_pointcloud);
}