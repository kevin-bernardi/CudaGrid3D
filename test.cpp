#include <iostream>

#include "vector3d.h"

int main() {
    std::cout << "Esecuzione Vector3D da file cpp" << std::endl;

    int dimX = 5;  // units
    int dimY = 5;
    int dimZ = 2;
    float resolution = 0.1;
    int numPoints = 10;

    bool memoryDebug = false;

    Vector3D* vector = new Vector3D;

    Point* h_pointcloud = new Point[10];
    Point* d_pointcloud;

    test(dimX, dimY, dimZ, resolution, numPoints, memoryDebug);

    // initVector(vector, dimX, dimY, dimZ);

    // initPointcloud(d_pointcloud, 10);

    // generateRandomPc(vector, d_pointcloud, 10, resolution);

    // copyPCToHost(h_pointcloud, d_pointcloud, 10);

    // for (int i = 0; i < 10; i++) {
    //     Point pt = h_pointcloud[i];
    //     printf("Point %d: (%f,%f,%f)\n", i, pt.x, pt.y, pt.z);
    // }
}