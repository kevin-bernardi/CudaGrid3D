#include <iostream>

#include "vector3d.h"

int main() {
    std::cout << "Esecuzione Vector3D da file cpp" << std::endl;

    int dimX = 200;
    int dimY = 100;
    int dimZ = 50;
    float resolution = 0.1;
    int numPoints = 1000;

    test(dimX, dimY, dimZ, resolution, numPoints);
}