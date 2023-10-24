#include <sys/time.h>

#include <iostream>
#include <opencv2/core/mat.hpp>

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

    Point* d_pointcloud;

    Transform3D tf;

    tf.setTranslation(1, 1, 1);
    tf.setRPY(0, 0, 0);

    cv::Mat mat = cv::Mat::zeros(20, 20, CV_32FC4);
    mat.at<cv::Vec4f>(0, 0) = cv::Vec4f(1.1, 1.1111, 2.0, 3.0);
    mat.at<cv::Vec4f>(1, 1) = cv::Vec4f(1.6, 1.3248, 2.0, 3.332132);
    mat.at<cv::Vec4f>(5, 1) = cv::Vec4f(1.6, 1.3248, 2.0, 3.332132);
    mat.at<cv::Vec4f>(1, 5) = cv::Vec4f(1.6, 1.3248, 2.0, 3.332132);

    std::cout << std::endl;

    // initDevicePointcloud(&d_pointcloud, numPoints);

    insertCvMatToPointcloud(&mat, &d_pointcloud, tf);

    int matCells = mat.rows * mat.cols;

    printPointcloud(d_pointcloud, matCells);

    // float diff = getTimeDifference(t1, t2);

    // std::cout << "Computed test.cpp in : " << diff << " ms" << std::endl;
}