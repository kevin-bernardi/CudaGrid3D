#include <sys/time.h>

#include <iostream>
#include <opencv2/core/mat.hpp>

#include "vector3d.h"

float getTimeDifference(timeval t1, timeval t2) {
    float diff = ((t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec) / 1000.0;
    return diff;
}

int main(int argc, char *argv[]) {
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

    Point *d_pointcloud;

    CudaTransform3D tf;

    // tf intiialization
    tf.tra[0] = 1;
    tf.tra[1] = 2;
    tf.tra[2] = 3;

    tf.rot[0][1] = 0.0;
    tf.rot[0][0] = 1.0;
    tf.rot[0][2] = 0.0;
    tf.rot[1][0] = 0.0;
    tf.rot[1][1] = 1.0;
    tf.rot[1][2] = 0.0;
    tf.rot[2][0] = 0.0;
    tf.rot[2][1] = 0.0;
    tf.rot[2][2] = 1.0;

    cv::Mat mat = cv::Mat::zeros(2, 2, CV_32FC4);
    mat.at<cv::Vec4f>(0, 0) = cv::Vec4f(1.1, 1.1111, 2.0, 3.0);
    mat.at<cv::Vec4f>(1, 1) = cv::Vec4f(1.6, 1.3248, 2.0, 3.332132);

    std::cout << std::endl;

    // convert cv::Mat to classic C array
    float *mat_arr = mat.isContinuous() ? (float *)mat.data : (float *)mat.clone().data;
    uint length = mat.total() * mat.channels();

    printf("Converted array size: %d\n", length);

    for (int i = 0; i < length; i++) {
        printf("%f |", mat_arr[i]);
    }
    printf("\n");

    insertCvMatToPointcloud(mat_arr, length, &d_pointcloud, tf);

    int matCells = mat.rows * mat.cols;

    printPointcloud(d_pointcloud, matCells);

    // float diff = getTimeDifference(t1, t2);

    // std::cout << "Computed test.cpp in : " << diff << " ms" << std::endl;
}