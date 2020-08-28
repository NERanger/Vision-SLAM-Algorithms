#include <iostream>
#include "opencv2/core.hpp"
#include "Eigen/Core"

int main(){
    Eigen::Matrix3d eigen_mat_3d;
    eigen_mat_3d << 1, 2, 3, 4, 5, 6, 7, 8, 9;

    cv::Mat cv_mat_3d;
    cv_mat_3d = (cv::Mat_<int>(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);

    std::cout << "When query an element in a martix either by Eigen3 or OpenCV, the first is the row number, the second is the column number" << std::endl;
    std::cout << "3D matrix in Eigen3:\n" << eigen_mat_3d << std::endl;
    std::cout << "Element at (1, 0):\n" << eigen_mat_3d(1, 0) << std::endl;
    std::cout << "3D matrix in OpenCV:\n" << cv_mat_3d << std::endl;
    std::cout << "Element at (1, 0):\n" << cv_mat_3d.at<int>(1, 0) << std::endl;

} 