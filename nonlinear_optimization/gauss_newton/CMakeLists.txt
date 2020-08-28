cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(gauss_newton main.cc)
target_link_libraries(gauss_newton ${OpenCV_LIBS})