cmake_minimum_required(VERSION 3.1)
project(Sophus_basics)

find_package(Sophus REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(Sophus_basics main.cc)
target_link_libraries(Sophus_basics Sophus::Sophus)