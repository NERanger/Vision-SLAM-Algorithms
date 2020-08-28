# Minimum cmake version required
cmake_minimum_required(VERSION 2.8)

# Declare a cmake project
project(Eigen_geometric_module)

find_package(Eigen3)

include_directories(${EIGEN3_INCLUDE_DIR})

# Add an executable
add_executable(Eigen_geometric_module main.cc)