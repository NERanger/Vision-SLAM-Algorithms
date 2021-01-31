cmake_minimum_required(VERSION 3.1)
project(trajectory_error)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Sophus REQUIRED)

# Need to update when running on different machine
include_directories(/home/artinx001/3rdparty/eigen-3.3.8)

#MESSAGE(STATUS ${EIGEN3_INCLUDE_DIRS})

add_executable(trajectory_error main.cc)
target_link_libraries(trajectory_error Sophus::Sophus ${Pangolin_LIBRARIES})