cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

add_executable(pnp_gaussnewton main.cc)
target_link_libraries(pnp_gaussnewton ${OpenCV_LIBS})
