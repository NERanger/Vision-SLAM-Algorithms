cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(pnp_ba_gaussnewton main.cc)
target_link_libraries(pnp_ba_gaussnewton ${OpenCV_LIBS})
