cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(epnp_opencv main.cc)
target_link_libraries(epnp_opencv ${OpenCV_LIBS})
