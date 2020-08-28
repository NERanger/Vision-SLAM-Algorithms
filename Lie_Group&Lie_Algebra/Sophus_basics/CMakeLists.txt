cmake_minimum_required(VERSION 2.8)

find_package(Sophus REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

add_executable(Sophus_basics main.cc)
target_link_libraries(Sophus_basics ${Sophus_LIBRARIES})