find_package(OpenCV REQUIRED)
include_directories(${Opencv_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Opencv_INCLUDE_DIRS})

find_package(PCL REQUIRED COMPONENT common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(join_pointcloud main.cc)
target_link_libraries(join_pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES})
