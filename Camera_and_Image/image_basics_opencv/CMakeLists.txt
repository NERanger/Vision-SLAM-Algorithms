set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(image_basics_opencv main.cc)

target_link_libraries(image_basics_opencv ${OpenCV_LIBS})