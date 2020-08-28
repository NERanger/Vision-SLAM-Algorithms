find_package(Pangolin REQUIRED)
find_package(Eigen3)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(plot_trajectory main.cc)

target_link_libraries(plot_trajectory ${Pangolin_LIBRARIES})