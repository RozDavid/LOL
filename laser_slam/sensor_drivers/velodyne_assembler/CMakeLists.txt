cmake_minimum_required(VERSION 2.8.3)
project(velodyne_assembler)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_simple()

add_definitions(-std=c++11)

cs_add_library(velodyne_assembler src/velodyne_assembler_ros.cpp)

cs_add_executable(velodyne_assembler_node src/velodyne_assembler_node.cpp)
target_link_libraries(velodyne_assembler_node velodyne_assembler)

cs_export()


