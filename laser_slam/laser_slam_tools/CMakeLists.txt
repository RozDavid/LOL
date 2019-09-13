cmake_minimum_required(VERSION 2.8.3)
project(laser_slam_tools)

add_definitions(-std=c++11)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
    roscpp
)

catkin_simple(ALL_DEPS_REQUIRED)

catkin_package()

cs_add_executable(octomap_to_point_cloud src/octomap_to_point_cloud.cpp)
cs_add_executable(laser_to_octomap src/laser_to_octomap.cpp)

cs_install()
cs_export()

