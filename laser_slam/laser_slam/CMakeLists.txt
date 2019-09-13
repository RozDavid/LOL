cmake_minimum_required(VERSION 2.8.3)
project(laser_slam)

add_definitions(-std=c++11)

find_package(catkin_simple 0.1.0 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
  src/laser_track.cpp
  src/incremental_estimator.cpp
  src/benchmarker.cpp
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_add_gtest(${PROJECT_NAME}_tests 
  test/test_main.cpp
  test/test_empty.cpp
WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)

target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

add_doxygen(REQUIRED)

cs_install()
cs_export()

