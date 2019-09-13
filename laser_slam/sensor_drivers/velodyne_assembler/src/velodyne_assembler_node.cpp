// VelodyneAssembler was originally developed by Philipp Kruesi.

#include <ros/ros.h>

#include "velodyne_assembler/velodyne_assembler_ros.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "VelodyneAssembler");

  VelodyneAssemblerRos velodyne_assembler;

  if(velodyne_assembler.init()) {
    try {
      ros::spin();
    }
    catch (const std::exception& e) {
      ROS_ERROR_STREAM("Exception: " << e.what());
      return 1;
    }
    catch (...) {
      ROS_ERROR_STREAM("Unknown Exception");
      return 1;
    }
  }

  return 0;
}
