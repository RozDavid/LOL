// VelodyneAssembler was originally developed by Philipp Kruesi.

#ifndef VELODYNE_ASSEMBLER_VELODYNE_ASSEMBLER_ROS_H_
#define VELODYNE_ASSEMBLER_VELODYNE_ASSEMBLER_ROS_H_

#include <memory>

#include <boost/shared_ptr.hpp>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

class VelodyneAssemblerRos {
  typedef PointMatcher<float> PM;
  typedef PM::DataPoints DataPoints;

  public:
    VelodyneAssemblerRos();
    ~VelodyneAssemblerRos();

    /// \brief Initialize assembler (to be called immediately after the constructor).
    bool init();

  private:

    /// Callback assembling scans to full revolution point cloud (with tf messages).
    void pclCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg);

    /// \brief Retrieve parameters.
    bool getParams();

    /// Subscribers.
    ros::Subscriber pcl_sub_;

    /// Transform communication.
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    /// Publishers.
    ros::Publisher assembled_cloud_publisher_;

    /// Node handle.
    ros::NodeHandle nh_;

    // Assemble point clouds without IMU and odometry when naive_assembling_ is true.
    bool naive_assembling_;

    // Frame ID of the fixed frame (inertial).
    std::string fixed_frame_id_;
    // Frame ID of the frame attached to the center (of rotation) of the robot at ground height.
    std::string vehicle_base_frame_id_;
    // Frame ID of the sensor (the point clouds are assumed to be in this frame).
    std::string sensor_frame_id_;

    // Raw/input point cloud.
    std::string raw_pcl_;
    // Assembled point cloud.
    std::string assembled_pcl_;

    std::shared_ptr<DataPoints> current_assembled_cloud_;
    std::shared_ptr<PM::Transformation> transformation_;

    float last_azimuth_rad_;
    ros::Time last_stamp_;
    bool initialized_;

    PM::TransformationParameters T_sensor_base_, T_base_sensor_;
    PM::TransformationParameters T_fixed_basePrevious_;
    PM::TransformationParameters T_sensorStart_sensorCurrent_;

    static constexpr unsigned int kScanSubscriberMessageQueueSize = 2000u;
}; // VelodyneAssemblerRos

#endif /* VELODYNE_ASSEMBLER_VELODYNE_ASSEMBLER_ROS_H_ */
