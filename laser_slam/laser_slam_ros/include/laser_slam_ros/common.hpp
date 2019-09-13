#ifndef LASER_SLAM_ROS_COMMON_HPP_
#define LASER_SLAM_ROS_COMMON_HPP_

#include <laser_slam/benchmarker.hpp>
#include <laser_slam/parameters.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace laser_slam_ros {

    typedef pcl::PointXYZ PclPoint;
    typedef pcl::PointCloud<PclPoint> PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;
    typedef pcl::PointXYZI PointI;
    typedef pcl::PointCloud<PointI> PointICloud;
    typedef PointICloud::Ptr PointICloudPtr;

    struct LaserSlamWorkerParams {
        // Map creation & filtering parameters.
        double distance_to_consider_fixed;
        bool separate_distant_map;
        bool create_filtered_map;
        double minimum_distance_to_add_pose;
        double voxel_size_m;
        int minimum_point_number_per_voxel;


        bool remove_ground_from_local_map = false;
        double ground_distance_to_robot_center_m;

        bool use_odometry_information = true;

        std::string path_to_export_trajectory;


        // Frames.
        std::string odom_frame;
        std::string sensor_frame;
        std::string world_frame;

        // Topics.
        std::string assembled_cloud_sub_topic;
        std::string trajectory_pub_topic;
        std::string odometry_trajectory_pub_topic;
        std::string full_map_pub_topic;
        std::string local_map_pub_topic;
        std::string distant_map_pub_topic;
        std::string get_laser_track_srv_topic;

        // Map publication.
        bool publish_local_map;
        bool publish_full_map;
        bool publish_distant_map;
        double map_publication_rate_hz;
    }; // struct LaserSlamWorkerParams

    static LaserSlamWorkerParams getLaserSlamWorkerParams(const ros::NodeHandle& nh,
                                                          const std::string& prefix) {
      LaserSlamWorkerParams params;
      const std::string ns = prefix + "/LaserSlamWorker";

      nh.getParam(ns + "/distance_to_consider_fixed", params.distance_to_consider_fixed);
      nh.getParam(ns + "/separate_distant_map", params.separate_distant_map);
      nh.getParam(ns + "/create_filtered_map", params.create_filtered_map);
      nh.getParam(ns + "/minimum_distance_to_add_pose", params.minimum_distance_to_add_pose);
      nh.getParam(ns + "/voxel_size_m", params.voxel_size_m);
      nh.getParam(ns + "/minimum_point_number_per_voxel", params.minimum_point_number_per_voxel);


      nh.getParam(ns + "/remove_ground_from_local_map", params.remove_ground_from_local_map);
      nh.getParam(ns + "/ground_distance_to_robot_center_m", params.ground_distance_to_robot_center_m);

      nh.getParam(ns + "/use_odometry_information", params.use_odometry_information);

      nh.getParam(ns + "/odom_frame", params.odom_frame);
      nh.getParam(ns + "/sensor_frame", params.sensor_frame);
      nh.getParam(ns + "/world_frame", params.world_frame);

      nh.getParam(ns + "/publish_local_map", params.publish_local_map);
      nh.getParam(ns + "/publish_full_map", params.publish_full_map);
      nh.getParam(ns + "/publish_distant_map", params.publish_distant_map);
      nh.getParam(ns + "/map_publication_rate_hz", params.map_publication_rate_hz);

      nh.getParam(ns + "/assembled_cloud_sub_topic", params.assembled_cloud_sub_topic);
      nh.getParam(ns + "/trajectory_pub_topic", params.trajectory_pub_topic);
      nh.getParam(ns + "/odometry_trajectory_pub_topic", params.odometry_trajectory_pub_topic);
      nh.getParam(ns + "/full_map_pub_topic", params.full_map_pub_topic);
      nh.getParam(ns + "/local_map_pub_topic", params.local_map_pub_topic);
      nh.getParam(ns + "/distant_map_pub_topic", params.distant_map_pub_topic);
      nh.getParam(ns + "/get_laser_track_srv_topic", params.get_laser_track_srv_topic);

      nh.getParam(ns + "/path_to_export_trajectory", params.path_to_export_trajectory);


      return params;
    }

    static laser_slam::LaserTrackParams getLaserTrackParams(const ros::NodeHandle& nh,
                                                            const std::string& prefix) {
      laser_slam::LaserTrackParams params;
      const std::string ns = prefix + "/LaserTrack";

      std::vector<float> odometry_noise_model, icp_noise_model;
      constexpr unsigned int kNoiseModelDimension = 6u;
      nh.getParam(ns + "/odometry_noise_model", odometry_noise_model);
      CHECK_EQ(odometry_noise_model.size(), kNoiseModelDimension);
      for (size_t i = 0u; i < 6u; ++i) {
        params.odometry_noise_model[i] = odometry_noise_model.at(i);
      }
      nh.getParam(ns + "/icp_noise_model", icp_noise_model);
      CHECK_EQ(icp_noise_model.size(), kNoiseModelDimension);
      for (size_t i = 0u; i < 6u; ++i) {
        params.icp_noise_model[i] = icp_noise_model.at(i);
      }
      nh.getParam(ns + "/add_m_estimator_on_odom", params.add_m_estimator_on_odom);
      nh.getParam(ns + "/add_m_estimator_on_icp", params.add_m_estimator_on_icp);

      // TODO move loading of icp_configuration_file and icp_input_filters_file to here.
      nh.getParam(ns + "/use_icp_factors", params.use_icp_factors);
      nh.getParam(ns + "/use_odom_factors", params.use_odom_factors);
      nh.getParam(ns + "/nscan_in_sub_map", params.nscan_in_sub_map);
      nh.getParam(ns + "/save_icp_results", params.save_icp_results);

      nh.getParam(ns + "/force_priors", params.force_priors);
      return params;
    }

    static laser_slam::EstimatorParams getOnlineEstimatorParams(const ros::NodeHandle& nh,
                                                                const std::string& prefix) {
      laser_slam::EstimatorParams params;
      const std::string ns = prefix + "/OnlineEstimator";

      std::vector<float>  loop_closure_noise_model;
      constexpr unsigned int kNoiseModelDimension = 6u;
      nh.getParam(ns + "/loop_closure_noise_model", loop_closure_noise_model);
      CHECK_EQ(loop_closure_noise_model.size(), kNoiseModelDimension);
      for (size_t i = 0u; i < 6u; ++i) {
        params.loop_closure_noise_model[i] = loop_closure_noise_model.at(i);
      }
      nh.getParam(ns + "/add_m_estimator_on_loop_closures", params.add_m_estimator_on_loop_closures);

      nh.getParam(ns + "/do_icp_step_on_loop_closures", params.do_icp_step_on_loop_closures);
      nh.getParam(ns + "/loop_closures_sub_maps_radius", params.loop_closures_sub_maps_radius);

      params.laser_track_params = getLaserTrackParams(nh, ns);

      return params;
    }

    static laser_slam::BenchmarkerParams getBenchmarkerParams(const ros::NodeHandle& nh,
                                                              const std::string& prefix) {
      laser_slam::BenchmarkerParams params;
      const std::string ns = prefix + "/Benchmarker";

      nh.getParam(ns + "/save_statistics_only", params.save_statistics_only);
      nh.getParam(ns + "/enable_live_output", params.enable_live_output);
      nh.getParam(ns + "/results_directory", params.results_directory);

      return params;
    }

    static PointCloud lpmToPcl(const laser_slam::PointMatcher::DataPoints& cloud_in) {
      PointCloud cloud_out;
      cloud_out.width = cloud_in.getNbPoints();
      cloud_out.height = 1;
      for (size_t i = 0u; i < cloud_in.getNbPoints(); ++i) {
        PclPoint point;
        point.x = cloud_in.features(0,i);
        point.y = cloud_in.features(1,i);
        point.z = cloud_in.features(2,i);
        cloud_out.push_back(point);
      }
      return cloud_out;
    }

    static void convert_to_pcl_point_cloud(const sensor_msgs::PointCloud2& cloud_message,
                                           PointICloud* converted) {
      pcl::PCLPointCloud2 pcl_point_cloud_2;
      pcl_conversions::toPCL(cloud_message, pcl_point_cloud_2);
      pcl::fromPCLPointCloud2(pcl_point_cloud_2, *converted);
    }

    template<typename PointCloudT>
    static void convert_to_point_cloud_2_msg(const PointCloudT& cloud,
                                             const std::string& frame,
                                             sensor_msgs::PointCloud2* converted) {
      CHECK_NOTNULL(converted);
      // Convert to PCLPointCloud2.
      pcl::PCLPointCloud2 pcl_point_cloud_2;
      pcl::toPCLPointCloud2(cloud, pcl_point_cloud_2);
      // Convert to sensor_msgs::PointCloud2.
      pcl_conversions::fromPCL(pcl_point_cloud_2, *converted);
      // Apply frame to msg.
      converted->header.frame_id = frame;
    }

    static void applyCylindricalFilter(const PclPoint& center, double radius_m,
                                       double height_m, bool remove_point_inside,
                                       PointCloud* cloud) {
      CHECK_NOTNULL(cloud);
      PointCloud filtered_cloud;

      const double radius_squared = pow(radius_m, 2.0);
      const double height_halved_m = height_m / 2.0;

      for (size_t i = 0u; i < cloud->size(); ++i) {
        if (remove_point_inside) {
          if ((pow(cloud->points[i].x - center.x, 2.0)
               + pow(cloud->points[i].y - center.y, 2.0)) >= radius_squared ||
              abs(cloud->points[i].z - center.z) >= height_halved_m) {
            filtered_cloud.points.push_back(cloud->points[i]);
          }
        } else {
          if ((pow(cloud->points[i].x - center.x, 2.0)
               + pow(cloud->points[i].y - center.y, 2.0)) <= radius_squared &&
              abs(cloud->points[i].z - center.z) <= height_halved_m) {
            filtered_cloud.points.push_back(cloud->points[i]);
          }
        }
      }

      filtered_cloud.width = 1;
      filtered_cloud.height = filtered_cloud.points.size();

      *cloud = filtered_cloud;
    }

} // namespace laser_slam_ros

#endif // LASER_SLAM_ROS_COMMON_HPP_
