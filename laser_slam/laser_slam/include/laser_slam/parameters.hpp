#ifndef LASER_SLAM_PARAMETERS_HPP_
#define LASER_SLAM_PARAMETERS_HPP_

#include <Eigen/Dense>

namespace laser_slam {

struct LaserTrackParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double,6,1> odometry_noise_model;
  Eigen::Matrix<double,6,1> icp_noise_model;
  bool add_m_estimator_on_odom;
  bool add_m_estimator_on_icp;

  std::string icp_configuration_file;
  std::string icp_input_filters_file;
  bool use_icp_factors;
  bool use_odom_factors;
  int nscan_in_sub_map;
  bool save_icp_results;

  bool force_priors;
}; // struct LaserTrackParams

struct EstimatorParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double,6,1> loop_closure_noise_model;
  bool add_m_estimator_on_loop_closures;

  bool do_icp_step_on_loop_closures;
  int loop_closures_sub_maps_radius;

  LaserTrackParams laser_track_params;
}; // struct EstimatorParams

} // namespace laser_slam

#endif // LASER_SLAM_PARAMETERS_HPP_
