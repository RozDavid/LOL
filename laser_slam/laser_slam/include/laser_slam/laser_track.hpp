#ifndef LASER_SLAM_LASER_TRACK_HPP_
#define LASER_SLAM_LASER_TRACK_HPP_

#include <mutex>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <mincurves/DiscreteSE3Curve.hpp>

#include "laser_slam/common.hpp"
#include "laser_slam/parameters.hpp"

namespace laser_slam {

/// \brief The LaserTrack class interfaces point cloud acquisition with Sliding Window Estimator
/// problem setup and optimization.
class LaserTrack {

 public:
  LaserTrack() {};

  /// \brief Constructor.
  explicit LaserTrack(const LaserTrackParams& parameters, unsigned int laser_track_id = 0u);

  ~LaserTrack() {};

  /// \brief Process a new pose measurement in world frame.
  void processPose(const Pose& pose);

  /// \brief Process a new laser scan in laser frame.
  void processLaserScan(const LaserScan& scan);

  void processPoseAndLaserScan(const Pose& pose, const LaserScan& in_scan,
                               gtsam::NonlinearFactorGraph* newFactors = NULL,
                               gtsam::Values* newValues = NULL,
                               bool* is_prior = NULL);

  // Accessing the laser data
  /// \brief Get the point cloud of the last laser scan.
  void getLastPointCloud(DataPoints* out_point_cloud) const;

  /// \brief Get the point cloud within a time interval.
  void getPointCloudOfTimeInterval(const std::pair<Time, Time>& times_ns,
                                   DataPoints* out_point_cloud) const;

  /// \brief Get one local cloud in world frame.
  void getLocalCloudInWorldFrame(const Time& timestamp, DataPoints* out_point_cloud) const;

  /// \brief Get all laser scans.
  const std::vector<LaserScan>& getLaserScans() const;

  /// \brief Get the trajectory.
  void getTrajectory(Trajectory* trajectory) const;

  /// \brief Get the trajectory based only on odometry data.
  void getOdometryTrajectory(Trajectory* out_trajectory) const;

  /// \brief Get the covariance matrices.
  void getCovariances(std::vector<Covariance>* out_covariances) const;

  /// \brief Get the current estimate.
  Pose getCurrentPose() const;

  Pose getPreviousPose() const;

  /// \brief Get the first valid time of the trajectory.
  Time getMinTime() const;

  /// \brief Get the last valid time of the trajectory.
  Time getMaxTime() const;

  /// \brief Get the timestamps of the laser scans.
  void getLaserScansTimes(std::vector<Time>* out_times_ns) const;

  /// \brief Append the prior factors to the factor graph.
  void appendPriorFactors(const curves::Time& prior_time_ns,
                          gtsam::NonlinearFactorGraph* graph) const;

  /// \brief Append the odometry factors to the factor graph.
  void appendOdometryFactors(const curves::Time& optimization_min_time_ns,
                             const curves::Time& optimization_max_time_ns,
                             gtsam::noiseModel::Base::shared_ptr noise_model,
                             gtsam::NonlinearFactorGraph* graph) const;

  /// \brief Append the ICP factors to the factor graph.
  void appendICPFactors(const curves::Time& optimization_min_time_ns,
                        const curves::Time& optimization_max_time_ns,
                        gtsam::noiseModel::Base::shared_ptr noise_model,
                        gtsam::NonlinearFactorGraph* graph) const;

  /// \brief Append loop closure factors to the factor graph.
  void appendLoopClosureFactors(const curves::Time& optimization_min_time_ns,
                                const curves::Time& optimization_max_time_ns,
                                gtsam::noiseModel::Base::shared_ptr noise_model,
                                gtsam::NonlinearFactorGraph* graph) const;

  /// \brief Initialize GTSAM values from the trajectory.
  void initializeGTSAMValues(const gtsam::KeySet& keys, gtsam::Values* values) const;

  /// \brief Update the trajectory from GTSAM values.
  void updateFromGTSAMValues(const gtsam::Values& values);

  /// \brief Update the covariance matrices from GTSAM values.
  void updateCovariancesFromGTSAMValues(const gtsam::NonlinearFactorGraph& factor_graph,
                                        const gtsam::Values& values);

  /// \brief Get the number of registered laser scans.
  size_t getNumScans() {
    std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
    return laser_scans_.size();
  };

  /// \brief Print the underlying trajectory -- only for debugging.
  void printTrajectory() {
    std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
    trajectory_.print("Laser track trajectory");
  };

  // Find nearest pose to a givent time.
  // TODO: Make obsolete?
  Pose findNearestPose(const Time& timestamp_ns) const;

  void buildSubMapAroundTime(const curves::Time& time_ns,
                             const unsigned int sub_maps_radius,
                             DataPoints* submap_out) const;

  gtsam::Expression<SE3> getValueExpression(const curves::Time& time_ns) {
    std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
    return trajectory_.getValueExpression(time_ns);
  };

  SE3 evaluate(const curves::Time& time_ns) const {
    std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
    return trajectory_.evaluate(time_ns);
  }

  void getScanMatchingTimes(std::map<laser_slam::Time, double>* scan_matching_times) const {
    CHECK_NOTNULL(scan_matching_times);
    *scan_matching_times = scan_matching_times_;
  }

  void saveTrajectory(const std::string& filename) const {
    trajectory_.saveCurveTimesAndValues(filename);
  }

 private:
  typedef curves::DiscreteSE3Curve CurveType;

  // Make a relative pose measurement factor.
  gtsam::ExpressionFactor<SE3>
  makeRelativeMeasurementFactor(const RelativePose& relative_pose_measurement,
                                gtsam::noiseModel::Base::shared_ptr noise_model,
                                const bool fix_first_node = false) const;

  // Make a pose measurement factor.
  gtsam::ExpressionFactor<SE3>
  makeMeasurementFactor(const Pose& pose_measurement,
                        gtsam::noiseModel::Base::shared_ptr noise_model) const;

  // Compute rigid ICP transformations according to the selected strategy.
  void computeICPTransformations();

  // Compute ICP transformation between the last local scan to a concatenation of the
  // previous scans.
  void localScanToSubMap();

  // Get the pose measurements at a given time.
  SE3 getPoseMeasurement(const Time& timestamp_ns) const { return findPose(timestamp_ns).T_w; };

  // Set a pose key.
  void setPoseKey(const Time& timestamp_ns, const Key& key) { findPose(timestamp_ns)->key = key; };

  // Get a pose key.
  Key getPoseKey(const Time& timestamp_ns) const { return findPose(timestamp_ns).key; };

  // Find a pose at a given time.
  Pose* findPose(const Time& timestamp_ns);

  // Find a pose at a given time.
  Pose findPose(const Time& timestamp_ns) const;

  // Wrapper to extend the trajectory cleanly.
  // TODO(Renaud): Move this to curves.
  Key extendTrajectory(const Time& timestamp_ns, const SE3& value);

  std::vector<LaserScan>::const_iterator getIteratorToScanAtTime(
      const curves::Time& time_ns) const;

  unsigned int laser_track_id_;

  // TODO move pose_measurements_ to the Trajectory type.
  // Pose measurements in world frame.
  PoseVector pose_measurements_;

  // Odometry measurements in laser frame.
  // Obtained from combining and interpolating the pose measurements.
  RelativePoseVector odometry_measurements_;

  // Rigid transformations obtained from ICP between LaserScans.
  RelativePoseVector icp_transformations_;

  RelativePoseVector loop_closures_;

  // Laser scans in laser frame.
  std::vector<LaserScan> laser_scans_;

  // Underlying trajectory.
  CurveType trajectory_;

  // TODO replace by standard mutex?
  mutable std::recursive_mutex full_laser_track_mutex_;

  // Covariance matrices.
  std::vector<Covariance> covariances_;

  // ICP algorithm object.
  PointMatcher::ICP icp_;

  // ICP input filters.
  PointMatcher::DataPointsFilters input_filters_;

  // Libpointmatcher rigid transformation.
  PointMatcher::Transformation* rigid_transformation_;

  // Noise models.
  gtsam::noiseModel::Base::shared_ptr prior_noise_model_;
  gtsam::noiseModel::Base::shared_ptr odometry_noise_model_;
  gtsam::noiseModel::Base::shared_ptr icp_noise_model_;

  std::map<laser_slam::Time, double> scan_matching_times_;

  // Parameters.
  LaserTrackParams params_;

  static constexpr double kDistanceBetweenPriorPoses_m = 100.0;
};

}  // namespace laser_slam

#endif /* LASER_SLAM_LASER_TRACK_HPP_ */
