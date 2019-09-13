#ifndef LASER_SLAM_INCREMENTAL_ESTIMATOR_HPP_
#define LASER_SLAM_INCREMENTAL_ESTIMATOR_HPP_

#include <mutex>
#include <unordered_map>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "laser_slam/common.hpp"
#include "laser_slam/laser_track.hpp"
#include "laser_slam/parameters.hpp"

namespace laser_slam {

/// \brief Sliding window estimator class.
class IncrementalEstimator {

 public:
  IncrementalEstimator() {};
  /// \brief Constructor.
  explicit IncrementalEstimator(const EstimatorParams& parameters,
                                unsigned int n_laser_slam_workers = 1u);

  ~IncrementalEstimator() {};

  /// \brief Process a new loop closure.
  void processLoopClosure(const RelativePose& loop_closure);

  /// \brief Get the current estimate.
  Pose getCurrentPose(unsigned int laser_track_id = 0u) const {
    std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
    return laser_tracks_[laser_track_id]->getCurrentPose();
  };

  std::shared_ptr<LaserTrack> getLaserTrack(unsigned int laser_track_id);

  std::vector<std::shared_ptr<LaserTrack> > getAllLaserTracks();

  // Build the factor graph and estimate the trajectory.
  gtsam::Values estimate(const gtsam::NonlinearFactorGraph& new_factors,
                         const gtsam::Values& new_values,
                         laser_slam::Time timestamp_ns = 0u);

  gtsam::Values estimateAndRemove(const gtsam::NonlinearFactorGraph& new_factors,
                                  const gtsam::NonlinearFactorGraph& new_associations_factors,
                                  const gtsam::Values& new_values,
                                  const std::vector<unsigned int>& affected_worker_ids,
                                  laser_slam::Time timestamp_ns = 0u);

  gtsam::Values registerPrior(const gtsam::NonlinearFactorGraph& new_factors,
                              const gtsam::Values& new_values,
                              const unsigned int worker_id);

 private:
  unsigned int n_laser_slam_workers_;

  // TODO replace by standard mutex?
  mutable std::recursive_mutex full_class_mutex_;

  // Underlying laser tracks.
  std::vector<std::shared_ptr<LaserTrack> > laser_tracks_;

  // Scans which are out of the sliding window estimation and fixed into world frame.
  std::vector<DataPoints> fixed_scans_;

  gtsam::ISAM2 isam2_;

  // ICP algorithm object.
  PointMatcher::ICP icp_;

  gtsam::noiseModel::Base::shared_ptr loop_closure_noise_model_;
  gtsam::noiseModel::Base::shared_ptr first_association_noise_model_;

  std::unordered_map<unsigned int, size_t> factor_indices_to_remove_;
  
  std::vector<std::vector<unsigned int> > linked_workers_;

  // Parameters.
  EstimatorParams params_;
}; // IncrementalEstimator

}  // namespace laser_slam

#endif /* LASER_SLAM_INCREMENTAL_ESTIMATOR_HPP_ */
