#include "laser_slam_ros/laser_slam_worker.hpp"

#include "laser_slam/benchmarker.hpp"

//TODO clean
#include <Eigen/Eigenvalues>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <laser_slam/benchmarker.hpp>
#include <laser_slam/common.hpp>
#include <laser_slam_ros/common.hpp>
#include <laser_slam_ros/laser_slam_worker.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>


namespace laser_slam_ros {

    using namespace laser_slam;

    LaserSlamWorker::LaserSlamWorker() {}

    LaserSlamWorker::~LaserSlamWorker() {}

    void LaserSlamWorker::init(
            ros::NodeHandle &nh, const LaserSlamWorkerParams &params,
            std::shared_ptr<laser_slam::IncrementalEstimator> incremental_estimator,
            unsigned int worker_id) {
        params_ = params;
        incremental_estimator_ = incremental_estimator;
        worker_id_ = worker_id;

        latest_match_transform = new Eigen::Matrix4f(Eigen::Matrix4f::Identity());
        match_transforms.insert(std::pair<int64_t, Eigen::Matrix4f *>(0, latest_match_transform));

        // Get the LaserTrack object from the IncrementalEstimator.
        laser_track_ = incremental_estimator_->getLaserTrack(worker_id);

        // Setup subscriber.
        scan_sub_ = nh.subscribe(params_.assembled_cloud_sub_topic, kScanSubscriberMessageQueueSize,
                                 &LaserSlamWorker::scanCallback, this);

        // Setup publishers.
        trajectory_pub_ = nh.advertise<nav_msgs::Path>(params_.trajectory_pub_topic,
                                                       kPublisherQueueSize, true);

        if (params_.publish_local_map) {
            local_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(params_.local_map_pub_topic,
                                                                    kPublisherQueueSize);
        }

        // Setup services.
        get_laser_track_srv_ = nh.advertiseService(
                "get_laser_track",
                &LaserSlamWorker::getLaserTracksServiceCall, this);
        export_trajectory_srv_ = nh.advertiseService(
                "export_trajectory",
                &LaserSlamWorker::exportTrajectoryServiceCall, this);

        voxel_filter_.setLeafSize(params_.voxel_size_m, params_.voxel_size_m,
                                  params_.voxel_size_m);
        voxel_filter_.setMinimumPointsNumberPerVoxel(params_.minimum_point_number_per_voxel);

        // Set the first world to odom transform.
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix;
        matrix.resize(4, 4);
        matrix = Eigen::Matrix<float, 4, 4>::Identity();
        world_to_odom_ = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
                matrix, params_.world_frame, params_.odom_frame, ros::Time::now());

        // TODO reactivate or rm.
        //  odometry_trajectory_pub_ = nh_.advertise<nav_msgs::Path>(params_.odometry_trajectory_pub_topic,
        //
        //  if (params_.publish_distant_map) {
        //    distant_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(params_.distant_map_pub_topic,
        //                                                               kPublisherQueueSize);
        //  }
        //  if (params_.publish_full_map) {
        //    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(params_.full_map_pub_topic,
        //                                                               kPublisherQueueSize);
        //  }
        //  new_fixed_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("new_fixed_cloud",
        //                                                               kPublisherQueueSize);
    }

    void LaserSlamWorker::scanCallback(const sensor_msgs::PointCloud2 &cloud_msg_in) {
        std::lock_guard<std::recursive_mutex> lock_scan_callback(scan_callback_mutex_);
        if (!lock_scan_callback_) {
            if (tf_listener_.waitForTransform(params_.odom_frame, params_.sensor_frame,
                                              cloud_msg_in.header.stamp, ros::Duration(kTimeout_s))) {


                // Get the tf transform.
                tf::StampedTransform tf_transform_old;
                tf::StampedTransform tf_transform;
                tf::Transform update;
                tf::Transform old;
                tf::Transform new_tf;
                tf_listener_.lookupTransform(params_.odom_frame, params_.sensor_frame,
                                             cloud_msg_in.header.stamp, tf_transform_old);

                old.setOrigin(tf_transform_old.getOrigin());
                old.setRotation(tf_transform_old.getRotation());
                old.getRotation().normalize();

                tf_transform.setRotation(tf_transform_old.getRotation());
                tf_transform.setOrigin(tf_transform_old.getOrigin());
                tf_transform.stamp_ = tf_transform_old.stamp_;
                tf_transform.frame_id_ = tf_transform_old.frame_id_;
                tf_transform.child_frame_id_ = tf_transform_old.child_frame_id_;

                //Updating the latest position with the right past transforms
                if (pose_transforms.size() > 0) {

                    Eigen::Matrix4f last_trans;
                    last_trans << pose_transforms[pose_transforms.size() - 1];
                    /*
                    Eigen::Quaternionf trans_quat(last_trans.block<3,3>(0,0));
                    tf::Quaternion q(trans_quat.x(), trans_quat.y(), trans_quat.z());
                    q.normalize();

                    update.setRotation(q);
                    update.setOrigin(tf::Vector3(last_trans(0,3), last_trans(1,3), last_trans(2,3)));
                    */

                    update = matrixToTransform(last_trans);
                    new_tf = update * old;
                    new_tf.getRotation().normalize();

                    tf_transform.setOrigin(new_tf.getOrigin());
                    tf_transform.setRotation(new_tf.getRotation());
                    tf_transform.getRotation().normalize();

                }


                bool process_scan = false;
                SE3 current_pose, pose_before_upate;

                if (!last_pose_set_) {
                    process_scan = true;
                    last_pose_set_ = true;
                    last_pose_ = tfTransformToPose(tf_transform).T_w;
                } else {
                    pose_before_upate = tfTransformToPose(tf_transform).T_w;
                    current_pose = SE3(pose_before_upate.getPosition(), pose_before_upate.getRotation());
                    float dist_m = distanceBetweenTwoSE3(current_pose, last_pose_);
                    if (dist_m > params_.minimum_distance_to_add_pose) {
                        process_scan = true;
                        last_pose_ = current_pose;
                    }
                }

                if (process_scan) {
                    // Convert input cloud to laser scan.
                    LaserScan new_scan;
                    new_scan.scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in);
                    new_scan.time_ns = rosTimeToCurveTime(cloud_msg_in.header.stamp.toNSec());

                    // Process the new scan and get new values and factors.
                    gtsam::NonlinearFactorGraph new_factors;
                    gtsam::Values new_values;
                    bool is_prior;
                    if (params_.use_odometry_information) {
                        laser_track_->processPoseAndLaserScan(tfTransformToPose(tf_transform), new_scan,
                                                              &new_factors, &new_values, &is_prior);
                    } else {
                        Pose new_pose, new_pose_upate;

                        Time new_pose_time_ns = tfTransformToPose(tf_transform).time_ns;

                        if (laser_track_->getNumScans() > 2u) {
                            Pose current_pose = laser_track_->getCurrentPose();

                            if (current_pose.time_ns > new_pose_time_ns - current_pose.time_ns) {
                                Time previous_pose_time = current_pose.time_ns -
                                                          (new_pose_time_ns - current_pose.time_ns);
                                if (previous_pose_time >= laser_track_->getMinTime() &&
                                    previous_pose_time <= laser_track_->getMaxTime()) {
                                    SE3 previous_pose = laser_track_->evaluate(previous_pose_time);
                                    new_pose.T_w = last_pose_sent_to_laser_track_.T_w *
                                                   previous_pose.inverse() * current_pose.T_w;
                                    new_pose.T_w = SE3(SO3::fromApproximateRotationMatrix(
                                            new_pose.T_w.getRotation().getRotationMatrix()),
                                                       new_pose.T_w.getPosition());
                                }
                            }
                        }


                        //Updating the latest position with the right past transforms
                        new_pose.time_ns = new_pose_time_ns;

                        laser_track_->processPoseAndLaserScan(new_pose, new_scan,
                                                              &new_factors, &new_values, &is_prior);

                        last_pose_sent_to_laser_track_ = new_pose;
                    }

                    // Process the new values and factors.
                    gtsam::Values result;
                    if (is_prior) {
                        result = incremental_estimator_->registerPrior(new_factors, new_values, worker_id_);
                    } else {
                        result = incremental_estimator_->estimate(new_factors, new_values, new_scan.time_ns);
                    }

                    // Update the trajectory.
                    laser_track_->updateFromGTSAMValues(result);

                    // Adjust the correction between the world and odom frames.
                    Pose current_pose = laser_track_->getCurrentPose();
                    SE3 T_odom_sensor = tfTransformToPose(tf_transform).T_w;
                    SE3 T_w_sensor = current_pose.T_w;
                    SE3 T_w_odom = T_w_sensor * T_odom_sensor.inverse();

                    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix;

                    // TODO resize needed?
                    matrix.resize(4, 4);
                    matrix = T_w_odom.getTransformationMatrix().cast<float>();

                    {
                        std::lock_guard<std::recursive_mutex> lock_world_to_odom(world_to_odom_mutex_);
                        world_to_odom_ = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
                                matrix, params_.world_frame, params_.odom_frame, cloud_msg_in.header.stamp);
                    }

                    //Save last true position update, to translate the path with at later timestamps
                    //Eigen::Vector3d vec_null(0.0,0.0,0.0);
                    publishTrajectories();

                    // Get the last cloud in world frame.
                    DataPoints new_fixed_cloud;
                    laser_track_->getLocalCloudInWorldFrame(laser_track_->getMaxTime(), &new_fixed_cloud);

                    // Transform the cloud in sensor frame
                    //TODO(Renaud) move to a transformPointCloud() fct.
                    //      laser_slam::PointMatcher::TransformationParameters transformation_matrix =
                    //          T_w_sensor.inverse().getTransformationMatrix().cast<float>();
                    //
                    //      laser_slam::correctTransformationMatrix(&transformation_matrix);
                    //
                    //      laser_slam::PointMatcher::Transformation* rigid_transformation =
                    //          laser_slam::PointMatcher::get().REG(Transformation).create("RigidTransformation");
                    //      CHECK_NOTNULL(rigid_transformation);
                    //
                    //      laser_slam::PointMatcher::DataPoints fixed_cloud_in_sensor_frame =
                    //          rigid_transformation->compute(new_fixed_cloud,transformation_matrix);
                    //
                    //
                    //      new_fixed_cloud_pub_.publish(
                    //          PointMatcher_ros::pointMatcherCloudToRosMsg<float>(fixed_cloud_in_sensor_frame,
                    //                                                             params_.sensor_frame,
                    //                                                             cloud_msg_in.header.stamp));

                    PointCloud new_fixed_cloud_pcl = lpmToPcl(new_fixed_cloud);

                    if (params_.remove_ground_from_local_map) {
                        const double robot_height_m = current_pose.T_w.getPosition()(2);
                        PointCloud new_fixed_cloud_no_ground;
                        for (size_t i = 0u; i < new_fixed_cloud_pcl.size(); ++i) {
                            if (new_fixed_cloud_pcl.points[i].z > robot_height_m -
                                                                  params_.ground_distance_to_robot_center_m) {
                                new_fixed_cloud_no_ground.push_back(new_fixed_cloud_pcl.points[i]);
                            }
                        }
                        new_fixed_cloud_no_ground.width = 1;
                        new_fixed_cloud_no_ground.height = new_fixed_cloud_no_ground.points.size();
                        new_fixed_cloud_pcl = new_fixed_cloud_no_ground;
                    }

                    // Add the local scans to the full point cloud.
                    if (params_.create_filtered_map) {
                        if (new_fixed_cloud_pcl.size() > 0u) {
                            std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
                            if (local_map_.size() > 0u) {
                                local_map_ += new_fixed_cloud_pcl;
                            } else {
                                local_map_ = new_fixed_cloud_pcl;
                            }
                            local_map_queue_.push_back(new_fixed_cloud_pcl);
                        }
                    }
                }
            } else {
                ROS_WARN_STREAM("[SegMapper] Timeout while waiting between " + params_.odom_frame +
                                " and " + params_.sensor_frame + ".");
            }
        }
    }

    void LaserSlamWorker::setLockScanCallback(bool new_state) {
        std::lock_guard<std::recursive_mutex> lock(scan_callback_mutex_);
        lock_scan_callback_ = new_state;
    }

    bool LaserSlamWorker::getLaserTracksServiceCall(
            laser_slam_ros::GetLaserTrackSrv::Request &request,
            laser_slam_ros::GetLaserTrackSrv::Response &response) {
        std::vector<std::shared_ptr<LaserTrack> > laser_tracks =
                incremental_estimator_->getAllLaserTracks();
        Trajectory trajectory;
        ros::Time scan_stamp;
        tf::StampedTransform tf_transform;
        geometry_msgs::TransformStamped ros_transform;
        std::vector<std::tuple<laser_slam::Time, sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> > data;
        for (const auto &track: laser_tracks) {
            track->getTrajectory(&trajectory);
            for (const auto &scan: track->getLaserScans()) {
                // Get data.
                scan_stamp.fromNSec(curveTimeToRosTime(scan.time_ns));

                sensor_msgs::PointCloud2 pc = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
                        scan.scan, params_.sensor_frame, scan_stamp);

                tf_transform = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
                        trajectory.at(scan.time_ns).getTransformationMatrix().cast<float>(),
                        params_.world_frame,
                        params_.sensor_frame,
                        scan_stamp);
                tf::transformStampedTFToMsg(tf_transform, ros_transform);

                data.push_back(std::make_tuple(scan.time_ns, pc, ros_transform));
            }
        }

        std::sort(data.begin(), data.end(),
                  [](const std::tuple<laser_slam::Time, sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> &a,
                     const std::tuple<laser_slam::Time, sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> &b) -> bool {
                      return std::get<0>(a) <= std::get<0>(b);
                  });

        bool zero_added = false;
        // Fill response.
        for (const auto &elem : data) {
            laser_slam::Time time;
            sensor_msgs::PointCloud2 pc;
            geometry_msgs::TransformStamped tf;
            std::tie(time, pc, tf) = elem;
            LOG(INFO) << "Time " << time;
            if (time == 0u) {
                if (!zero_added) {
                    zero_added = true;
                } else {
                    continue;
                }
            }
            response.laser_scans.push_back(pc);
            response.transforms.push_back(tf);
        }

        return true;
    }

    void LaserSlamWorker::publishTrajectory(const Trajectory &trajectory,
                                            const ros::Publisher &publisher, Eigen::Matrix4f *update_mat) const {


        //Ez a kódrész azért szükséges, hogy a loamos térképen is össze tudjuk illeszteni a elodyne helyét a kamion gps pozíciójáal.
        //Ehhez meghatározzuk az UTM rendszerbn a kettő relatív helyzetét
        //Majd eltranszformáljuk a LOAM koordinátákba
        //Végül az aktuális Loam koordinátás yaw angellel elforgatjuk
        Eigen::Matrix3d maps_registration_UTM_2;
        maps_registration_UTM_2 << 0.867799, 0.496051, 0.029283,
                -0.496479, 0.867999, 0.009289,
                -0.020810, -0.022599, 0.999528;


        double roll, pitch, yaw_to_loam;

        Eigen::Vector3d ea = maps_registration_UTM_2.eulerAngles(2, 1, 0);
        yaw_to_loam = ea(0);

        Eigen::Vector4d lidar_to_gps = Eigen::Vector4d((-2.6695), (1.7835), 0, 1);
        Eigen::Vector3d l_t_gps_h(lidar_to_gps(0), lidar_to_gps(1), lidar_to_gps(1));

        Eigen::Vector3d rotated_lidar_to_gps;
        nav_msgs::Path traj_msg;
        traj_msg.header.frame_id = params_.world_frame;
        Time traj_time = curveTimeToRosTime(trajectory.rbegin()->first);
        traj_msg.header.stamp.fromNSec(traj_time);

        Eigen::MatrixXd matrix;
        matrix.resize(trajectory.size(), 6);
        unsigned int i = 0u;


        for (const auto &timePose : trajectory) {
            Eigen::Matrix4f absolut_trans_until_timestamp = Eigen::Matrix4f::Identity();
            for (const auto &transforms : match_transforms) {
                if (curveTimeToRosTime(timePose.first) > curveTimeToRosTime(transforms.first))
                    //absolut_trans_until_timestamp += *transforms.second;
                    absolut_trans_until_timestamp = *transforms.second;
            }
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header = traj_msg.header;
            pose_msg.header.stamp.fromNSec(curveTimeToRosTime(timePose.first));

            /*pose_msg.pose.position.x = timePose.second.getPosition().x() + absolut_trans_until_timestamp(0, 0);
            pose_msg.pose.position.y = timePose.second.getPosition().y() + absolut_trans_until_timestamp(1, 0);
            pose_msg.pose.position.z = timePose.second.getPosition().z() + absolut_trans_until_timestamp(2, 0);
*/

            pose_msg.pose.position.x = timePose.second.getPosition().x();
            pose_msg.pose.position.y = timePose.second.getPosition().y();
            pose_msg.pose.position.z = timePose.second.getPosition().z();

            pose_msg.pose.orientation.w = timePose.second.getRotation().w();
            pose_msg.pose.orientation.x = timePose.second.getRotation().x();
            pose_msg.pose.orientation.y = timePose.second.getRotation().y();
            pose_msg.pose.orientation.z = timePose.second.getRotation().z();
            traj_msg.poses.push_back(pose_msg);


            double yaw_angle = tf::getYaw(tf::Quaternion(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                                                         pose_msg.pose.orientation.z, pose_msg.pose.orientation.w));
            Eigen::Matrix3d rot(Eigen::AngleAxisd((yaw_angle + yaw_to_loam), Eigen::Vector3d::UnitZ()));
            rotated_lidar_to_gps = rot * l_t_gps_h;
            matrix(i, 0) = curveTimeToRosTime(timePose.first);
            matrix(i, 1) = timePose.second.getPosition().x() + rotated_lidar_to_gps(0);
            matrix(i, 2) = timePose.second.getPosition().y() + rotated_lidar_to_gps(1);
            matrix(i, 3) = timePose.second.getPosition().z() + rotated_lidar_to_gps(2);
            matrix(i, 4) = yaw_angle * 180 / M_PI;
            matrix(i, 5) = yaw_to_loam * 180 / M_PI;
            ++i;
        }


        //Saving the result for evaluation hardcoded path as won't be needed later

        matrix.conservativeResize(i, 6);

        std::string filename(params_.path_to_export_trajectory);
        std::ifstream f(filename.c_str());
        if (remove(filename.c_str()) != 0) {
            //perror( "Error deleting file" );
        } else {
            //writeEigenMatrixXdCSV(matrix, filename);
        }


        publisher.publish(traj_msg);
    }

    void LaserSlamWorker::publishMap() {
        // TODO make thread safe.
        if (local_map_.size() > 0) {
            PointCloud filtered_map;
            getFilteredMap(&filtered_map);

            //maximumNumberPointsFilter(&filtered_map);
            //    if (params_.publish_full_map) {
            //      sensor_msgs::PointCloud2 msg;
            //      convert_to_point_cloud_2_msg(filtered_map, params_.world_frame, &msg);
            //      point_cloud_pub_.publish(msg);
            //    }
            if (params_.publish_local_map) {
                sensor_msgs::PointCloud2 msg;
                {
                    std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
                    convert_to_point_cloud_2_msg(local_map_filtered_, params_.world_frame, &msg);
                }
                local_map_pub_.publish(msg);
            }
            //    if (params_.publish_distant_map) {
            //      sensor_msgs::PointCloud2 msg;
            //      convert_to_point_cloud_2_msg(distant_map_, params_.world_frame, &msg);
            //      distant_map_pub_.publish(msg);
            //    }
        }
    }


    bool findByValue(std::map<int64_t, Eigen::Vector3d *> mapOfElemen, Eigen::Vector3d *value) {
        bool bResult = false;
        //auto it = mapOfElemen.begin();
        auto it = mapOfElemen.end();
        it--;

        Eigen::Vector3d dist = (*(it->second) - *value);
        if (dist.norm() < 0.4) {
            bResult = true;
        }

        return bResult;
    }

    void LaserSlamWorker::publishTrajectories(Eigen::Matrix4f *mat, std::int64_t Time) {
        Trajectory trajectory;
        laser_track_->getTrajectory(&trajectory);

        Eigen::Matrix4f abs_transform = Eigen::Matrix4f::Identity();
        for (const auto &transforms : match_transforms) {
            abs_transform = *transforms.second * abs_transform;
        }

        std::map<int64_t, Eigen::Matrix4f *>::iterator it = match_transforms.find(Time);

        if (*latest_match_transform != *mat) {
            //Ros Transform plot
            tf::Transform update;
            /*Eigen::Quaternionf trans_quat(mat->block<3,3>(0,0));
            tf::Quaternion q(trans_quat.x(), trans_quat.y(), trans_quat.z());
            q.normalize();

            update.setRotation(q);
            update.setOrigin(tf::Vector3((*mat)(0,3), (*mat)(1,3), (*mat)(2,3)));*/

            update = matrixToTransform(*mat);

            latest_match_transform = new Eigen::Matrix4f(*mat);
            match_transforms.insert(std::pair<int64_t, Eigen::Matrix4f *>(Time, latest_match_transform));
            ROS_FATAL("Absolute transformation after new trans:\n");
            std::cout << *latest_match_transform << "\n(" << update.getOrigin().x() << " " << update.getOrigin().y()
                      << " " << update.getOrigin().z() << ")\n(" << update.getRotation().x() << " "
                      << update.getRotation().y() << " " << update.getRotation().z() << " " << update.getRotation().w()
                      << ") " << std::endl;
        }

        //updateLocalMap(abs_transform.cast<float>());

        if (trajectory.size() > 0u) publishTrajectory(trajectory, trajectory_pub_, mat);

    }

    void LaserSlamWorker::publishTrajectories() {
        Trajectory trajectory;
        laser_track_->getTrajectory(&trajectory);
        if (trajectory.size() > 0u) publishTrajectory(trajectory, trajectory_pub_, latest_match_transform);

    }

    // TODO can we move?
    Pose LaserSlamWorker::tfTransformToPose(const tf::StampedTransform &tf_transform) {
        // Register new pose.
        Pose pose;
        SE3::Position pos(tf_transform.getOrigin().getX(), tf_transform.getOrigin().getY(),
                          tf_transform.getOrigin().getZ());
        SE3::Rotation::Implementation rot(tf_transform.getRotation().getW(),
                                          tf_transform.getRotation().getX(),
                                          tf_transform.getRotation().getY(),
                                          tf_transform.getRotation().getZ());
        pose.T_w = SE3(pos, rot);
        pose.time_ns = rosTimeToCurveTime(tf_transform.stamp_.toNSec());

        return pose;
    }

    Time LaserSlamWorker::rosTimeToCurveTime(const Time &timestamp_ns) {
        if (!base_time_set_) {
            base_time_ns_ = timestamp_ns;
            base_time_set_ = true;
        }
        return timestamp_ns - base_time_ns_;
    }

    Time LaserSlamWorker::curveTimeToRosTime(const Time &timestamp_ns) const {
        CHECK(base_time_set_);
        return timestamp_ns + base_time_ns_;
    }

    std::vector<laser_slam_ros::PointCloud> LaserSlamWorker::getQueuedPoints() {
        std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
        std::vector<laser_slam_ros::PointCloud> new_points;
        new_points.swap(local_map_queue_);
        return new_points;
    }

// TODO one shot of cleaning.
    void LaserSlamWorker::getFilteredMap(PointCloud *filtered_map) {
        laser_slam::Pose current_pose = laser_track_->getCurrentPose();

        PclPoint current_position;
        current_position.x = current_pose.T_w.getPosition()[0];
        current_position.y = current_pose.T_w.getPosition()[1];
        current_position.z = current_pose.T_w.getPosition()[2];

        // Apply the cylindrical filter on the local map and get a copy.
        PointCloud local_map;
        {
            std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
            local_map = local_map_;
            applyCylindricalFilter(current_position, params_.distance_to_consider_fixed,
                                   40, false, &local_map_);
        }
        // Apply a voxel filter.
        laser_slam::Clock clock;

        PointCloudPtr local_map_ptr(new PointCloud());
        pcl::copyPointCloud<PclPoint, PclPoint>(local_map, *local_map_ptr);

        PointCloud local_map_filtered;

        voxel_filter_.setInputCloud(local_map_ptr);
        voxel_filter_.filter(local_map_filtered);

        clock.takeTime();

        if (params_.separate_distant_map) {
            // If separating the map is enabled, the distance between each point in the local_map_ will
            // be compared to the current robot position. Points which are far from the robot will
            // be transfered to the distant_map_. This is helpful for publishing (points in distant_map_
            // need to be filtered only once) and for any other processing which needs to be done only
            // when a map is distant from robot and can be assumed as static (until loop closure).

            // TODO(renaud) Is there a way to separate the cloud without having to transform in sensor
            // frame by setting the position to compute distance from?
            // Transform local_map_ in sensor frame.
            clock.start();

            // Save before removing points.
            PointCloud new_distant_map = local_map_filtered;

            applyCylindricalFilter(current_position, params_.distance_to_consider_fixed,
                                   40, false, &local_map_filtered);

            applyCylindricalFilter(current_position, params_.distance_to_consider_fixed,
                                   40, true, &new_distant_map);
            {
                std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
                local_map_filtered_ = local_map_filtered;
            }

            // Add the new_distant_map to the distant_map_.
            // TODO add lock if used
            if (distant_map_.size() > 0u) {
                distant_map_ += new_distant_map;
            } else {
                distant_map_ = new_distant_map;
            }

            *filtered_map = local_map_filtered;
            *filtered_map += distant_map_;

            clock.takeTime();
            // LOG(INFO) << "new_local_map.size() " << local_map.size();
            // LOG(INFO) << "new_distant_map.size() " << new_distant_map.size();
            // LOG(INFO) << "distant_map_.size() " << distant_map_.size();
            // LOG(INFO) << "Separating done! Took " << clock.getRealTime() << " ms.";
        } else {
            *filtered_map = local_map;
        }
    }

    void LaserSlamWorker::getLocalMapFiltered(PointCloud *local_map_filtered) {
        CHECK_NOTNULL(local_map_filtered);
        std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
        *local_map_filtered = local_map_filtered_;
    }

    void LaserSlamWorker::clearLocalMap() {
        {
            std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
            local_map_.clear();
        }

        {
            std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
            local_map_filtered_.clear();
        }
    }

    tf::StampedTransform LaserSlamWorker::getWorldToOdom() {
        std::lock_guard<std::recursive_mutex> lock_world_to_odom(world_to_odom_mutex_);
        tf::StampedTransform world_to_odom = world_to_odom_;
        return world_to_odom;
    }

    void LaserSlamWorker::getTrajectory(Trajectory *out_trajectory) const {
        laser_track_->getTrajectory(out_trajectory);
    }

    void LaserSlamWorker::getOdometryTrajectory(Trajectory *out_trajectory) const {
        laser_track_->getOdometryTrajectory(out_trajectory);
    }

    //The preious version, now my version is used
    /*void LaserSlamWorker::updateLocalMap(const SE3 &last_pose_before_update,
                                         const laser_slam::Time last_pose_before_update_timestamp_ns) {

        Trajectory new_trajectory;
        laser_track_->getTrajectory(&new_trajectory);

        SE3 new_last_pose = new_trajectory.at(last_pose_before_update_timestamp_ns);

        const Eigen::Matrix4f transform_matrix = (new_last_pose * last_pose_before_update.inverse()).
                getTransformationMatrix().cast<float>();
        {
            std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
            pcl::transformPointCloud(local_map_, local_map_, transform_matrix);
        }
        {
            std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
            pcl::transformPointCloud(local_map_filtered_, local_map_filtered_, transform_matrix);
        }
    }*/

    void LaserSlamWorker::updateLocalMap(Eigen::Vector3f updateTransfrm) {

        Eigen::Matrix4f transform_matrix = Eigen::Matrix<float, 4, 4>::Identity();
        transform_matrix.topRightCorner(4, 1) << updateTransfrm(0, 0), updateTransfrm(1, 0), updateTransfrm(2, 0), 1;

        {
            std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
            pcl::transformPointCloud(local_map_, local_map_, transform_matrix);
        }
        {
            std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
            pcl::transformPointCloud(local_map_filtered_, local_map_filtered_, transform_matrix);
        }

        std::cout << "Local Map updated with: " << updateTransfrm << std::endl;
    }


    laser_slam::SE3 LaserSlamWorker::getTransformBetweenPoses(
            const laser_slam::SE3 &start_pose, const laser_slam::Time end_pose_timestamp_ns) const {
        Trajectory new_trajectory;
        laser_track_->getTrajectory(&new_trajectory);

        SE3 last_pose = new_trajectory.at(end_pose_timestamp_ns);
        return last_pose * start_pose.inverse();
    }

    void LaserSlamWorker::exportTrajectories() const {
        Trajectory traj;
        laser_track_->getTrajectory(&traj);
        Eigen::MatrixXd matrix;
        matrix.resize(traj.size(), 4);
        unsigned int i = 0u;
        for (const auto &pose : traj) {
            matrix(i, 0) = pose.first;
            matrix(i, 1) = pose.second.getPosition()(0);
            matrix(i, 2) = pose.second.getPosition()(1);
            matrix(i, 3) = pose.second.getPosition()(2);
            ++i;
        }
        writeEigenMatrixXdCSV(matrix, "/tmp/trajectory_" + std::to_string(worker_id_) + ".csv");
    }

    void LaserSlamWorker::exportTrajectoryHead(laser_slam::Time head_duration_ns,
                                               const std::string &filename) const {
        Eigen::MatrixXd matrix;
        Trajectory traj;
        laser_track_->getTrajectory(&traj);
        CHECK_GE(traj.size(), 1u);
        matrix.resize(traj.size(), 4);

        const Time traj_end_ns = traj.rbegin()->first;
        Time head_start_ns;
        if (traj_end_ns > head_duration_ns) {
            head_start_ns = traj_end_ns - head_duration_ns;
        } else {
            head_start_ns = 0u;
        }

        unsigned int i = 0u;
        for (const auto &pose : traj) {
            if (pose.first > head_start_ns) {
                matrix(i, 0) = curveTimeToRosTime(pose.first);
                matrix(i, 1) = pose.second.getPosition()(0);
                matrix(i, 2) = pose.second.getPosition()(1);
                matrix(i, 3) = pose.second.getPosition()(2);
                ++i;
            }
        }
        matrix.conservativeResize(i, 4);
        writeEigenMatrixXdCSV(matrix, filename);
        LOG(INFO) << "Exported " << i << " trajectory poses to " << filename << ".";
    }

    bool LaserSlamWorker::exportTrajectoryServiceCall(std_srvs::Empty::Request &req,
                                                      std_srvs::Empty::Response &res) {
        exportTrajectoryHead(laser_track_->getMaxTime(),
                             "/home/david/Catkin/loam-segmap/records/traj/trajectory_exported.csv");
        return true;
    }

    void LaserSlamWorker::insertLatestPose(Eigen::Matrix4f &matrix4f) {
        if (!pose_transforms.size() == 0 &&
            pose_transforms[pose_transforms.size() - 1].block<3, 1>(0, 3).norm() != matrix4f.block<3, 1>(0, 3).norm()) {
            pose_transforms.push_back(matrix4f);
        } else if (pose_transforms.size() == 0) {
            pose_transforms.push_back(matrix4f);
        } else {
        }
    }

} // namespace laser_slam_ros
