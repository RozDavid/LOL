#include "segmatch_ros/segmatch_worker.hpp"
#include <ros/console.h>

#include <laser_slam/benchmarker.hpp>
#include <laser_slam/common.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <math.h>

#include "segmatch/rviz_utilities.hpp"
#include "segmatch/utilities.hpp"

namespace segmatch_ros {

    using namespace laser_slam;
    using namespace segmatch;

    SegMatchWorker::SegMatchWorker() {}

    SegMatchWorker::~SegMatchWorker() {}

    void SegMatchWorker::init(ros::NodeHandle &nh, const SegMatchWorkerParams &params,
                              unsigned int num_tracks) {
        params_ = params;
        num_tracks_ = num_tracks;

        // Initialize SegMatch.
        segmatch_.init(params_.segmatch_params, num_tracks);

        // Setup publishers.
        source_representation_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                "/segmatch/source_representation", kPublisherQueueSize);
        target_representation_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                "/segmatch/target_representation", kPublisherQueueSize, true);
        source_reconstruction_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                "/segmatch/source_reconstruction", kPublisherQueueSize);
        target_reconstruction_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                "/segmatch/target_reconstruction", kPublisherQueueSize, true);
        source_semantics_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                "/segmatch/source_semantics", kPublisherQueueSize);
        target_semantics_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                "/segmatch/target_semantics", kPublisherQueueSize);
        matches_pub_ = nh.advertise<visualization_msgs::Marker>(
                "/segmatch/segment_matches", kPublisherQueueSize);
        false_matches_pub_ = nh.advertise<visualization_msgs::Marker>(
                "/segmatch/false_segment_matches", kPublisherQueueSize);
        if (params_.publish_predicted_segment_matches) {
            predicted_matches_pub_ = nh.advertise<visualization_msgs::Marker>(
                    "/segmatch/predicted_segment_matches", kPublisherQueueSize);
        }
        loop_closures_pub_ = nh.advertise<visualization_msgs::Marker>(
                "/segmatch/loop_closures", kPublisherQueueSize);
        segmentation_positions_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                "/segmatch/segmentation_positions", kPublisherQueueSize);
        target_segments_centroids_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                "/segmatch/target_segments_centroids", kPublisherQueueSize);
        source_segments_centroids_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                "/segmatch/source_segments_centroids", kPublisherQueueSize);

        export_run_service_ = nh.advertiseService("export_run",
                                                  &SegMatchWorker::exportRunServiceCall, this);
        segments_database_ = SegmentedCloud(false);

        reconstruct_segments_service_ = nh.advertiseService(
                "reconstruct_segments", &SegMatchWorker::reconstructSegmentsServiceCall, this);

        toggle_compression_service_ = nh.advertiseService(
                "toggle_compression", &SegMatchWorker::toggleCompressionServiceCall, this);

        toggle_publish_target_service_ = nh.advertiseService(
                "toggle_publish_target", &SegMatchWorker::togglePublishTargetServiceCall, this);

        export_target_map_ = nh.advertiseService(
                "export_target_map", &SegMatchWorker::exportTargetMap, this);

        if (std::find(params_.segmatch_params.descriptors_params.descriptor_types.begin(),
                      params_.segmatch_params.descriptors_params.descriptor_types.end(),
                      "CNN") !=
            params_.segmatch_params.descriptors_params.descriptor_types.end()) {
            reconstruction_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                    "/segmatch/reconstruction", kPublisherQueueSize);
            bounding_boxes_pub_ = nh.advertise<visualization_msgs::Marker>(
                    "/segmatch/bounding_boxes", kPublisherQueueSize);
        }

        if (params_.localize) {
            loadTargetCloud();
            publishTargetRepresentation();
            publishTargetSegmentsCentroids();
            latest_match_transform = new Eigen::Matrix4f(Eigen::Matrix4f::Identity());
            latest_match_timestamp = ros::Time::now().toNSec();
            latest_transform_index = 0;
        }

        for (unsigned int i = 0u; i < num_tracks_; ++i) {
            publish_local_representation_.push_back(true);
        }

        BENCHMARK_RESET_ALL();
    }

    void SegMatchWorker::loadTargetCloud() {
        ROS_INFO("Loading target cloud.");
        segmatch::MapCloud target_cloud;
        segmatch::loadCloud(params_.target_cloud_filename, &target_cloud);
        segmatch_.processAndSetAsTargetCloud(target_cloud);
        target_cloud_loaded_ = true;
    }

    bool SegMatchWorker::processLocalMap(
            segmatch::SegMatch::LocalMapT &local_map,
            const laser_slam::Pose &latest_pose,
            unsigned int track_id,
            RelativePose *loop_closure) {
        BENCHMARK_BLOCK("SM.Worker");

        local_map_cloud = local_map.getFilteredPoints();

        if (params_.close_loops) {
            CHECK_NOTNULL(loop_closure);
        }

        if ((params_.localize && target_cloud_loaded_) || params_.close_loops) {
            // Check that the robot drove enough since last segmentation.
            bool robot_drove_enough = false;
            if (last_segmented_poses_.empty()) {
                last_segmented_poses_.push_back(PoseTrackIdPair(latest_pose, track_id));
                robot_drove_enough = true;
            } else {
                bool last_segmented_pose_set = false;
                for (auto &pose_track_id_pair: last_segmented_poses_) {
                    if (pose_track_id_pair.second == track_id) {
                        last_segmented_pose_set = true;
                        if (distanceBetweenTwoSE3(pose_track_id_pair.first.T_w, latest_pose.T_w) >
                            params_.distance_between_segmentations_m) {
                            robot_drove_enough = true;
                            pose_track_id_pair.first = latest_pose;
                        }
                    }
                }
                if (!last_segmented_pose_set) {
                    robot_drove_enough = true;
                    last_segmented_poses_.push_back(PoseTrackIdPair(latest_pose, track_id));
                }
            }

            if (!robot_drove_enough) return false;

            // Process the source cloud, with tha past transforms and segment, describe inside segmatch
            Eigen::Matrix<float, 4, 4> past_transform_sum;
            past_transform_sum << Eigen::Matrix<float, 4, 4>::Identity();

            if (latest_transform_index < match_transforms.size()) {
                latest_transform_index = match_transforms.size();
                //ROS_FATAL("Transform LocalMap");
                std::cout << "Index: " << latest_transform_index << "  Size: " << match_transforms.size() << std::endl;
                past_transform_sum = *match_transforms[match_transforms.size() - 1];

                //Eigen::Matrix<float, 4, 4> ambiguous_error_avoider(past_transform_sum.cast<float>());
                //kindr::minimal::QuatTransformationTemplate<float> quatTransformation(ambiguous_error_avoider);
                //local_map.transform(quatTransformation);

                local_map.clear();
            }


            segmatch_.processAndSetAsSourceCloud(local_map, latest_pose, track_id);
            if (params_.export_segments_and_matches) {
                segments_database_ += segmatch_.getSourceAsSegmentedCloud(track_id);
            }

            // Find matches.
            PairwiseMatches predicted_matches = segmatch_.findMatches(NULL, track_id,
                                                                      latest_pose.time_ns);

            // Filter matches and try to recognize the local map.
            PairwiseMatches filtered_matches = segmatch_.filterMatches(predicted_matches, track_id);
            const PairwiseMatches &recognized_matches = segmatch_.recognize(filtered_matches,
                                                                            track_id,
                                                                            latest_pose.time_ns,
                                                                            loop_closure);

            // TODO move after optimizing and updating target map?
            if (params_.close_loops) {
                // If we did not find a loop-closure, transfer the source to the target map.
                if (recognized_matches.empty()) {
                    segmatch_.transferSourceToTarget(track_id, latest_pose.time_ns);
                }
            } else if (params_.localize) {
                if (!recognized_matches.empty() && !first_localization_occured_) {
                    first_localization_occured_ = true;
                    if (params_.align_target_map_on_first_loop_closure) {
                        BENCHMARK_BLOCK("SM.Worker.AlignTargetMap");
                        LOG(INFO) << "Aligning target map.";
                        segmatch_.alignTargetMap();
                        publishTargetRepresentation();
                        publishTargetSegmentsCentroids();
                    }
                }
            }

            // Store segments and matches in database if desired, for later export.
            if (params_.export_segments_and_matches && !recognized_matches.empty()) {
                BENCHMARK_BLOCK("SM.Worker.AddMatchesToDatabase");
                for (size_t i = 0u; i < recognized_matches.size(); ++i) {
                    matches_database_.addMatch(recognized_matches.at(i).ids_.first,
                                               recognized_matches.at(i).ids_.second);
                }
            }

            if (params_.localize) {
                if (!first_localization_occured_ || !recognized_matches.empty()) {
                    publish();
                    if (clear_local_map) {
                        clear_local_map = false;
                        local_map.clear();
                        matches_.clear();
                    }
                }
            }

            if (params_.close_loops && recognized_matches.empty()) {
                publish();
            }
            return !recognized_matches.empty();
        } else {
            return false;
        }
    }

    void SegMatchWorker::update(const Trajectory &trajectory) {
        std::vector<Trajectory> trajectories;
        trajectories.push_back(trajectory);
        segmatch_.update(trajectories);
        publish();
    }

    void SegMatchWorker::update(const std::vector<Trajectory> &trajectories) {
        segmatch_.update(trajectories);
        publish();
    }

    void SegMatchWorker::publish() {

        PairwiseMatches new_matches = segmatch_.getFilteredMatches();
        matches_.insert(matches_.end(), new_matches.begin(), new_matches.end());

        pub_counter_++;
        if (pub_counter_ % num_tracks_ == 0) {
            pub_counter_ = 0;
            BENCHMARK_BLOCK("SM.Worker.Publish");
            // Publish matches, source representation and segmentation positions.
            publishMatches();
            matches_.clear();
            publishSourceRepresentation();
            publishSourceReconstruction();
            publishSourceSemantics();

            publishSourceSegmentsCentroids();
            publishSegmentationPositions();

            // If closing loops, republish the target map.
            if (params_.close_loops) {
                publishLoopClosures();
                if (publish_target_) {
                    publishTargetRepresentation();
                    publishTargetSegmentsCentroids();
                    publishTargetReconstruction();
                }
            }

            // Publish the bounding boxes for the source cloud if using an autoencoder.
            if (std::find(params_.segmatch_params.descriptors_params.descriptor_types.begin(),
                          params_.segmatch_params.descriptors_params.descriptor_types.end(),
                          "Autoencoder") !=
                params_.segmatch_params.descriptors_params.descriptor_types.end()) {
                //publishTargetBoundingBoxes();
                //publishSourceBoundingBoxes();
            }
        }
    }

    void SegMatchWorker::publishTargetRepresentation() const {
        PointICloud target_representation;
        segmatch_.getTargetRepresentation(&target_representation, compress_when_publishing_);
        translateCloud(Translation(0.0, 0.0, -params_.distance_to_lower_target_cloud_for_viz_m),
                       &target_representation);
        sensor_msgs::PointCloud2 target_representation_as_message;
        convert_to_point_cloud_2_msg(target_representation, params_.world_frame,
                                     &target_representation_as_message);
        target_representation_pub_.publish(target_representation_as_message);
    }

    void SegMatchWorker::publishTargetReconstruction() const {
        PointICloud target_reconstruction;
        segmatch_.getTargetReconstruction(&target_reconstruction, compress_when_publishing_);
        translateCloud(Translation(0.0, 0.0, -params_.distance_to_lower_target_cloud_for_viz_m),
                       &target_reconstruction);
        sensor_msgs::PointCloud2 target_reconstruction_as_message;
        convert_to_point_cloud_2_msg(target_reconstruction, params_.world_frame,
                                     &target_reconstruction_as_message);
        target_reconstruction_pub_.publish(target_reconstruction_as_message);
    }

    void SegMatchWorker::publishSourceReconstruction() const {
        PointICloud source_reconstruction;
        segmatch_.getSourceReconstruction(&source_reconstruction);
        translateCloud(Translation(0.0, 0.0, params_.distance_to_lower_target_cloud_for_viz_m),
                       &source_reconstruction);
        sensor_msgs::PointCloud2 source_reconstruction_as_message;
        convert_to_point_cloud_2_msg(source_reconstruction, params_.world_frame,
                                     &source_reconstruction_as_message);
        source_reconstruction_pub_.publish(source_reconstruction_as_message);
    }

    void SegMatchWorker::publishSourceRepresentation() const {
        laser_slam::Clock clock;
        PointICloud full_source_representation;
        BENCHMARK_BLOCK("SM.Worker.PublishSourceRepresentation");

        for (unsigned int i = 0u; i < num_tracks_; ++i) {
            if (!publish_local_representation_[i]) continue;
            PointICloud source_representation;
            segmatch_.getSourceRepresentation(&source_representation, 0.0, i);
            //segmatch_.getSourceSemantics(&source_representation, 0.0, i);
            full_source_representation += source_representation;
        }

        applyRandomFilterToCloud(params_.ratio_of_points_to_keep_when_publishing,
                                 &full_source_representation);
        sensor_msgs::PointCloud2 source_representation_as_message;
        convert_to_point_cloud_2_msg(full_source_representation, params_.world_frame,
                                     &source_representation_as_message);
        source_representation_pub_.publish(source_representation_as_message);
    }

    void SegMatchWorker::publishSourceSemantics() const {
        laser_slam::Clock clock;

        PointICloud full_source_semantics;

        for (unsigned int i = 0u; i < num_tracks_; ++i) {
            PointICloud source_semantics;
            segmatch_.getSourceSemantics(&source_semantics, 0.0, i);
            full_source_semantics += source_semantics;
        }

        sensor_msgs::PointCloud2 source_semantics_as_message;
        convert_to_point_cloud_2_msg(full_source_semantics, params_.world_frame,
                                     &source_semantics_as_message);
        source_semantics_pub_.publish(source_semantics_as_message);
    }

    void SegMatchWorker::publishTargetSemantics() const {
        // TODO
    }


    void SegMatchWorker::publishMatches() {
        PointPairs point_pairs;
        double x1, y1, z1;
        int number_of_matches = 0;
        std::vector<pcl::PointXYZ> distances;
        std::vector<Eigen::Vector3d> match_distances;

        //harcode need to import into parameters
        bool true_match = false;
        double paralellity_treshold =
                params_.max_distance_for_accepting_matches / 5.0; //m for cancelling the multidirecional matches

        double transformation_difference = 0;


        for (size_t i = 0u; i < matches_.size(); ++i) {

            PclPoint target_segment_centroid = matches_[i].getCentroids().second;
            target_segment_centroid.z -= params_.distance_to_lower_target_cloud_for_viz_m;
            point_pairs.push_back(
                    PointPair(matches_[i].getCentroids().first, target_segment_centroid));
            number_of_matches++;

            x1 = (matches_[i].getCentroids().second.x - matches_[i].getCentroids().first.x);
            y1 = (matches_[i].getCentroids().second.y - matches_[i].getCentroids().first.y);
            z1 = (matches_[i].getCentroids().second.z - matches_[i].getCentroids().first.z);


            pcl::PointXYZ dist(x1, y1, z1);
            distances.push_back(dist);
            match_distances.push_back(Eigen::Vector3d(x1, y1, z1));

        }

        try {
            if (match_distances.size() > 1) {
                true_match = true;
                /*for(auto const &m_dist : match_distances){
                    double planar_norm= std::sqrt(std::pow(m_dist(0,0),2)+ std::pow(m_dist(1,0),2));
                    if(planar_norm>params_.max_distance_for_accepting_matches){
                        true_match= false;
                        break;
                    }
                }*/
                for (size_t i = 0; i < match_distances.size() - 1; i++) {
                    double direction_x = std::abs(match_distances.at(i)(0, 0) - match_distances.at(i + 1)(0, 0));
                    double direction_y = std::abs(match_distances.at(i)(1, 0) - match_distances.at(i + 1)(1, 0));
                    if (direction_x > paralellity_treshold || direction_y > paralellity_treshold) {
                        true_match = false;
                        transformation_difference = std::sqrt(pow(direction_x, 2) + pow(direction_y, 2));
                        break;
                    }

                }
            }
        } catch (...) {
            ROS_FATAL("An exception ocured while accepting the matches");
        }


        //Inspecting the paralellity treshold, and the last timestamp of the last match with the yaml parameter
        if (true_match && ((ros::Time::now().toNSec() - latest_match_timestamp) >
                           (static_cast<double>(params_.min_time_between_matches_s) * pow(10, 9)))) {
            //We do not want to acept the z direction mutches, thus here ommenting that part out for now. Later for UAV-s might be useful
            Eigen::Matrix4f *accepted_transform(new Eigen::Matrix4f(Eigen::Matrix4f::Identity()));

            if (matches_.size() > 3) {
                transformationfromMatches(matches_, accepted_transform);

            }

            latest_match_transform = accepted_transform;
            latest_match_timestamp = ros::Time::now().toNSec();
            match_transforms.push_back(accepted_transform);


            publishLineSet(point_pairs, params_.world_frame, params_.line_scale_matches,
                           Color(0.0, 1.0, 0.0), matches_pub_);

            //Update the latest match transform for the segmatch object, to later translate the local map in the right position.
            //The combination to a vector of past matches will be made there to concatenate into a vector. The final tansformation will ba the sum of past matches.
            segmatch_.updateTransformations(*accepted_transform);

        } else if (true_match) {
            //don't publish, beacause of timestamp redundancy
            matches_.clear();
        }
            //publish in red, not paralell, thus wrong transform from matches
        else if (!true_match) {
            publishLineSet(point_pairs, params_.world_frame, params_.line_scale_matches,
                           Color(1.0, 0.0, 0.0), matches_pub_);
            latest_match_timestamp = ros::Time::now().toNSec();
            if (matches_.size() > 4) {
                clear_local_map = true;
            }


        } else {
            //do nothing, thank you
        }

        if (params_.publish_predicted_segment_matches) {
            const PairwiseMatches predicted_matches = segmatch_.getPredictedMatches();
            point_pairs.clear();
            for (size_t i = 0u; i < predicted_matches.size(); ++i) {
                PclPoint target_segment_centroid = predicted_matches[i].getCentroids().second;
                target_segment_centroid.z -= params_.distance_to_lower_target_cloud_for_viz_m;
                point_pairs.push_back(
                        PointPair(predicted_matches[i].getCentroids().first, target_segment_centroid));
            }
            publishLineSet(point_pairs, params_.world_frame, params_.line_scale_matches,
                           Color(0.7, 0.7, 0.7), predicted_matches_pub_);
        }
    }

    bool SegMatchWorker::acceptMatches() const {}

    void SegMatchWorker::publishSegmentationPositions() const {
        PointCloud segmentation_positions_cloud;
        std::vector<Trajectory> segmentation_poses;
        segmatch_.getSegmentationPoses(&segmentation_poses);

        for (const auto &trajectory: segmentation_poses) {
            for (const auto &pose: trajectory) {
                segmentation_positions_cloud.points.push_back(
                        se3ToPclPoint(pose.second));
            }
        }

        segmentation_positions_cloud.width = 1;
        segmentation_positions_cloud.height = segmentation_positions_cloud.points.size();
        sensor_msgs::PointCloud2 segmentation_positions_as_message;
        convert_to_point_cloud_2_msg(segmentation_positions_cloud, params_.world_frame,
                                     &segmentation_positions_as_message);
        segmentation_positions_pub_.publish(segmentation_positions_as_message);
    }

    void SegMatchWorker::publishTargetSegmentsCentroids() const {
        PointICloud segments_centroids;
        segmatch_.getTargetSegmentsCentroidsWithTrajIdAsIntensity(&segments_centroids);
        translateCloud(Translation(0.0, 0.0, -params_.distance_to_lower_target_cloud_for_viz_m),
                       &segments_centroids);
        sensor_msgs::PointCloud2 segments_centroids_as_message;
        convert_to_point_cloud_2_msg(segments_centroids, params_.world_frame,
                                     &segments_centroids_as_message);
        target_segments_centroids_pub_.publish(segments_centroids_as_message);
    }

    void SegMatchWorker::publishSourceSegmentsCentroids() const {
        PointICloud full_segments_centroids;

        for (unsigned int i = 0u; i < num_tracks_; ++i) {
            PointICloud segments_centroids;
            segmatch_.getSourceSegmentsCentroids(&segments_centroids, i);
            full_segments_centroids += segments_centroids;
        }

        sensor_msgs::PointCloud2 segments_centroids_as_message;
        convert_to_point_cloud_2_msg(full_segments_centroids, params_.world_frame,
                                     &segments_centroids_as_message);
        source_segments_centroids_pub_.publish(segments_centroids_as_message);
    }

    void SegMatchWorker::publishLoopClosures() const {
        PointPairs point_pairs;

        std::vector<laser_slam::RelativePose> loop_closures;
        segmatch_.getLoopClosures(&loop_closures);
        std::vector<laser_slam::Trajectory> segmentation_poses;
        segmatch_.getSegmentationPoses(&segmentation_poses);

        for (const auto &loop_closure: loop_closures) {
            PclPoint source_point, target_point;
            source_point = se3ToPclPoint(
                    segmentation_poses.at(loop_closure.track_id_a).at(loop_closure.time_a_ns));
            target_point = se3ToPclPoint(
                    segmentation_poses.at(loop_closure.track_id_b).at(loop_closure.time_b_ns));
            point_pairs.push_back(PointPair(source_point, target_point));
        }

        // Query the segmentation_poses_ at that time.
        publishLineSet(point_pairs, params_.world_frame, params_.line_scale_loop_closures,
                       Color(1.0, 1.0, 1.0), loop_closures_pub_);
    }

    bool SegMatchWorker::exportRunServiceCall(std_srvs::Empty::Request &req,
                                              std_srvs::Empty::Response &res) {
        // Get current date.
        const boost::posix_time::ptime time_as_ptime = ros::WallTime::now().toBoost();
        std::string acquisition_time = to_iso_extended_string(time_as_ptime);

        if (params_.export_segments_and_matches) {
            // TODO RD clean if not needed.
            ROS_FATAL("Segments, Matches Positions events and Features exported");
            database::exportMatches("/tmp/online_matcher/run_" + acquisition_time + "_matches.csv",
                                    matches_database_);
            database::exportSegmentsAndFeatures("/tmp/online_matcher/run_" + acquisition_time,
                                                segments_database_, true);
            database::exportPositions("/tmp/online_matcher/run_" + acquisition_time + "_positions.csv",
                                      segments_database_, true);
            database::exportMergeEvents("/tmp/online_matcher/run_" + acquisition_time + "_merge_events.csv",
                                        segmatch_.getMergeEvents());
        }

        // segmatch_.exportDescriptorsData();

        return true;
    }

    bool SegMatchWorker::exportTargetMap(std_srvs::Empty::Request &req,
                                         std_srvs::Empty::Response &res) {
        // Get current date.
        const boost::posix_time::ptime time_as_ptime = ros::WallTime::now().toBoost();
        std::string acquisition_time = to_iso_extended_string(time_as_ptime);
        SegmentedCloud target_map = segmatch_.getTargetAsSegmentedCloud();

        database::exportSegments("/tmp/online_matcher/run_" + acquisition_time + "_segments.csv",
                                 target_map, false);

        database::exportSegments("/tmp/online_matcher/run_" + acquisition_time + "_reconstructions.csv",
                                 target_map, false, true);

        ROS_FATAL("Segments Exported");
        return true;
    }

    std::vector<BoundingBox> getBoundingBoxesFromSegmentedCloud(
            const SegmentedCloud &segmented_cloud) {
        std::vector<BoundingBox> bounding_boxes;
        for (const auto &id_segment : segmented_cloud) {
            BoundingBox bounding_box;
            bounding_box.centroid = id_segment.second.getLastView().centroid;

            Feature feature = id_segment.second.getLastView().features.at(0u);
            FeatureValue feature_value;
            CHECK(feature.findValueByName("alignment", &feature_value));
            bounding_box.alignment = feature_value.value;

            CHECK(feature.findValueByName("origin_dx", &feature_value));
            bounding_box.centroid.x += feature_value.value;

            CHECK(feature.findValueByName("origin_dy", &feature_value));
            bounding_box.centroid.y += feature_value.value;

            CHECK(feature.findValueByName("scale_x", &feature_value));
            bounding_box.scale_x_m = feature_value.value;

            CHECK(feature.findValueByName("scale_y", &feature_value));
            bounding_box.scale_y_m = feature_value.value;

            CHECK(feature.findValueByName("scale_z", &feature_value));
            bounding_box.scale_z_m = feature_value.value;

            bounding_boxes.push_back(bounding_box);
        }
        return bounding_boxes;
    }

    template<typename T>
    std::vector<size_t> getIndexesInDecreasingOrdering(const std::vector<T> &v) {

        // initialize original index locations
        std::vector<size_t> idx(v.size());
        std::iota(idx.begin(), idx.end(), 0);

        // sort indexes based on comparing values in v
        std::sort(idx.begin(), idx.end(),
                  [&v](size_t i1, size_t i2) { return v[i1] > v[i2]; });

        return idx;
    }

    bool SegMatchWorker::reconstructSegmentsServiceCall(std_srvs::Empty::Request &req,
                                                        std_srvs::Empty::Response &res) {
        publishTargetReconstruction();
        return true;
    }

    bool SegMatchWorker::toggleCompressionServiceCall(std_srvs::Empty::Request &req,
                                                      std_srvs::Empty::Response &res) {
        compress_when_publishing_ = !compress_when_publishing_;
        if (compress_when_publishing_) {
            LOG(INFO) << "Compression set to true";
        } else {
            LOG(INFO) << "Compression set to false";
        }
        return true;
    }

    bool SegMatchWorker::togglePublishTargetServiceCall(std_srvs::Empty::Request &req,
                                                        std_srvs::Empty::Response &res) {
        publish_target_ = !publish_target_;
        if (publish_target_) {
            LOG(INFO) << "Publish target set to true";
        } else {
            LOG(INFO) << "Publish target set to false";
        }
        return true;
    }

    void SegMatchWorker::publishTargetBoundingBoxes() const {
        std::vector<BoundingBox> all_bounding_boxes;
        for (unsigned int i = 0u; i < num_tracks_; ++i) {
            SegmentedCloud segmented_target_cloud = segmatch_.getTargetAsSegmentedCloud();
            std::vector<BoundingBox> bounding_boxes = getBoundingBoxesFromSegmentedCloud(
                    segmented_target_cloud);
            all_bounding_boxes.insert(all_bounding_boxes.end(), bounding_boxes.begin(),
                                      bounding_boxes.end());
        }
        publishBoundingBoxes(all_bounding_boxes, params_.world_frame, bounding_boxes_pub_,
                             Color(1.0, 1.0, 0.0),
                             1.0 * params_.distance_to_lower_target_cloud_for_viz_m);
    }

    void SegMatchWorker::publishSourceBoundingBoxes() const {
        std::vector<BoundingBox> all_bounding_boxes;
        for (unsigned int i = 0u; i < num_tracks_; ++i) {
            SegmentedCloud segmented_source_cloud = segmatch_.getSourceAsSegmentedCloud();
            std::vector<BoundingBox> bounding_boxes = getBoundingBoxesFromSegmentedCloud(
                    segmented_source_cloud);
            all_bounding_boxes.insert(all_bounding_boxes.end(), bounding_boxes.begin(),
                                      bounding_boxes.end());
        }
        publishBoundingBoxes(all_bounding_boxes, params_.world_frame, bounding_boxes_pub_,
                             Color(0.0, 1.0, 1.0),
                             1.0 * params_.distance_to_lower_target_cloud_for_viz_m);
    }

    void SegMatchWorker::transformationfromMatches(segmatch::PairwiseMatches matches,
                                                   Eigen::Matrix<float, 4, 4> *transform) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trgt(new pcl::PointCloud<pcl::PointXYZ>());

        cloud_src->width = matches.size();
        cloud_src->height = 1;
        cloud_src->is_dense = false;
        cloud_src->resize(cloud_src->width * cloud_src->height);

        cloud_trgt->width = matches.size();
        cloud_trgt->height = 1;
        cloud_trgt->is_dense = false;
        cloud_trgt->resize(cloud_trgt->width * cloud_trgt->height);

        Eigen::Vector3f mean(0, 0, 0);

        for (int i = 0; i < matches.size(); i++) {
            cloud_src->points[i].x = matches[i].centroids_.first.x;
            cloud_src->points[i].y = matches[i].centroids_.first.y;
            cloud_src->points[i].z = matches[i].centroids_.first.z;

            cloud_trgt->points[i].x = matches[i].centroids_.second.x;
            cloud_trgt->points[i].y = matches[i].centroids_.second.y;
            cloud_trgt->points[i].z = matches[i].centroids_.second.z;

            mean += Eigen::Vector3f(cloud_trgt->at(i).x - cloud_src->at(i).x, cloud_trgt->at(i).y - cloud_src->at(i).y,
                                    cloud_trgt->at(i).z - cloud_src->at(i).z);
        }
        mean /= matches.size();

        Eigen::Matrix4f to_mean = Eigen::Matrix4f::Identity();
        to_mean.topRightCorner(3, 1) << mean(0, 0), mean(1, 0), mean(2, 0);
        //pcl::transformPointCloud(*cloud_src, *cloud_src, to_mean);

        boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
        corr_est.setInputCloud(cloud_src);
        corr_est.setInputTarget(cloud_trgt);
        corr_est.determineCorrespondences(*correspondences);

        pcl::Correspondences new_corrs;
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> corrsRansac;
        corrsRansac.setInputCloud(cloud_src);
        corrsRansac.setTargetCloud(cloud_trgt);
        corrsRansac.setMaxIterations(params_.ransac_max_iterations);
        corrsRansac.setInlierThreshold(params_.ransac_outlier_trashold);
        corrsRansac.setInputCorrespondences(correspondences);
        corrsRansac.getCorrespondences(new_corrs);

        //write transforms to file
        std::ofstream out_file_stream;
        out_file_stream.open("/home/david/Catkin/loam-segmap/records/updates/updates.txt",
                             std::ofstream::out | std::ofstream::app);
        out_file_stream << "Timestamp:\t" << latest_match_timestamp << std::endl;
        out_file_stream << "Correspondences:\n";


        for (int i = new_corrs.size() - 1; i >= 0; i--) {
            out_file_stream << "From: " << new_corrs[i].index_query << "\t To: " << new_corrs[i].index_match
                            << std::endl;
            if (new_corrs[i].index_match != new_corrs[i].index_query) {
                //std::cout<<"Removed correspondence #"<< new_corrs[i].index_query <<"->"<<new_corrs[i].index_match<< std::endl;
                new_corrs.erase(new_corrs.begin() + i);
            }
        }


        if (new_corrs.size() > 8) {

            pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd_trans;
            svd_trans.estimateRigidTransformation(*cloud_src, *cloud_trgt, new_corrs, *transform);

            std::cout << "SVD Transform:\n " << *transform << "\n\n";
            out_file_stream << "SVD Transform: \n";
            out_file_stream << *transform << "\n\n";

            *transform = *transform * to_mean;

        } else {
            *transform = to_mean;
        }


        Eigen::Matrix4f prior_tf(*transform);
        segmatch_.compareScene(local_map_cloud, matches, prior_tf, transform);
        std::cout << "Number of corrs: " << new_corrs.size() << std::endl;

        out_file_stream << "Transform: \n";
        out_file_stream << *transform << "\n\n";
        out_file_stream.close();


    }

} // namespace segmatch_ros
