#ifndef SEGMATCH_ROS_SEGMATCH_WORKER_HPP_
#define SEGMATCH_ROS_SEGMATCH_WORKER_HPP_

#include <utility>

#include <laser_slam/common.hpp>
#include <segmatch/database.hpp>
#include <segmatch/local_map.hpp>
#include <segmatch/segmatch.hpp>
#include <std_srvs/Empty.h>

#include "segmatch_ros/common.hpp"

#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>


namespace segmatch_ros {

    class SegMatchWorker {

    public:
        explicit SegMatchWorker();

        ~SegMatchWorker();

        /**
        * \brief Initializing the Segmatch objects connection interface for the ROS middleware
        * @param nh The nodehanler for the publishers
        * @param params  The read parameters from the yaml file
        * @param num_tracks The tracks to densifie the local map
        */
        void init(ros::NodeHandle &nh, const SegMatchWorkerParams &params,
                  unsigned int num_tracks = 1u);

        /// Process the local map. This performs segmentation, matching and, if
        /// \c loop_closure is provided, loop closure detection.
        /// \param local_map The current local map.
        /// \param latest_pose The latest known pose of the robot.
        /// \param track_id ID of the track being currently processed.
        /// \param loop_closure If specified, pointer to an object where the result
        /// loop closure will be stored.
        /// \returns True if a loop closure was found.
        bool processLocalMap(
                segmatch::SegMatch::LocalMapT &local_map,
                const laser_slam::Pose &latest_pose,
                unsigned int track_id = 0u,
                laser_slam::RelativePose *loop_closure = NULL);

        void update(const laser_slam::Trajectory &trajectory);

        void update(const std::vector<laser_slam::Trajectory> &trajectories);

        /**
         * \brief For the Segmapper node to acces the lates position updates
         * @return A pointer for the 3D translation vector
         */
        Eigen::Matrix4f *getLatestTrajectoryUpdate() {
            return latest_match_transform;
        }


        /**
         * \brief For Obtaining the transformation update according to the found matches between the target and source clouds
         * @param matches The paired matches between the target and source clouds
         * @param transform pointer to the transform where we will save the resulting transformation matrix
         */
        void transformationfromMatches(segmatch::PairwiseMatches matches, Eigen::Matrix<float, 4, 4> *transform);


        /**
          * \brief For the Segmapper node to acces the lates position updates
          * @return A pointer for the 3D translation vector
          */
        std::vector<Eigen::Matrix4f *> getTrajectoryUpdates() {
            return match_transforms;
        }


        /**
       * \brief Stores the trajectories in a vector. Always the latest is visualized
       * @param trajectories A map of scan times and matrix<double,4,4> rotation matrix. Invoked from the Segmapper node, to update the trajecories
       */

        void saveTimings() const {
            segmatch_.saveTimings();
        }

        void publish();

        /**
         * \brief Could be used, if Rviz visualization is not requred
         *
         */
        void stopPublishing(unsigned int track_id) {
            publish_local_representation_[track_id] = false;
        }

    private:

        void loadTargetCloud();

        void publishTargetRepresentation() const;

        void publishSourceRepresentation() const;

        void publishTargetReconstruction() const;

        void publishSourceReconstruction() const;

        void publishSourceSemantics() const;

        void publishTargetSemantics() const;

        /**
         * \brief It publishes and accepts/declines the matches between the target and source maps.
         */
        void publishMatches();

        bool acceptMatches() const;

        void publishSegmentationPositions() const;

        void publishLoopClosures() const;

        void publishTargetSegmentsCentroids() const;

        void publishSourceSegmentsCentroids() const;

        void publishSourceBoundingBoxes() const;

        void publishTargetBoundingBoxes() const;

        bool exportRunServiceCall(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        bool reconstructSegmentsServiceCall(std_srvs::Empty::Request &req,
                                            std_srvs::Empty::Response &res);

        bool toggleCompressionServiceCall(std_srvs::Empty::Request &req,
                                          std_srvs::Empty::Response &res);

        bool togglePublishTargetServiceCall(std_srvs::Empty::Request &req,
                                            std_srvs::Empty::Response &res);

        bool exportTargetMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        // Parameters.
        SegMatchWorkerParams params_;

        // Publishers.
        ros::Publisher source_representation_pub_;
        ros::Publisher source_reconstruction_pub_;
        ros::Publisher target_representation_pub_;
        ros::Publisher target_reconstruction_pub_;
        ros::Publisher source_semantics_pub_;
        ros::Publisher target_semantics_pub_;
        ros::Publisher matches_pub_;
        ros::Publisher false_matches_pub_;
        ros::Publisher predicted_matches_pub_;
        ros::Publisher loop_closures_pub_;
        ros::Publisher segmentation_positions_pub_;
        ros::Publisher target_segments_centroids_pub_;
        ros::Publisher source_segments_centroids_pub_;
        ros::Publisher reconstruction_pub_;
        ros::Publisher bounding_boxes_pub_;

        ros::ServiceServer export_run_service_;
        ros::ServiceServer reconstruct_segments_service_;
        ros::ServiceServer toggle_compression_service_;
        ros::ServiceServer toggle_publish_target_service_;
        ros::ServiceServer export_target_map_;

        // SegMatch object.
        segmatch::SegMatch segmatch_;

        typedef segmatch::DynamicVoxelGrid <pcl::PointXYZ, segmatch::PointExtended> VoxelGrid;
        typedef typename VoxelGrid::InputCloud InputCloud;
        typedef typename VoxelGrid::VoxelCloud ClusteredCloud;

        ClusteredCloud local_map_cloud;


        bool target_cloud_loaded_ = false;

        typedef std::pair<laser_slam::Pose, unsigned int> PoseTrackIdPair;

        /**
        * \brief important for benchmarking and calculating the distances between the segmentation poses.
        */
        std::vector<PoseTrackIdPair> last_segmented_poses_;

        bool first_localization_occured_ = false;

        segmatch::SegmentedCloud segments_database_;
        segmatch::database::UniqueIdMatches matches_database_;

        std::unordered_map<unsigned int, segmatch::PointICloud> source_representations_;

        unsigned int num_tracks_;
        unsigned int pub_counter_ = 0;
        segmatch::PairwiseMatches matches_;

        std::vector<bool> publish_local_representation_;

        bool compress_when_publishing_ = false;
        bool publish_target_ = true;

        /* Eigen::Vector3d* latest_match_transform;
        std::vector<Eigen::Vector3d*> match_transforms;*/

        Eigen::Matrix4f *latest_match_transform;
        std::vector<Eigen::Matrix4f *> match_transforms;

        uint64_t latest_match_timestamp;
        uint64_t latest_transform_index;
        bool clear_local_map = false;

        static constexpr unsigned int kPublisherQueueSize = 50u;
    }; // SegMatchWorker

} // namespace segmatch_ros

#endif /* SEGMATCH_ROS_SEGMATCH_WORKER_HPP_ */
