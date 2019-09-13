#include <chrono>

#include <laser_slam_ros/GetLaserTrackSrv.h>
#include <octomap_world/octomap_manager.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  // Initialize glog for volumetric_mapping.
  google::InitGoogleLogging(argv[0]);
  // Get output file path.
  if (argc < 2) {
    ROS_ERROR("No output file path specified");
    return -1;
  }
  const std::string file_path(argv[1]);

  // Defaults.
  double resolution = 0.075;
  double prob_hit = 0.9;
  double prob_miss = 0.4;
  double max_range = 20.0;
  bool publish = true;

  // Parse arguments.
  if (argc > 2) {
    if (argc % 2) {
      ROS_ERROR("Invalid number of command line arguments");
      return -1;
    }
    for (int i = 2; i < argc; i+=2) {
      const std::string argument(argv[i]);
      if (argument == "resolution") resolution = std::atof(argv[i+1]);
      else if (argument == "probability_hit") prob_hit = std::atof(argv[i+1]);
      else if (argument == "probability_miss") prob_miss = std::atof(argv[i+1]);
      else if (argument == "sensor_max_range") max_range = std::atof(argv[i+1]);
      else {
        ROS_ERROR("Invalid command line argument \"%s\"", argv[i]);
        return -1;
      }
    }

  }
  ROS_INFO("Output file path is: \"%s\"", file_path.c_str());
  ROS_INFO("Octomap resolution is %fm", resolution);
  ROS_INFO("Probability hit/miss are: %f / %f", prob_hit, prob_miss);
  ROS_INFO("Sensor max range is %fm", max_range);

  // Init ROS things.
  ros::init(argc, argv, "laser_to_octomap");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Get laser track.
  ros::ServiceClient client;
  client = nh.serviceClient<laser_slam_ros::GetLaserTrackSrv>("/segmapper/get_laser_track");
  laser_slam_ros::GetLaserTrackSrv call;
  ROS_INFO("Requested laser track. Waiting...");
  if (!client.call(call)) {
    ROS_ERROR("Client call failed");
    return -1;
  }
  const size_t traj_length = call.response.laser_scans.size();
  ROS_INFO("Received laser track of length %lu", traj_length);
  ROS_INFO("You can shut down laser_mapper");

  // Set parameters.
  nh_private.setParam("resolution", resolution);
  nh_private.setParam("probability_hit", prob_hit);
  nh_private.setParam("probability_miss", prob_miss);
  nh_private.setParam("sensor_max_range", max_range);
  nh_private.setParam("use_tf_transforms", false);
  nh_private.setParam("robot_frame", call.response.transforms.front().header.frame_id);
  nh_private.setParam("world_frame", call.response.transforms.front().child_frame_id);

  volumetric_mapping::OctomapManager manager(nh, nh_private);

  // Volumetric Mapping expects ConstPtr.
  // Make copies so there is no double free when ConstPtr falls out of scope.
  std::vector<sensor_msgs::PointCloud2::ConstPtr> ptr_vec;
  for (auto&& scan : call.response.laser_scans) {
    ptr_vec.emplace_back(new sensor_msgs::PointCloud2(scan));
  }
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

  // Insertion loop.
  for (size_t i = 0u; i < traj_length && ros::ok(); ++i) {
    std::cout << "\rInserting point cloud " << i+1 << "/" << traj_length;
    if (i > 0) {
      // Estimate remaining time.
      std::chrono::high_resolution_clock::time_point end =
          std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> duration = end - start;
      duration *= (double)(traj_length - i) / i;
      auto int_min = std::chrono::duration_cast<std::chrono::minutes>(duration);
      auto int_seconds = std::chrono::duration_cast<std::chrono::seconds>(duration - int_min);
      if (int_min.count() > 0) std::cout << "\t" << int_min.count() << " minutes remaining  ";
      else std::cout << "\t" << int_seconds.count() << " seconds remaining  ";
    }
    std::cout << std::flush;
    // Do insertion.
    manager.transformCallback(call.response.transforms[i]);
    manager.insertPointcloudWithTf(ptr_vec[i]);
    if (publish) manager.publishAll();
  }
  std::cout << "\r" << std::flush;
  ROS_INFO("Done inserting all clouds");

  // Save octomap.
  volumetric_msgs::SaveMap octo_call;
  octo_call.request.file_path = file_path;
  if (!manager.saveOctomapCallback(octo_call.request, octo_call.response)) {
    ROS_ERROR("Saving octomap to file failed");
  }
  else ROS_INFO("Saved octomap to file");
}
