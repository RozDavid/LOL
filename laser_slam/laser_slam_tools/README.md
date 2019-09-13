laser_slam_tools
==================

Useful tools to support laser mapping.
Requires [volumetric_mapping](https://github.com/ethz-asl/volumetric_mapping.git)

## laser_to_octomap
Creates octomap by successively inserting scans from the laser_mapper trajectory. Useful if there were loop closures.  
Usage: `rosrun laser_slam_tools laser_to_octomap output_file_path arg_name arg_val`  
Possible arguments are:
* **resolution** Octomap resolution
* **probability_hit** Probability of sensor beam hit
* **probability_miss** Probability of sensor beam miss
* **sensor_max_range** Maximum range of sensor

## octomap_to_point_cloud
Saves occupied leaf nodes of octomap as point cloud.
Supported output file formats are .pcd and .ply.
Usage: `rosrun laser_slam_tools octomap_to_point_cloud input_file_path output_file_path`