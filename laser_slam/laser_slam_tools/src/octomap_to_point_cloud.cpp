#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <octomap/OcTree.h>

std::string getFileExtension(const std::string& filename) {
  const size_t pos = filename.rfind('.');
  CHECK_NE(pos, std::string::npos) << "File path " << filename << " does not contain '.'";
  return filename.substr(pos + 1u);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  CHECK_EQ(argc, 3) << "Wrong number of input arguments.";
  const std::string octomap_file(argv[1]);
  const std::string output_file(argv[2]);

  octomap::OcTree octree(octomap_file);
  CHECK_NE(octree.size(), 0u) << "Failed to load Octomap from: " << octomap_file << ".";

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ point;
  for (auto it = octree.begin_leafs(); it != octree.end_leafs(); ++it) {
    if (octree.isNodeOccupied(*it)) {
      point.x = it.getX();
      point.y = it.getY();
      point.z = it.getZ();
      cloud.push_back(point);
    }
  }

  const std::string output_file_extension(getFileExtension(output_file));
  if (output_file_extension == "pcd") {
    CHECK_NE(pcl::io::savePCDFile(output_file, cloud), -1)
        << "Something went wrong when trying to write the pcd file.\n";
  }
  else if (output_file_extension == "ply") {
    CHECK_NE(pcl::io::savePLYFile(output_file, cloud) , -1)
        << "Something went wrong when trying to write the ply file.\n";
  }
  else {
    LOG(FATAL) << "Unknown output file extension " << output_file_extension << ".";
  }
  std::cout << "Successfully wrote point cloud to " << output_file << "\n";

}
