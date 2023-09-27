#include <beam_filtering/VoxelDownsample.h>
#include <beam_utils/utils.h>

#include <gflags/gflags.h>

DEFINE_string(map1_cloud_path, "", "Full path to point cloud.");
DEFINE_validator(map1_cloud_path, &beam::gflags::ValidateFileMustExist);

DEFINE_string(map2_cloud_path, "", "Full path to point cloud.");
DEFINE_validator(map2_cloud_path, &beam::gflags::ValidateFileMustExist);

DEFINE_string(output_root, "/home/jake/results/change_detection/",
              "Full path to output folder.");

DEFINE_double(alpha, 0.3, "Minimum threshold for point match");

std::string output_folder = beam::CombinePaths(
    FLAGS_output_root,
    beam::ConvertTimeToDate(std::chrono::system_clock::now()));

pcl::PointCloud<pcl::PointXYZRGB>
DetectChanges(const pcl::PointCloud<pcl::PointXYZ> &cloud_source,
              const pcl::PointCloud<pcl::PointXYZ> &cloud_target,
              double threshold) {
  pcl::PointCloud<pcl::PointXYZRGB> change_cloud;

  beam::KdTree<pcl::PointXYZ> tree(cloud_target);

  for (std::size_t point_i = 0; point_i < cloud_source.size(); ++point_i) {
    const auto source_point = cloud_source.points[point_i];
    if (!std::isfinite(source_point.x) || !std::isfinite(source_point.y) ||
        !std::isfinite(source_point.z))
      continue;

    std::vector<uint32_t> nn_indices(1);
    std::vector<float> nn_distances(1);
    const auto neighbours =
        tree.radiusSearch(source_point, threshold, nn_indices, nn_distances);

    if (neighbours == 0) {
      pcl::PointXYZRGB point;
      point.x = source_point.x;
      point.y = source_point.y;
      point.z = source_point.z;
      point.r = 255;
      point.g = 0;
      point.b = 0;
      change_cloud.points.push_back(point);
    }
  }
  return change_cloud;
}

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (!boost::filesystem::is_directory(output_folder)) {
    boost::filesystem::create_directory(output_folder);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(FLAGS_map1_cloud_path, *cloud1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(FLAGS_map2_cloud_path, *cloud2);

  Eigen::Vector3f scan_voxel_size(.03, .03, .03);
  beam_filtering::VoxelDownsample<> downsampler(scan_voxel_size);

  BEAM_INFO("Filtering map 1");
  downsampler.SetInputCloud(cloud1);
  downsampler.Filter();
  auto map1_filtered = downsampler.GetFilteredCloud();

  BEAM_INFO("Filtering map 2");
  downsampler.SetInputCloud(cloud2);
  downsampler.Filter();
  auto map2_filtered = downsampler.GetFilteredCloud();

  BEAM_INFO("Detecting changes between map1 and map2.");
  auto change_cloud1 = DetectChanges(map1_filtered, map2_filtered, FLAGS_alpha);
  beam::SavePointCloud(output_folder + "/cloud1_to_cloud2.pcd", change_cloud1);

  BEAM_INFO("Detecting changes between map2 and map1.");
  auto change_cloud2 = DetectChanges(map2_filtered, map1_filtered, FLAGS_alpha);
  beam::SavePointCloud(output_folder + "/cloud2_to_cloud1.pcd", change_cloud2);

  return 0;
}
