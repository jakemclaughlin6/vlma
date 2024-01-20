#include <beam_utils/utils.h>
#include <pcl/point_types.h>

#include <gflags/gflags.h>

DEFINE_string(pointcloud_path, "", "Full path to point cloud.");
DEFINE_validator(pointcloud_path, &beam::gflags::ValidateFileMustExist);

struct EIGEN_ALIGN16 PointRoughness {
  float x;
  float y;
  float z;
  float roughness;
  PCL_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointRoughness,
    (float, x, x)(float, y, y)(float, z, z)(float, roughness, roughness));

float AverageRoughness(const pcl::PointCloud<PointRoughness> &cloud) {
  double sum = 0.0;
  for (const auto &point : cloud.points) {
    if (std::isfinite(point.roughness)) {
      sum += static_cast<double>(point.roughness);
    }
  }

  return sum / static_cast<double>(cloud.points.size());
}

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  pcl::PointCloud<PointRoughness>::Ptr cloud(
      new pcl::PointCloud<PointRoughness>);

  pcl::PLYReader reader;
  reader.read(FLAGS_pointcloud_path, *cloud);

  BEAM_INFO("Roughness: {}", AverageRoughness(*cloud));

  return 0;
}
