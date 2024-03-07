#include <beam_filtering/VoxelDownsample.h>
#include <beam_utils/utils.h>
#include <pcl/point_types.h>

#include <gflags/gflags.h>

DEFINE_string(pointcloud_path, "", "Full path to point cloud.");
DEFINE_validator(pointcloud_path, &beam::gflags::ValidateFileMustExist);

struct EIGEN_ALIGN16 PointRoughness {
  float x;
  float y;
  float z;
  // float roughness;
  // float mean_curvature;
  // float gaussian_curvature;
  // float nn;
  float surface_density;
  float volume_density;
  PCL_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
};

// POINT_CLOUD_REGISTER_POINT_STRUCT(
//     PointRoughness,
//     (float, x, x)(float, y, y)(float, z, z)(float, roughness, roughness)(
//         float, mean_curvature, mean_curvature)(float, gaussian_curvature,
//                                                gaussian_curvature)(
//         float, nn, nn)(float, surface_density,
//                        surface_density)(float, volume_density,
//                        volume_density));

POINT_CLOUD_REGISTER_POINT_STRUCT(PointRoughness,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, surface_density,
                                      surface_density)(float, volume_density,
                                                       volume_density));

void Average(const pcl::PointCloud<PointRoughness> &cloud) {
  // {
  //   double sum = 0.0;
  //   int total_nan = 0;
  //   for (const auto &point : cloud.points) {
  //     if (std::isfinite(point.roughness)) {
  //       sum += static_cast<double>(point.roughness);
  //     } else {
  //       total_nan++;
  //     }
  //   }
  //   BEAM_INFO("Average roughness: {}",
  //             sum / static_cast<double>(cloud.points.size() - total_nan));
  //   BEAM_INFO("% NaN: {}",
  //             total_nan / static_cast<double>(cloud.points.size()));
  // }

  // {
  //   double sum = 0.0;
  //   int total_nan = 0;
  //   for (const auto &point : cloud.points) {
  //     if (std::isfinite(point.mean_curvature)) {
  //       sum += static_cast<double>(point.mean_curvature);
  //     } else {
  //       total_nan++;
  //     }
  //   }
  //   BEAM_INFO("Average mean_curvature: {}",
  //             sum / static_cast<double>(cloud.points.size() - total_nan));
  //   BEAM_INFO("% NaN: {}",
  //             total_nan / static_cast<double>(cloud.points.size()));
  // }

  // {
  //   double sum = 0.0;
  //   int total_nan = 0;
  //   for (const auto &point : cloud.points) {
  //     if (std::isfinite(point.gaussian_curvature)) {
  //       sum += static_cast<double>(point.gaussian_curvature);
  //     } else {
  //       total_nan++;
  //     }
  //   }
  //   BEAM_INFO("Average gaussian_curvature: {}",
  //             sum / static_cast<double>(cloud.points.size() - total_nan));
  //   BEAM_INFO("% NaN: {}",
  //             total_nan / static_cast<double>(cloud.points.size()));
  // }

  // {
  //   double sum = 0.0;
  //   int total_nan = 0;
  //   for (const auto &point : cloud.points) {
  //     if (std::isfinite(point.nn)) {
  //       sum += static_cast<double>(point.nn);
  //     } else {
  //       total_nan++;
  //     }
  //   }
  //   BEAM_INFO("Average number of neighbours: {}",
  //             sum / static_cast<double>(cloud.points.size() - total_nan));
  //   BEAM_INFO("% NaN: {}",
  //             total_nan / static_cast<double>(cloud.points.size()));
  // }

  {
    double sum = 0.0;
    int total_nan = 0;
    for (const auto &point : cloud.points) {
      if (std::isfinite(point.surface_density)) {
        sum += static_cast<double>(point.surface_density);
      } else {
        total_nan++;
      }
    }
    BEAM_INFO("Average surface_density: {}",
              sum / static_cast<double>(cloud.points.size() - total_nan));
    BEAM_INFO("% NaN: {}",
              total_nan / static_cast<double>(cloud.points.size()));
  }

  {
    double sum = 0.0;
    int total_nan = 0;
    for (const auto &point : cloud.points) {
      if (std::isfinite(point.volume_density)) {
        sum += static_cast<double>(point.volume_density);
      } else {
        total_nan++;
      }
    }
    BEAM_INFO("Average volume_density: {}",
              sum / static_cast<double>(cloud.points.size() - total_nan));
    BEAM_INFO("% NaN: {}",
              total_nan / static_cast<double>(cloud.points.size()));
  }
}

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  BEAM_INFO("\n\nProcessing {}", FLAGS_pointcloud_path);
  pcl::PointCloud<PointRoughness>::Ptr cloud(
      new pcl::PointCloud<PointRoughness>);

  pcl::PLYReader reader;
  reader.read(FLAGS_pointcloud_path, *cloud);
  Average(*cloud);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new
  // pcl::PointCloud<pcl::PointXYZ>);

  // pcl::PCDReader reader;
  // reader.read(FLAGS_pointcloud_path, *cloud);

  // std::string output =
  //     FLAGS_pointcloud_path.substr(0, FLAGS_pointcloud_path.size() - 4) +
  //     "_downsampled.pcd";

  // Eigen::Vector3f scan_voxel_size(.05, .05, .05);
  // beam_filtering::VoxelDownsample<> downsampler(scan_voxel_size);

  // downsampler.SetInputCloud(cloud);
  // downsampler.Filter();
  // auto map1_filtered = downsampler.GetFilteredCloud();
  // beam::SavePointCloud(output, map1_filtered);

  return 0;
}
