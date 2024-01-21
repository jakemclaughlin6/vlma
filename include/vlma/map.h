#pragma once

#include <vlma/pose_lookup.h>

#include <beam_filtering/CropBox.h>
#include <beam_mapping/Poses.h>
#include <beam_utils/utils.h>
#include <sensor_msgs/Image.h>

#include <beam_calibration/CameraModel.h>
#include <beam_cv/Utils.h>

beam_filtering::CropBox<pcl::PointXYZI>
    cropper(Eigen::Vector3f(-1.0, -1.0, -1.0), Eigen::Vector3f(1.0, 1.0, 1.0),
            Eigen::Affine3f(Eigen::Matrix4f::Identity()), false);

namespace vlma {

struct VisualMapMatch {
  ros::Time map1_stamp;
  ros::Time map2_stamp;
  double visual_match_score;
};

class Map {
public:
  Map(const std::string &json_path, bool offset_poses,
      const std::string &world_frame) {
    nlohmann::json J_map;
    beam::ReadJson(json_path, J_map);

    // load camera model and generate rectified model
    bag_file = J_map["bag_file"];
    std::string cam_file = J_map["camera_intrinsics_file"];
    cam_model = beam_calibration::CameraModel::Create(cam_file);

    // load slam trajectory
    trajectory_file = J_map["trajectory_file"];
    trajectory.LoadFromJSON(trajectory_file);

    // offset the trajectory if desired
    if (offset_poses) {
      Eigen::Vector3d pert = beam::UniformRandomVector<3>(10.0, 15.0);
      pert.z() = 0.0;
      auto traj_poses = trajectory.GetPoses();
      std::vector<Eigen::Matrix4d, beam::AlignMat4d> new_poses;
      for (const auto pose : traj_poses) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0);
        T.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3) + pert;
        new_poses.push_back(T);
      }
      trajectory.SetPoses(new_poses);
    }

    std::string lidar_frame = J_map["lidar_frame_id"];
    vlma::PoseLookup traj_lookup_tmp(trajectory, lidar_frame, world_frame);
    trajectory_lookup = traj_lookup_tmp;
    topics.push_back(J_map["image_topic"]);
    topics.push_back(J_map["lidar_topic"]);
  }

  void AddPointCloud(const sensor_msgs::PointCloud2::Ptr &scan) {
    // convert to pcl
    pcl::PCLPointCloud2 pcl_pc2;
    beam::pcl_conversions::toPCL(*scan, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_in_lidar_frame(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *scan_in_lidar_frame);

    // remove points inside 2m box
    cropper.SetInputCloud(scan_in_lidar_frame);
    cropper.Filter();
    auto filtered_cloud = cropper.GetFilteredCloud();

    // transform into world frame
    Eigen::Matrix4d T_WORLD_LIDAR;
    if (trajectory_lookup.GetT_WORLD_SENSOR(T_WORLD_LIDAR,
                                            scan->header.stamp)) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr scan_in_world_frame(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud(filtered_cloud, *scan_in_world_frame,
                               T_WORLD_LIDAR);
      // store
      pointclouds[scan->header.stamp] = scan_in_world_frame;
    }
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetFullCloud() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_full_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto [stamp, cloud] : pointclouds) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cloud, *map_cloud);
      *map_full_cloud += *map_cloud;
    }
    return map_full_cloud;
  }

  vlma::PoseLookup trajectory_lookup;
  beam_mapping::Poses trajectory;
  std::string trajectory_file;
  std::shared_ptr<beam_calibration::CameraModel> cam_model;
  std::map<ros::Time, pcl::PointCloud<pcl::PointXYZI>::Ptr> pointclouds;
  std::string bag_file;
  std::vector<std::string> topics;
  std::map<ros::Time, sensor_msgs::Image> images;
  std::vector<cv::Mat> features;
};
} // namespace vlma