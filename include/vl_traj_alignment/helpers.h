#pragma once

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <vl_traj_alignment/pose_lookup.h>

#include <beam_filtering/VoxelDownsample.h>
#include <beam_mapping/Poses.h>
#include <beam_matching/GicpMatcher.h>
#include <beam_utils/utils.h>
#include <pcl/common/centroid.h>

// DONT CHANGE INCLUDE ORDER
#include <beam_calibration/CameraModel.h>
#include <beam_cv/ImageDatabase.h>
#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/Utils.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/matchers/Matchers.h>
#include <beam_cv/trackers/Trackers.h>
#include <opencv2/core/eigen.hpp>

#include <vl_traj_alignment/scancontext/Scancontext.h>

double ComputeSCDist(SCManager &sc,
                     const pcl::PointCloud<pcl::PointXYZI> &cloud1,
                     const Eigen::Matrix4d &T_WORLD_LIDAR1,
                     const pcl::PointCloud<pcl::PointXYZI> &cloud2,
                     const Eigen::Matrix4d &T_WORLD_LIDAR2) {

  // transform aggregate scans into their lidar frame for scan context
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan1_in_lidar_frame(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(cloud1, *scan1_in_lidar_frame,
                           beam::InvertTransform(T_WORLD_LIDAR1));

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan2_in_lidar_frame(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(cloud2, *scan2_in_lidar_frame,
                           beam::InvertTransform(T_WORLD_LIDAR2));

  // if the resulting scan contexts are not similar enough, reject
  auto sc1 = sc.makeScancontext(*scan1_in_lidar_frame);
  auto sc2 = sc.makeScancontext(*scan2_in_lidar_frame);
  auto [dist, shift] = sc.distanceBtnScanContext(sc1, sc2);
  return dist;
}

std::pair<double, Eigen::Matrix4d>
RegisterScans(beam_matching::GicpMatcher &matcher,
              const pcl::PointCloud<pcl::PointXYZI> &cloud1,
              const pcl::PointCloud<pcl::PointXYZI> &cloud2) {
  // transform scan 2 into map 1 frame with the initial estimate
  pcl::PointCloud<pcl::PointXYZ>::Ptr map1_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(cloud1, *map1_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map2_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(cloud2, *map2_cloud);

  // attempt scan registration, keep best result
  matcher.SetRef(map1_cloud);
  matcher.SetTarget(map2_cloud);
  bool converged = matcher.Match();
  auto scan_reg_result = matcher.GetResult();
  double fitness = matcher.GetFitnessScore();
  Eigen::Matrix4d T_map1_map2 = scan_reg_result.inverse().matrix();
  return {fitness, T_map1_map2};
}

template <typename PointT>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
CreateScanRegistrationCloud(const pcl::PointCloud<PointT> &cloud1,
                            const pcl::PointCloud<PointT> &cloud2,
                            const pcl::PointCloud<PointT> &cloud2_in_cloud1) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr total_scan(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (const auto &p : cloud1.points) {
    pcl::PointXYZRGB point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.r = 255;
    point.g = 0;
    point.b = 0;
    total_scan->points.push_back(point);
  }
  for (const auto &p : cloud2.points) {
    pcl::PointXYZRGB point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.r = 0;
    point.g = 0;
    point.b = 255;
    total_scan->points.push_back(point);
  }
  for (const auto &p : cloud2_in_cloud1.points) {
    pcl::PointXYZRGB point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.r = 0;
    point.g = 255;
    point.b = 0;
    total_scan->points.push_back(point);
  }
  return total_scan;
}
