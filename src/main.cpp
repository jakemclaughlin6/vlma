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

#include <vl_traj_alignment/helpers.h>
#include <vl_traj_alignment/scancontext/Scancontext.h>

#include <boost/progress.hpp>
#include <gflags/gflags.h>

constexpr int RANSAC_ITERATIONS = 50;

DEFINE_string(map1_config_file, "",
              "Full path to config file to load (Required).");
DEFINE_validator(map1_config_file, &beam::gflags::ValidateFileMustExist);

DEFINE_string(map2_config_file, "",
              "Full path to config file to load (Required).");
DEFINE_validator(map2_config_file, &beam::gflags::ValidateFileMustExist);

DEFINE_double(alignment_alpha, 1.0,
              "Amount ot align map2 to map1 (1.0 align map2 completely to "
              "map1, 0.0 align map1 completely to map2)");

DEFINE_bool(offset_map2, false,
            "Whether to offset the second map trajectory (for testing)");

std::string output_folder = beam::CombinePaths(
    "/home/jake/results/vl_traj_alignment/",
    beam::ConvertTimeToDate(std::chrono::system_clock::now()));

namespace vl_traj_alignment {
class Map {
public:
  Map(const std::string &json_path, bool offset_poses,
      const std::string &world_frame) {
    static int map_num = 1;
    nlohmann::json J_map;
    beam::ReadJson(json_path, J_map);

    bag_file = J_map["bag_file"];
    std::string cam_file = J_map["camera_intrinsics_file"];
    cam_model = beam_calibration::CameraModel::Create(cam_file);
    trajectory_file = J_map["trajectory_file"];
    trajectory.LoadFromJSON(trajectory_file);

    // offset the trajectory if desired
    if (offset_poses) {
      Eigen::Vector3d pert = beam::UniformRandomVector<3>(3.0, 7.0);
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

    // save trajectory as point cloud
    pcl::PointCloud<pcl::PointXYZRGB> trajectory_cloud;
    for (const auto &T : trajectory.GetPoses()) {
      trajectory_cloud = beam::AddFrameToCloud(trajectory_cloud, T);
    }
    beam::SavePointCloud(output_folder + "/map" + std::to_string(map_num) +
                             "_trajectory.pcd",
                         trajectory_cloud);
    map_num++;

    std::string lidar_frame = J_map["lidar_frame_id"];
    vl_traj_alignment::PoseLookup traj_lookup_tmp(trajectory, lidar_frame,
                                                  world_frame);
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

    // transform into world frame
    Eigen::Matrix4d T_WORLD_LIDAR;
    if (trajectory_lookup.GetT_WORLD_SENSOR(T_WORLD_LIDAR,
                                            scan->header.stamp)) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr scan_in_world_frame(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud(*scan_in_lidar_frame, *scan_in_world_frame,
                               T_WORLD_LIDAR);
      // store
      pointclouds[scan->header.stamp] = scan_in_world_frame;
    }
    return;
  }

  vl_traj_alignment::PoseLookup trajectory_lookup;
  beam_mapping::Poses trajectory;
  std::string trajectory_file;
  std::shared_ptr<beam_calibration::CameraModel> cam_model;
  std::map<ros::Time, pcl::PointCloud<pcl::PointXYZI>::Ptr> pointclouds;
  std::string bag_file;
  std::vector<std::string> topics;
};
} // namespace vl_traj_alignment

int main(int argc, char *argv[]) {
  // Load config file
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (!boost::filesystem::exists(FLAGS_map1_config_file)) {
    BEAM_ERROR("Invalid Config File Path.");
    return -1;
  }
  if (!boost::filesystem::exists(FLAGS_map2_config_file)) {
    BEAM_ERROR("Invalid Config File Path.");
    return -1;
  }

  // load each map object
  vl_traj_alignment::Map map1(FLAGS_map1_config_file, false, "world");
  vl_traj_alignment::Map map2(FLAGS_map2_config_file, FLAGS_offset_map2,
                              "world");

  // setup output folder
  if (!boost::filesystem::is_directory(output_folder)) {
    boost::filesystem::create_directory(output_folder);
  }
  if (!boost::filesystem::is_directory(output_folder + "/image_matches")) {
    boost::filesystem::create_directory(output_folder + "/image_matches");
  }
  if (!boost::filesystem::is_directory(output_folder + "/image_matches/map1")) {
    boost::filesystem::create_directory(output_folder + "/image_matches/map1");
  }
  if (!boost::filesystem::is_directory(output_folder + "/image_matches/map2")) {
    boost::filesystem::create_directory(output_folder + "/image_matches/map2");
  }
  if (!boost::filesystem::is_directory(output_folder + "/cloud_matches")) {
    boost::filesystem::create_directory(output_folder + "/cloud_matches");
  }

  // storage variables
  std::vector<std::pair<ros::Time, ros::Time>>
      matched_stamps; // visual matches from map2 to map1
  std::vector<double> match_scores;
  std::map<ros::Time, sensor_msgs::Image> db_images;

  // visual feature detector, desciptor, matcher and tracker
  beam_cv::ORBDetector::Params detector_params{
      300, 1.2, 8, 31, cv::ORB::FAST_SCORE, 20, 3, 2};
  beam_cv::ORBDescriptor::Params descriptor_params;
  auto detector = std::make_shared<beam_cv::ORBDetector>(detector_params);
  auto descriptor = std::make_shared<beam_cv::ORBDescriptor>(descriptor_params);
  auto matcher = std::make_shared<beam_cv::BFMatcher>(cv::NORM_HAMMING, false,
                                                      false, 0.8, 5, true);
  beam_cv::KLTracker::Params tracker_params;
  auto tracker = std::make_shared<beam_cv::KLTracker>(tracker_params, detector,
                                                      descriptor, 500);

  /*************************************
                Process bag 1
  **************************************/
  std::vector<cv::Mat> features;
  BEAM_INFO("Processing {}", map1.bag_file);
  rosbag::Bag bag1;
  bag1.open(map1.bag_file);
  rosbag::View bag1_view(bag1, rosbag::TopicQuery(map1.topics));
  boost::progress_display loading_bar_bag1(bag1_view.size());
  ros::Time prev_frame_time(0.0);
  for (rosbag::MessageInstance const m : bag1_view) {
    ++loading_bar_bag1;

    // add lidar scan to storage
    auto scan = m.instantiate<sensor_msgs::PointCloud2>();
    if (scan) {
      map1.AddPointCloud(scan);
      continue;
    }

    // add image to database if we've detected a keyframe
    auto buffer_image = m.instantiate<sensor_msgs::Image>();
    if (buffer_image) {
      ros::Time stamp = buffer_image->header.stamp;
      cv::Mat image = beam_cv::OpenCVConversions::RosImgToMat(*buffer_image);
      tracker->AddImage(image, stamp);

      auto ids = tracker->GetLandmarkIDsInImage(stamp);
      for (const auto id : ids) {
        cv::Mat desc = tracker->GetDescriptor(stamp, id);
        features.push_back(desc);
      }

      if (prev_frame_time == ros::Time(0.0)) {
        prev_frame_time = stamp;
        db_images.insert({stamp, *buffer_image});
      } else {
        auto prev_ids = tracker->GetLandmarkIDsInImage(prev_frame_time);
        size_t total_lms = prev_ids.size();
        size_t num_matches = 0;
        for (auto id : prev_ids) {
          try {
            tracker->Get(stamp, id);
            num_matches++;
          } catch (const std::out_of_range &oor) {
          }
        }
        float percent_tracked = (float)num_matches / (float)total_lms;
        if (percent_tracked <= 0.6) {
          prev_frame_time = stamp;
          db_images.insert({stamp, *buffer_image});
        }
      }
    }
  }

  BEAM_INFO("Training vocabulary on map 1, numer of features: {}",
            features.size());
  DBoW3::Vocabulary map1_vocabulary(9, 3, DBoW3::TF_IDF, DBoW3::L1_NORM);
  map1_vocabulary.create(features);

  BEAM_INFO("Building image database for map 1.");
  boost::progress_display loading_bar_imagedb(db_images.size());
  beam_cv::ImageDatabase image_db(detector_params, descriptor_params,
                                  map1_vocabulary);
  for (const auto &[stamp, img_msg] : db_images) {
    ++loading_bar_imagedb;
    image_db.AddImage(beam_cv::OpenCVConversions::RosImgToMat(img_msg), stamp);
  }

  /*************************************
                Process bag 2
  **************************************/
  int idx = 0;
  BEAM_INFO("Processing {}", map2.bag_file);
  rosbag::Bag bag2;
  bag2.open(map2.bag_file);
  rosbag::View bag2_view(bag2, rosbag::TopicQuery(map2.topics));
  boost::progress_display loading_bar_bag2(bag2_view.size());
  prev_frame_time = ros::Time(0.0);
  for (rosbag::MessageInstance const m : bag2_view) {
    ++loading_bar_bag2;

    // add lidar scan to storage
    auto scan = m.instantiate<sensor_msgs::PointCloud2>();
    if (scan) {
      map2.AddPointCloud(scan);
      continue;
    }

    // query image database to get initial matches
    auto buffer_image = m.instantiate<sensor_msgs::Image>();
    if (buffer_image) {
      const auto stamp_map2 = buffer_image->header.stamp;
      if (stamp_map2 - prev_frame_time >= ros::Duration(0.2)) {
        prev_frame_time = stamp_map2;
        cv::Mat image = beam_cv::OpenCVConversions::RosImgToMat(*buffer_image);
        const auto results = image_db.QueryDatabase(image, 1);
        // require matches
        if (results.size() < 1) {
          continue;
        }
        const auto score = results[0].Score;
        if (!image_db.GetImageTimestamp(results[0].Id).has_value() ||
            score < 0.5) {
          continue;
        }
        const auto stamp_map1 =
            image_db.GetImageTimestamp(results[0].Id).value();
        if (db_images.find(stamp_map1) == db_images.end()) {
          continue;
        }
        cv::Mat map1_image =
            beam_cv::OpenCVConversions::RosImgToMat(db_images.at(stamp_map1));

        // match descriptors directly
        std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels_map2;
        std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels_map1;
        beam_cv::DetectComputeAndMatch(image, map1_image, descriptor, detector,
                                       matcher, pixels_map2, pixels_map1);
        if (pixels_map1.size() < 7) {
          continue;
        }
        // compute relative pose
        auto T_map2_map1_offset =
            beam_cv::RelativePoseEstimator::RANSACEstimator(
                map1.cam_model, map2.cam_model, pixels_map1, pixels_map2,
                beam_cv::EstimatorMethod::SEVENPOINT, 100, 10.0);
        if (!T_map2_map1_offset.has_value()) {
          continue;
        }
        const int num_inliers = beam_cv::CheckInliers(
            map1.cam_model, map2.cam_model, pixels_map1, pixels_map2,
            Eigen::Matrix4d::Identity(), T_map2_map1_offset.value(), 10.0);
        const float inlier_ratio =
            (float)num_inliers / (float)pixels_map1.size();
        if (inlier_ratio < 0.5) {
          continue;
        }

        cv::imwrite(output_folder + "/image_matches/map1/" +
                        std::to_string(idx) + ".png",
                    map1_image);
        cv::imwrite(output_folder + "/image_matches/map2/" +
                        std::to_string(idx) + ".png",
                    image);
        idx++;

        // store candidate matches
        matched_stamps.push_back({stamp_map2, stamp_map1});
        match_scores.push_back(score);
      }
    }
  }

  /*************************************
         Get initial full maps
  **************************************/
  pcl::PointCloud<pcl::PointXYZ>::Ptr map1_full_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto [stamp, cloud] : map1.pointclouds) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *map_cloud);
    *map1_full_cloud += *map_cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr map2_full_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto [stamp, cloud] : map2.pointclouds) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *map_cloud);
    *map2_full_cloud += *map_cloud;
  }
  Eigen::Vector3f scan_voxel_size(.3, .3, .3);
  beam_filtering::VoxelDownsample<> downsampler(scan_voxel_size);

  BEAM_INFO("Filtering map 1");
  downsampler.SetInputCloud(map1_full_cloud);
  downsampler.Filter();
  auto map1_filtered = downsampler.GetFilteredCloud();
  beam::SavePointCloud(output_folder + "/map1.pcd", map1_filtered);

  BEAM_INFO("Filtering map 2");
  downsampler.SetInputCloud(map2_full_cloud);
  downsampler.Filter();
  auto map2_filtered = downsampler.GetFilteredCloud();
  beam::SavePointCloud(output_folder + "/map2.pcd", map2_filtered);

  /*************************************
         Filter candidate matches
  **************************************/
  beam_matching::GicpMatcher::Params matcher_params;
  matcher_params.max_iter = 50;
  beam_matching::GicpMatcher scan_registration(matcher_params);
  SCManager scan_context_extractor;

  // lambda to get a scan as well as its N neighbours
  auto get_neighbourhood_scan =
      [&](auto pointclouds, const ros::Time timestamp,
          const size_t radius) -> pcl::PointCloud<pcl::PointXYZI>::Ptr {
    auto itlow = pointclouds.lower_bound(timestamp);
    if (itlow == pointclouds.end() || itlow == pointclouds.begin()) {
      return nullptr;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr aggregate_scan(
        new pcl::PointCloud<pcl::PointXYZI>);
    *aggregate_scan += *pointclouds[itlow->first];

    auto cur_prev = itlow, cur_next = itlow;
    for (int i = 0; i < radius; i++) {
      cur_prev = std::prev(cur_prev);
      if (cur_prev != std::prev(pointclouds.begin())) {
        if (pointclouds.find(cur_prev->first) != pointclouds.end()) {
          *aggregate_scan += *pointclouds[cur_prev->first];
        }
      }
      cur_next = std::next(cur_next);
      if (cur_next != pointclouds.end()) {
        if (pointclouds.find(cur_next->first) != pointclouds.end()) {
          *aggregate_scan += *pointclouds[cur_next->first];
        }
      }
    }
    return aggregate_scan;
  };

  std::map<ros::Time, Eigen::Matrix4d>
      relative_poses;                        // <map2 stamp: T_map1_map2>
  std::map<ros::Time, double> best_fitness;  // <map2 stamp : best fitness>
  std::map<ros::Time, ros::Time> best_match; // <map2 stamp : map1 stamp>

  if (matched_stamps.empty()) {
    std::cout << "No visual matches! Are these two maps of the same area?"
              << std::endl;
    throw std::runtime_error(
        "No visual matches! Are these two maps of the same area?");
  }

  BEAM_INFO("Filtering visual matches with Scancontext");
  // boost::progress_display pose_estimation_loading_bar(matched_stamps.size());
  size_t i = 0;
  for (const auto &[timestamp_map2, timestamp_map1] : matched_stamps) {
    // ++pose_estimation_loading_bar;
    // get aggregate scans around target timestamp
    auto map1_scan =
        get_neighbourhood_scan(map1.pointclouds, timestamp_map1, 20);
    auto map2_scan =
        get_neighbourhood_scan(map2.pointclouds, timestamp_map2, 20);
    if (map1_scan && map2_scan) {

      // get poses of each lidar scan
      Eigen::Matrix4d T_WORLD_LIDAR1, T_WORLD_LIDAR2;
      if (!map1.trajectory_lookup.GetT_WORLD_SENSOR(T_WORLD_LIDAR1,
                                                    timestamp_map1) ||
          !map2.trajectory_lookup.GetT_WORLD_SENSOR(T_WORLD_LIDAR2,
                                                    timestamp_map2)) {
        continue;
      }

      // compute scan context distance
      auto dist = ComputeSCDist(scan_context_extractor, *map1_scan,
                                T_WORLD_LIDAR1, *map2_scan, T_WORLD_LIDAR2);
      if (dist > 0.15) {
        continue;
      }

      // attempt scan registration
      double fitness;
      Eigen::Matrix4d T_map1_map2;
      bool converged = RegisterScans(scan_registration, *map1_scan, *map2_scan,
                                     fitness, T_map1_map2);
      if (fitness > 0.5 || !converged) {
        continue;
      }

      // check its fitness on a larger aggregate scan
      auto map1_scan_l =
          get_neighbourhood_scan(map1.pointclouds, timestamp_map1, 40);
      auto map2_scan_l =
          get_neighbourhood_scan(map2.pointclouds, timestamp_map2, 40);

      if (!map1_scan_l->empty() && !map2_scan_l->empty()) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr map2_aligned_to_map1(
            new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*map2_scan_l, *map2_aligned_to_map1,
                                 T_map1_map2);
        double ransac_fitness =
            beam::PointCloudError(*map1_scan_l, *map2_aligned_to_map1);
        // visualize scan registration
        auto total_scan = CreateScanRegistrationCloud(
            *map1_scan_l, *map2_scan_l, *map2_aligned_to_map1);
        beam::SavePointCloud(output_folder + "/cloud_matches/" +
                                 std::to_string(timestamp_map1.toNSec()) + "_" +
                                 std::to_string(timestamp_map2.toNSec()) +
                                 ".pcd",
                             *total_scan);

        // log results
        std::cout << std::fixed << std::endl;
        std::cout << timestamp_map1 << " : " << timestamp_map2 << std::endl;
        std::cout << "Pose: \n" << T_map1_map2 << std::endl;
        std::cout << "Fitness: " << fitness << std::endl;
        std::cout << "RANSAC Fitness: " << ransac_fitness << std::endl;
        std::cout << "SC Dist: " << dist << std::endl;
        std::cout << "DBoW score: " << match_scores[i] << std::endl;
        i++;

        // update best result
        if (relative_poses.find(timestamp_map2) != relative_poses.end()) {
          if (best_fitness[timestamp_map2] > ransac_fitness) {
            relative_poses[timestamp_map2] = T_map1_map2;
            best_fitness[timestamp_map2] = ransac_fitness;
            best_match[timestamp_map2] = timestamp_map1;
          }
        } else {
          relative_poses[timestamp_map2] = T_map1_map2;
          best_fitness[timestamp_map2] = ransac_fitness;
          best_match[timestamp_map2] = timestamp_map1;
        }
      }
    }
  }

  /*************************************
      Apply resulting relative poses
  **************************************/
  BEAM_INFO("Estimating spline for relative trajectory.");
  // create spline for relative estimates to smooth it
  std::vector<beam::Pose> relative_trajectory;
  for (const auto &[stamp, T_FIXED_MOVING] : relative_poses) {
    beam::Pose pose{T_FIXED_MOVING, stamp.toNSec()};
    relative_trajectory.push_back(pose);
  }
  beam::BsplineSE3 spline;
  spline.feed_trajectory(relative_trajectory);

  // get first and last pose rather than using the spline extrpolation
  const auto start_pose = relative_trajectory[0];
  const auto end_pose = relative_trajectory[relative_trajectory.size() - 1];

  BEAM_INFO("Applying resulting relative poses.");
  // apply relative estimates to map2 trajectory
  std::vector<Eigen::Matrix4d, beam::AlignMat4d> map2_updated_poses;
  const auto map2_poses = map2.trajectory.GetPoses();
  const auto map2_stamps = map2.trajectory.GetTimeStamps();
  const auto N = map2_poses.size();
  for (int i = 0; i < N; i++) {
    const auto T_map2_lidar = map2_poses[i];
    const auto stamp = map2_stamps[i];
    Eigen::Matrix4d T_map1_map2;
    // get pose or "extrapolate"
    if (!spline.get_pose(stamp.toSec(), T_map1_map2)) {
      if (stamp.toNSec() < start_pose.timestampInNs) {
        T_map1_map2 = start_pose.T_FIXED_MOVING;
      } else if (stamp.toNSec() > end_pose.timestampInNs) {
        T_map1_map2 = end_pose.T_FIXED_MOVING;
      }
    }

    Eigen::Matrix4d updated_pose = T_map1_map2 * T_map2_lidar;
    map2_updated_poses.push_back(updated_pose);
  }

  beam_mapping::Poses map2_updated_trajectory;
  map2_updated_trajectory.SetTimeStamps(map2_stamps);
  map2_updated_trajectory.SetPoses(map2_updated_poses);
  std::string output_file =
      map2.trajectory_file.substr(0, map2.trajectory_file.length() - 5) +
      "_aligned.json";
  map2_updated_trajectory.WriteToJSON(output_file);

  /*********************************************
    Create final aligned map for visualization
  **********************************************/
  BEAM_INFO("Creating final aligned map for visualization.");
  pcl::PointCloud<pcl::PointXYZ>::Ptr map2_full_cloud_aligned(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto [stamp, cloud] : map2.pointclouds) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *map_cloud);
    Eigen::Matrix4d T_map1_map2;

    // get pose or "extrapolate"
    if (!spline.get_pose(stamp.toSec(), T_map1_map2)) {
      if (stamp.toNSec() < start_pose.timestampInNs) {
        T_map1_map2 = start_pose.T_FIXED_MOVING;
      } else if (stamp.toNSec() > end_pose.timestampInNs) {
        T_map1_map2 = end_pose.T_FIXED_MOVING;
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*map_cloud, *aligned_cloud, T_map1_map2);

    *map2_full_cloud_aligned += *aligned_cloud;
  }

  BEAM_INFO("Filtering aligned map 2");
  downsampler.SetInputCloud(map2_full_cloud_aligned);
  downsampler.Filter();
  auto map2_aligned_filtered = downsampler.GetFilteredCloud();
  beam::SavePointCloud(output_folder + "/map2_aligned.pcd",
                       map2_aligned_filtered);

  return 0;
}