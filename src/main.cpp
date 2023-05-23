#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <vl_traj_alignment/pose_lookup.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/ImageDatabase.h>
#include <beam_cv/Utils.h>
#include <beam_mapping/Poses.h>
#include <beam_matching/IcpMatcher.h>
#include <beam_utils/utils.h>

#include <gflags/gflags.h>
#include <boost/progress.hpp>

DEFINE_string(map1_config_file, "", "Full path to config file to load (Required).");
DEFINE_validator(map1_config_file, &beam::gflags::ValidateFileMustExist);

DEFINE_string(map2_config_file, "", "Full path to config file to load (Required).");
DEFINE_validator(map2_config_file, &beam::gflags::ValidateFileMustExist);

int main(int argc, char *argv[])
{
  // Load config file
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (!boost::filesystem::exists(FLAGS_map1_config_file))
  {
    BEAM_ERROR("Invalid Config File Path.");
    return -1;
  }
  if (!boost::filesystem::exists(FLAGS_map2_config_file))
  {
    BEAM_ERROR("Invalid Config File Path.");
    return -1;
  }

  nlohmann::json J_map1;
  beam::ReadJson(FLAGS_map1_config_file, J_map1);

  nlohmann::json J_map2;
  beam::ReadJson(FLAGS_map2_config_file, J_map2);

  // Open bag and get its info
  std::string bag_file1 = J_map1["bag_file"];
  std::string bag_file2 = J_map2["bag_file"];
  std::vector<std::string> topics1{J_map1["image_topic"], J_map1["lidar_topic"]};
  std::vector<std::string> topics2{J_map2["image_topic"], J_map2["lidar_topic"]};

  // fixed frame in both trajectory files (must match)
  std::string world_frame = "world";

  // load trajectory from map 1
  std::string trajectory_file1 = J_map1["trajectory_file"];
  beam_mapping::Poses trajectory1;
  trajectory1.LoadFromJSON(trajectory_file1);
  std::string lidar_frame_id1 = J_map1["lidar_frame_id"];
  vl_traj_alignment::PoseLookup trajectory1_lookup(trajectory1, lidar_frame_id1, world_frame);

  // load trajectory from map 2
  std::string trajectory_file2 = J_map2["trajectory_file"];
  beam_mapping::Poses trajectory2;
  trajectory2.LoadFromJSON(trajectory_file2);
  std::string lidar_frame_id2 = J_map2["lidar_frame_id"];
  vl_traj_alignment::PoseLookup trajectory2_lookup(trajectory2, lidar_frame_id2, world_frame);

  // storage variables
  std::map<double, pcl::PointCloud<pcl::PointXYZ>::Ptr> pointclouds1;
  std::map<double, pcl::PointCloud<pcl::PointXYZ>::Ptr> pointclouds2;
  std::map<double, double> matched_stamps;               // visual matches from map1 to map2
  std::map<double, Eigen::Matrix4d> relative_trajectory; // map of relative poses from map 2 to map 1
  std::map<double, cv::Mat> images_map1;

  // matchings variables
  beam_cv::ImageDatabase image_db;
  beam_matching::IcpMatcher::Params matcher_params;
  beam_matching::IcpMatcher scan_registration(matcher_params);

  /*
    Process bag 1
  */
  BEAM_INFO("Processing {}", bag_file1);

  rosbag::Bag bag1;
  bag1.open(bag_file1);
  rosbag::View bag1_view(bag1, rosbag::TopicQuery(topics1));

  boost::progress_display loading_bar_bag1(bag1_view.size());

  ros::Time prev_frame_time(0.0);
  for (rosbag::MessageInstance const m : bag1_view)
  {
    // add lidar scan to storage
    sensor_msgs::PointCloud2::Ptr scan =
        m.instantiate<sensor_msgs::PointCloud2>();
    if (scan)
    {
      ++loading_bar_bag1;
      // convert to pcl
      pcl::PCLPointCloud2 pcl_pc2;
      beam::pcl_conversions::toPCL(*scan, pcl_pc2);
      pcl::PointCloud<pcl::PointXYZ>::Ptr scan_in_lidar_frame(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(pcl_pc2, *scan_in_lidar_frame);

      // transform into world frame
      Eigen::Matrix4d T_WORLD_LIDAR;
      if (trajectory1_lookup.GetT_WORLD_SENSOR(T_WORLD_LIDAR,
                                               scan->header.stamp))
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_in_world_frame(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(
            *scan_in_lidar_frame, *scan_in_world_frame, T_WORLD_LIDAR);
        // store
        pointclouds1[scan->header.stamp.toSec()] = scan_in_world_frame;
      }
    }

    // add image to database
    sensor_msgs::Image::Ptr buffer_image = m.instantiate<sensor_msgs::Image>();
    if (buffer_image)
    {
      ++loading_bar_bag1;
      ros::Time stamp = buffer_image->header.stamp;
      // add at 5 hz
      if (stamp - prev_frame_time >= ros::Duration(0.5))
      {
        cv::Mat image = beam_cv::OpenCVConversions::RosImgToMat(*buffer_image);
        image_db.AddImage(image, stamp);
        // images_map1[stamp.toSec()] = image;
        prev_frame_time = stamp;
      }
    }
  }

  /*
    Process bag 2
  */
  BEAM_INFO("Processing {}", bag_file2);

  rosbag::Bag bag2;
  bag2.open(bag_file2);
  rosbag::View bag2_view(bag1, rosbag::TopicQuery(topics2));

  boost::progress_display loading_bar_bag2(bag2_view.size());

  prev_frame_time = ros::Time(0.0);
  for (rosbag::MessageInstance const m : bag2_view)
  {
    // add lidar scan to storage
    sensor_msgs::PointCloud2::Ptr scan =
        m.instantiate<sensor_msgs::PointCloud2>();

    if (scan)
    {
      ++loading_bar_bag2;
      // convert to pcl
      pcl::PCLPointCloud2 pcl_pc2;
      beam::pcl_conversions::toPCL(*scan, pcl_pc2);
      pcl::PointCloud<pcl::PointXYZ>::Ptr scan_in_lidar_frame(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(pcl_pc2, *scan_in_lidar_frame);
      // transform into world frame
      Eigen::Matrix4d T_WORLD_LIDAR;
      if (trajectory2_lookup.GetT_WORLD_SENSOR(T_WORLD_LIDAR,
                                               scan->header.stamp))
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_in_world_frame(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(
            *scan_in_lidar_frame, *scan_in_world_frame, T_WORLD_LIDAR);
        // store
        pointclouds2[scan->header.stamp.toSec()] = scan_in_world_frame;
      }
    }

    // add image to database
    sensor_msgs::Image::Ptr buffer_image = m.instantiate<sensor_msgs::Image>();
    if (buffer_image)
    {
      ++loading_bar_bag2;
      // reduce rate
      // query image database for a match
      // store a timestamp to timestamp map
      ros::Time stamp = buffer_image->header.stamp;
      if (stamp - prev_frame_time >= ros::Duration(0.2))
      {
        cv::Mat image = beam_cv::OpenCVConversions::RosImgToMat(*buffer_image);
        prev_frame_time = stamp;

        auto results = image_db.QueryDatabase(image, 3);
        // require matches
        if (results.size() < 2)
        {
          continue;
        }
        auto timestamp = image_db.GetImageTimestamp(results[0].Id);
        const auto score = results[0].Score;
        if (timestamp.has_value() && score > 0.10)
        {
          matched_stamps[stamp.toSec()] = timestamp.value().toSec();
          // auto matching_image = images_map1[timestamp.value().toSec()];
          // cv::Mat H;
          // cv::hconcat(image, matching_image, H);
          // std::string image_filename = std::to_string(stamp.toNSec()) + "_" + std::to_string(score) + ".png";
          // cv::imwrite("/userhome/data/image_matches/" + image_filename, H);
        }
      }
    }
  }

  BEAM_INFO("Performing scan registration");
  // perform scan registration between matches
  for (const auto &[timestamp_map1, timestamp_map2] : matched_stamps)
  {
    auto itlow1 = pointclouds1.lower_bound(timestamp_map1);
    auto itlow2 = pointclouds2.lower_bound(timestamp_map2);
    if (itlow1 != pointclouds1.end() && itlow2 != pointclouds2.end())
    {
      std::cout << timestamp_map1 << " : " << timestamp_map2 << std::endl;
      auto pointcloud_map1 = itlow1->second;
      auto pointcloud_map2 = itlow2->second;

      pcl::PointCloud<pcl::PointXYZ> combined_cloud;

      combined_cloud += *pointcloud_map1;
      combined_cloud += *pointcloud_map2;

      std::string filename = std::to_string(itlow1->first) + ".pcd";
      auto success = beam::SavePointCloud("/userhome/data/clouds/" + filename, combined_cloud);
      std::cout << success << std::endl;
      // // scan registration between them
      // scan_registration.SetRef(pointcloud_map1);
      // scan_registration.SetTarget(pointcloud_map2);
      // scan_registration.Match();
      // // estimate info, don't use if its not high enough
      // auto T_map1_map2 = scan_registration.GetResult().inverse().matrix();
      // relative_trajectory[timestamp_map2] = T_map1_map2;
    }
  }

  // for each timestamp in trajectory 2, interpolate its relative pose from the relative trajectory
  // apply the pose to the pose in trajectory 2 and output a new trajectory file

  return 0;
}
