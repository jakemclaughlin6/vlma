#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <vl_traj_alignment/pose_lookup.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_cv/ImageDatabase.h>
#include <beam_cv/Utils.h>
#include <beam_mapping/Poses.h>
#include <beam_matching/GicpMatcher.h>
#include <beam_utils/utils.h>
#include <beam_filtering/VoxelDownsample.h>

#include <gflags/gflags.h>
#include <boost/progress.hpp>

constexpr int RANSAC_ITERATIONS = 50;

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
  std::vector<std::pair<double, double>> matched_stamps; // visual matches from map2 to map1
  pcl::PointCloud<pcl::PointXYZ> map1;
  pcl::PointCloud<pcl::PointXYZ> map2;

  // matchings variables
  beam_cv::ImageDatabase image_db;

  BEAM_INFO("Processing {}", bag_file1);
  rosbag::Bag bag1;
  bag1.open(bag_file1);
  rosbag::View bag1_view(bag1, rosbag::TopicQuery(topics1));
  boost::progress_display loading_bar_bag1(bag1_view.size());
  ros::Time prev_frame_time(0.0);
  for (rosbag::MessageInstance const m : bag1_view)
  {
    ++loading_bar_bag1;

    // add lidar scan to storage
    sensor_msgs::PointCloud2::Ptr scan =
        m.instantiate<sensor_msgs::PointCloud2>();
    if (scan)
    {
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
        map1 += *scan_in_world_frame;
      }
    }

    // add image to database
    sensor_msgs::Image::Ptr buffer_image = m.instantiate<sensor_msgs::Image>();
    if (buffer_image)
    {
      ros::Time stamp = buffer_image->header.stamp;
      // add at 2 hz
      if (stamp - prev_frame_time >= ros::Duration(0.5))
      {
        prev_frame_time = stamp;
        cv::Mat image = beam_cv::OpenCVConversions::RosImgToMat(*buffer_image);
        image_db.AddImage(image, stamp);
      }
    }
  }

  /*
    Process bag 2
  */
  BEAM_INFO("Processing {}", bag_file2);
  rosbag::Bag bag2;
  bag2.open(bag_file2);
  rosbag::View bag2_view(bag2, rosbag::TopicQuery(topics2));
  boost::progress_display loading_bar_bag2(bag2_view.size());
  prev_frame_time = ros::Time(0.0);
  for (rosbag::MessageInstance const m : bag2_view)
  {
    ++loading_bar_bag2;

    // add lidar scan to storage
    sensor_msgs::PointCloud2::Ptr scan =
        m.instantiate<sensor_msgs::PointCloud2>();
    if (scan)
    {
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
        map2 += *scan_in_world_frame;
      }
    }

    // add image to database
    sensor_msgs::Image::Ptr buffer_image = m.instantiate<sensor_msgs::Image>();
    if (buffer_image)
    {
      ros::Time stamp = buffer_image->header.stamp;
      if (stamp - prev_frame_time >= ros::Duration(0.2))
      {
        prev_frame_time = stamp;
        cv::Mat image = beam_cv::OpenCVConversions::RosImgToMat(*buffer_image);
        const auto results = image_db.QueryDatabase(image, 1);
        // require matches
        if (results.size() < 1)
        {
          continue;
        }
        auto timestamp = image_db.GetImageTimestamp(results[0].Id);
        const auto score = results[0].Score;
        if (timestamp.has_value() && score > 0.10)
        {
          auto match = std::make_pair(stamp.toSec(), timestamp.value().toSec());
          matched_stamps.push_back(match);
        }
      }
    }
  }

  Eigen::Vector3f scan_voxel_size(.5, .5, .5);
  beam_filtering::VoxelDownsample<> downsampler(scan_voxel_size);

  BEAM_INFO("Filtering map 1");
  downsampler.SetInputCloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map1));
  downsampler.Filter();
  auto map1_filtered = downsampler.GetFilteredCloud();
  beam::SavePointCloud("/userhome/data/clouds/map1.pcd", map1_filtered);

  BEAM_INFO("Filtering map 2");
  downsampler.SetInputCloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map2));
  downsampler.Filter();
  auto map2_filtered = downsampler.GetFilteredCloud();
  beam::SavePointCloud("/userhome/data/clouds/map2.pcd", map2_filtered);

  BEAM_INFO("Performing RANSAC scan registration");
  beam_matching::GicpMatcher::Params matcher_params;
  matcher_params.max_iter = 50;
  beam_matching::GicpMatcher scan_registration(matcher_params);

  beam_matching::GicpMatcher::Params map_score_params;
  map_score_params.max_iter = 1;
  beam_matching::GicpMatcher map_scoring(map_score_params);

  const int N = matched_stamps.size() - 1;
  Eigen::Matrix4d T_map1_map2_best;
  for (int i = 0; i < RANSAC_ITERATIONS; i++)
  {
    int random_index = beam::randi(N, 0);
    const auto [timestamp_map2, timestamp_map1] = matched_stamps[random_index];
    auto itlow1 = pointclouds1.lower_bound(timestamp_map1);
    auto itlow2 = pointclouds2.lower_bound(timestamp_map2);
    if (itlow1 != pointclouds1.end() && itlow2 != pointclouds2.end())
    {
      pcl::PointCloud<pcl::PointXYZRGB> cloud1_xyzrgb;
      pcl::copyPointCloud(*itlow1->second, cloud1_xyzrgb);
      for (size_t i = 0; i < cloud1_xyzrgb.points.size(); i++)
      {
        cloud1_xyzrgb.points[i].r = 0;
        cloud1_xyzrgb.points[i].g = 0;
        cloud1_xyzrgb.points[i].b = 255;
      }

      pcl::PointCloud<pcl::PointXYZRGB> cloud2_xyzrgb;
      pcl::copyPointCloud(*itlow2->second, cloud2_xyzrgb);
      for (size_t i = 0; i < cloud2_xyzrgb.points.size(); i++)
      {
        cloud2_xyzrgb.points[i].r = 255;
        cloud2_xyzrgb.points[i].g = 0;
        cloud2_xyzrgb.points[i].b = 0;
      }
      pcl::PointCloud<pcl::PointXYZRGB> combined_cloud;
      combined_cloud += cloud1_xyzrgb;
      combined_cloud += cloud2_xyzrgb;
      std::string filename = std::to_string(random_index) + ".pcd";
      auto success = beam::SavePointCloud("/userhome/data/clouds/" + filename, combined_cloud);

      // perform scan registration on cloud
      scan_registration.SetRef(itlow1->second);
      scan_registration.SetTarget(itlow2->second);
      bool converged = scan_registration.Match();
      auto T_map1_map2 = scan_registration.GetResult().inverse().matrix();

      std::cout << std::fixed;
      std::cout << "\nIteration: " << i << std::endl;
      std::cout << "Index: " << random_index << std::endl;
      std::cout << "Timestamp match: [" << timestamp_map2 << " : " << timestamp_map1 << "]" << std::endl;
      std::cout << "GICP Converged: " << converged << std::endl;
      std::cout << "Result: \n"
                << T_map1_map2 << std::endl;

      // apply estimate to map2
      pcl::PointCloud<pcl::PointXYZ>::Ptr map2_in_map1(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(
          map2_filtered, *map2_in_map1, T_map1_map2);
          
      // compute GICP fitness score of map1-map2 without running any cycles
      map_scoring.SetRef(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map1_filtered));
      map_scoring.SetTarget(map2_in_map1);
      map_scoring.Match();
      auto score = map_scoring.GetFitnessScore();
      std::cout << "Total fitness score: " << score << std::endl;
    }
  }

  return 0;
}
