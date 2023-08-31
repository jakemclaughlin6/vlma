#include <cstdint>
#include <iostream>
#include <sstream>

#include <beam_cv/OpenCVConversions.h>
#include <beam_utils/utils.h>
#include <rosbag/bag.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/progress.hpp>

#include <filesystem>
#include <gflags/gflags.h>

using namespace boost::lambda;

DEFINE_string(rgb_path, "", "Full path to image folder.");
DEFINE_validator(rgb_path, &beam::gflags::ValidateDirMustExist);

DEFINE_string(lidar_path, "", "Full path to lidar folder.");
DEFINE_validator(lidar_path, &beam::gflags::ValidateDirMustExist);

int main(int argc, char *argv[]) {
  // Load config file
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  rosbag::Bag bag;
  bag.open("/home/jake/data/conSLAM/sequence1/test.bag",
           rosbag::bagmode::Write);

  uint64_t img_seq = 0;

  boost::filesystem::path rgb_folder(FLAGS_rgb_path);
  int num_images =
      std::count_if(boost::filesystem::directory_iterator(rgb_folder),
                    boost::filesystem::directory_iterator(),
                    static_cast<bool (*)(const boost::filesystem::path &)>(
                        boost::filesystem::is_regular_file));
  boost::progress_display loading_bar1(num_images);
  if (boost::filesystem::is_directory(rgb_folder)) {
    for (auto &entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(rgb_folder), {})) {
      ++loading_bar1;

      cv::Mat image = cv::imread(entry.path().string(), cv::IMREAD_COLOR);

      uint64_t nsec;
      std::istringstream iss(entry.path().stem().string());
      iss >> nsec;
      ros::Time timestamp = beam::NSecToRos(nsec * 1000);

      if (nsec <= 0)
        continue;
      std_msgs::Header header;
      header.seq = img_seq++;
      header.frame_id = "image";
      header.stamp = timestamp;
      sensor_msgs::Image out_msg =
          beam_cv::OpenCVConversions::MatToRosImg(image, header, "bgr8");
      bag.write("image", timestamp, out_msg);
    }
  } else {
    BEAM_ERROR("Not a valid directory.");
    return -1;
  }

  boost::filesystem::path lidar_folder(FLAGS_lidar_path);
  int num_clouds =
      std::count_if(boost::filesystem::directory_iterator(lidar_folder),
                    boost::filesystem::directory_iterator(),
                    static_cast<bool (*)(const boost::filesystem::path &)>(
                        boost::filesystem::is_regular_file));
  boost::progress_display loading_bar2(num_clouds);
  if (boost::filesystem::is_directory(lidar_folder)) {
    for (auto &entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(lidar_folder), {})) {
      ++loading_bar2;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZ>);

      if (pcl::io::loadPCDFile<pcl::PointXYZ>(entry.path().string(), *cloud) ==
          -1) {
        continue;
      }

      uint64_t nsec;
      std::istringstream iss(entry.path().stem().string());
      iss >> nsec;
      ros::Time timestamp = beam::NSecToRos(nsec * 1000);

      if (nsec <= 0)
        continue;

      sensor_msgs::PointCloud2 out_msg =
          beam::PCLToROS(*cloud, timestamp, "cloud");
      bag.write("cloud", timestamp, out_msg);
    }
  } else {
    BEAM_ERROR("Not a valid directory.");
    return -1;
  }

  bag.close();

  return 0;
}
