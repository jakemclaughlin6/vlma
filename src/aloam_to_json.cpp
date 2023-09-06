#include <beam_mapping/Poses.h>
#include <beam_utils/utils.h>

#include <filesystem>
#include <fstream>
#include <gflags/gflags.h>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

DEFINE_string(poses_path, "", "Full path to bag file.");
DEFINE_validator(poses_path, &beam::gflags::ValidateFileMustExist);

DEFINE_string(times_path, "", "Full path to bag file.");
DEFINE_validator(times_path, &beam::gflags::ValidateFileMustExist);

int main(int argc, char *argv[]) {
  // Load config file
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  beam_mapping::Poses trajectory;

  std::vector<Eigen::Matrix4d, beam::AlignMat4d> poses;
  std::vector<ros::Time> timestamps;

  {
    std::ifstream file(FLAGS_poses_path);
    std::string s;
    while (std::getline(file, s)) {
      std::vector<double> vals;
      if (beam::StringToNumericValues(" ", s, vals)) {
        std::vector<double> pose(vals.begin(), vals.end());
        Eigen::Matrix4d T = beam::VectorToEigenTransform(pose);
        poses.push_back(T);
      }
    }
  }

  {
    std::ifstream file(FLAGS_times_path);
    std::string s;
    while (std::getline(file, s)) {
      double time = std::stod(s);
      ros::Time timestamp(time);
      timestamps.push_back(timestamp);
    }
  }

  trajectory.SetPoses(poses);
  trajectory.SetTimeStamps(timestamps);
  trajectory.WriteToJSON("/home/jake/trajectory.json");

  return 0;
}
