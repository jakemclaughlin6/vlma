#include <beam_mapping/Poses.h>
#include <beam_utils/utils.h>

#include <filesystem>
#include <gflags/gflags.h>
namespace fs = std::filesystem;

DEFINE_string(bag_path, "", "Full path to bag file.");
DEFINE_validator(bag_path, &beam::gflags::ValidateFileMustExist);

DEFINE_string(path_topic, "",
              "Topic of the path to transform into a json trajectory.");

int main(int argc, char *argv[]) {
  // Load config file
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (!boost::filesystem::exists(FLAGS_bag_path)) {
    BEAM_ERROR("Invalid Config Bag file path.");
    return -1;
  }
  fs::path bag_path(FLAGS_bag_path);
  std::string directory = bag_path.parent_path().string();
  beam_mapping::Poses poses;
  poses.LoadFromBAG(FLAGS_bag_path, FLAGS_path_topic);
  poses.WriteToJSON(directory);
  return 0;
}
