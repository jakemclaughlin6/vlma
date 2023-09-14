#include <cstdint>
#include <iostream>
#include <sstream>

#include <beam_utils/utils.h>

#include <boost/filesystem.hpp>

#include <gflags/gflags.h>

using namespace boost::lambda;

DEFINE_string(map1_cloud_path, "", "Full path to image folder.");
DEFINE_validator(map1_cloud_path, &beam::gflags::ValidateFileMustExist);


DEFINE_string(map2_cloud_path, "", "Full path to image folder.");
DEFINE_validator(map2_cloud_path, &beam::gflags::ValidateFileMustExist);

int main(int argc, char *argv[]) {

  // 1. load first point cloud
  // 2. load second point cloud
  // 3. for each point in the first cloud, find the closest point in the second
  // 4. if the closest point isnt within a range, then we highlight that point red
  // 5. we do the same for the second map to the first, but we highlight green

  return 0;
}
