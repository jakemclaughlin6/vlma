#pragma once

#include <Eigen/Dense>
#include <beam_mapping/Poses.h>
#include <tf2/buffer_core.h>

namespace vl_traj_alignment
{
  /**
   * @brief This class can be used to estimate the pose of any frame given its
   * timestamp and a tf2::BufferCore which contains the poses.
   *
   * Frames: Extrinsic calibration must be available on tf. Extrinsics can be
   * static or dynamic. To lookup the transforms needed, the sensor frame id must
   * be supplied, if it is not supplied then it will assume the odometry is
   * already in the correct frame.
   *
   */
  class PoseLookup
  {
  public:
    /**
     * @brief Constructor
     * @param poses shared pointer to the poses
     */
    PoseLookup(const beam_mapping::Poses &poses,
               const std::string &moving_frame, const std::string &world_frame);

    /**
     * @brief Gets estimate of baselink frame pose wrt world frame
     * @param T_WORLD_BASELINK reference to result
     * @param time pose time to lookup.
     * @param error_msg if unsuccesful, this message will explain why
     * @return true if pose lookup was successful
     */
    bool GetT_WORLD_SENSOR(Eigen::Matrix4d &T_WORLD_BASELINK,
                           const ros::Time &time);

  private:
    std::shared_ptr<tf2::BufferCore> poses_{nullptr};
    std::string moving_frame_;
    std::string world_frame_;
  };
} // namespace vl_map_refinement
