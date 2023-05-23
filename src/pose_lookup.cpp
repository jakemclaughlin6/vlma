#include <vl_traj_alignment/pose_lookup.h>
#include <beam_utils/log.h>
#include <geometry_msgs/TransformStamped.h>

namespace vl_traj_alignment
{

  PoseLookup::PoseLookup(const beam_mapping::Poses &poses,
                         const std::string &moving_frame,
                         const std::string &world_frame)
      : world_frame_(world_frame), moving_frame_(moving_frame)
  {

    std::vector<ros::Time> timestamps = poses.GetTimeStamps();
    std::vector<Eigen::Matrix4d, beam::AlignMat4d> transforms = poses.GetPoses();

    int64_t cache_time =
        1.2 * (timestamps[timestamps.size() - 1].toSec() - timestamps[0].toSec());
    if (cache_time < 10)
    {
      cache_time = 10;
    }

    poses_ = std::make_shared<tf2::BufferCore>(ros::Duration(cache_time));

    for (int i = 0; i < transforms.size(); i++)
    {
      const Eigen::Matrix4d &T_WORLD_MOVINGFRAME = transforms[i];
      geometry_msgs::TransformStamped tf_stamped;

      tf_stamped.header.stamp = timestamps[i];
      tf_stamped.header.seq = i;
      tf_stamped.header.frame_id = world_frame_;
      tf_stamped.child_frame_id = moving_frame_;
      tf_stamped.transform.translation.x = T_WORLD_MOVINGFRAME(0, 3);
      tf_stamped.transform.translation.y = T_WORLD_MOVINGFRAME(1, 3);
      tf_stamped.transform.translation.z = T_WORLD_MOVINGFRAME(2, 3);
      Eigen::Matrix3d R = T_WORLD_MOVINGFRAME.block(0, 0, 3, 3);
      Eigen::Quaterniond q(R);
      tf_stamped.transform.rotation.x = q.x();
      tf_stamped.transform.rotation.y = q.y();
      tf_stamped.transform.rotation.z = q.z();
      tf_stamped.transform.rotation.w = q.w();

      poses_->setTransform(tf_stamped, "poses_file", false);
    }
  }

  bool PoseLookup::GetT_WORLD_SENSOR(Eigen::Matrix4d &T_WORLD_SENSOR,
                                     const ros::Time &time)
  {
    std::string error_msg;
    bool can_transform =
        poses_->canTransform(world_frame_, moving_frame_, time, &error_msg);

    if (!can_transform)
    {
      return false;
    }

    geometry_msgs::TransformStamped TROS_WORLD_SENSOR =
        poses_->lookupTransform(world_frame_, moving_frame_, time);

    Eigen::Matrix4f T_float{Eigen::Matrix4f::Identity()};
    T_float(0, 3) = TROS_WORLD_SENSOR.transform.translation.x;
    T_float(1, 3) = TROS_WORLD_SENSOR.transform.translation.y;
    T_float(2, 3) = TROS_WORLD_SENSOR.transform.translation.z;
    Eigen::Quaternionf q;
    q.x() = TROS_WORLD_SENSOR.transform.rotation.x;
    q.y() = TROS_WORLD_SENSOR.transform.rotation.y;
    q.z() = TROS_WORLD_SENSOR.transform.rotation.z;
    q.w() = TROS_WORLD_SENSOR.transform.rotation.w;
    T_float.block(0, 0, 3, 3) = q.toRotationMatrix();
    T_WORLD_SENSOR = T_float.cast<double>();

    return true;
  }
} // namespace vl_map_refinement
