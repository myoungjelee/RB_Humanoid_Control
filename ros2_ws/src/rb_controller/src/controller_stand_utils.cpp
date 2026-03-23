#include "controller_stand_utils.hpp"

#include <algorithm>
#include <iterator>
#include <unordered_set>
#include <utility>

namespace rb_controller::internal
{

StandReferenceBuildResult build_stand_reference(
    const std::vector<std::string> &joint_names,
    const std::vector<double> &latest_joint_positions,
    const std::vector<double> &stand_q_ref_param,
    const std::vector<double> &stand_q_ref_trim_param,
    bool stand_hold_current_on_start)
{
  StandReferenceBuildResult result;
  if (joint_names.empty() || latest_joint_positions.size() != joint_names.size())
  {
    return result;
  }

  if (!stand_q_ref_param.empty())
  {
    if (stand_q_ref_param.size() == joint_names.size())
    {
      result.values = stand_q_ref_param;
      if (!stand_q_ref_trim_param.empty())
      {
        if (stand_q_ref_trim_param.size() == joint_names.size())
        {
          for (std::size_t i = 0; i < result.values.size(); ++i)
          {
            result.values[i] += stand_q_ref_trim_param[i];
          }
          result.trim_applied = true;
        }
        else
        {
          result.trim_size_mismatch = true;
        }
      }
      result.ready = true;
      return result;
    }
    result.q_ref_size_mismatch = true;
  }

  if (stand_hold_current_on_start)
  {
    result.values = latest_joint_positions;
    result.used_current_pose = true;
    result.ready = true;
  }
  return result;
}

StandControlMaskBuildResult build_stand_control_mask(
    const std::vector<std::string> &joint_names,
    const std::vector<std::string> &stand_control_joints_param)
{
  StandControlMaskBuildResult result;
  if (joint_names.empty())
  {
    return result;
  }

  result.mask.assign(joint_names.size(), 0U);
  if (stand_control_joints_param.empty())
  {
    std::fill(result.mask.begin(), result.mask.end(), 1U);
    result.all_joints = true;
    result.matched_count = joint_names.size();
    result.ready = true;
    return result;
  }

  std::unordered_set<std::string> requested(
      stand_control_joints_param.begin(), stand_control_joints_param.end());
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    if (requested.erase(joint_names[i]) > 0U)
    {
      result.mask[i] = 1U;
      ++result.matched_count;
    }
  }

  result.unknown_names.assign(requested.begin(), requested.end());
  result.ready = result.matched_count > 0U;
  return result;
}

StandGainMapBuildResult build_stand_gain_map(
    const std::vector<std::string> &joint_names,
    double stand_kp_scale_hip,
    double stand_kp_scale_knee,
    double stand_kp_scale_ankle,
    double stand_kp_scale_torso,
    double stand_kp_scale_other,
    double stand_kd_scale_hip,
    double stand_kd_scale_knee,
    double stand_kd_scale_ankle,
    double stand_kd_scale_torso,
    double stand_kd_scale_other)
{
  StandGainMapBuildResult result;
  if (joint_names.empty())
  {
    return result;
  }

  result.kp_scales.assign(joint_names.size(), stand_kp_scale_other);
  result.kd_scales.assign(joint_names.size(), stand_kd_scale_other);

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    const std::string &joint = joint_names[i];
    if (joint == "torso_joint")
    {
      result.kp_scales[i] = stand_kp_scale_torso;
      result.kd_scales[i] = stand_kd_scale_torso;
      ++result.torso_count;
      continue;
    }
    if (joint.find("_hip_") != std::string::npos)
    {
      result.kp_scales[i] = stand_kp_scale_hip;
      result.kd_scales[i] = stand_kd_scale_hip;
      ++result.hip_count;
      continue;
    }
    if (joint.find("_knee_") != std::string::npos)
    {
      result.kp_scales[i] = stand_kp_scale_knee;
      result.kd_scales[i] = stand_kd_scale_knee;
      ++result.knee_count;
      continue;
    }
    if (joint.find("_ankle_") != std::string::npos)
    {
      result.kp_scales[i] = stand_kp_scale_ankle;
      result.kd_scales[i] = stand_kd_scale_ankle;
      ++result.ankle_count;
      continue;
    }
    ++result.other_count;
  }

  result.ready = true;
  return result;
}

LimitAvoidMapBuildResult build_limit_avoid_map(
    const std::vector<std::string> &joint_names,
    const std::vector<std::string> &limit_avoid_joint_names_param,
    const std::vector<double> &limit_avoid_lower_param,
    const std::vector<double> &limit_avoid_upper_param)
{
  LimitAvoidMapBuildResult result;
  if (joint_names.empty())
  {
    return result;
  }

  const std::size_t n = limit_avoid_joint_names_param.size();
  if (n == 0U || limit_avoid_lower_param.size() != n || limit_avoid_upper_param.size() != n)
  {
    result.invalid_table = true;
    return result;
  }

  result.lower_per_joint.assign(joint_names.size(), 0.0);
  result.upper_per_joint.assign(joint_names.size(), 0.0);
  result.mask.assign(joint_names.size(), 0U);

  for (std::size_t i = 0; i < n; ++i)
  {
    const auto it = std::find(joint_names.begin(), joint_names.end(), limit_avoid_joint_names_param[i]);
    if (it == joint_names.end())
    {
      continue;
    }
    const std::size_t idx = static_cast<std::size_t>(std::distance(joint_names.begin(), it));
    const double lo = limit_avoid_lower_param[i];
    const double hi = limit_avoid_upper_param[i];
    if (hi <= lo)
    {
      continue;
    }
    result.lower_per_joint[idx] = lo;
    result.upper_per_joint[idx] = hi;
    result.mask[idx] = 1U;
    ++result.mapped_count;
  }

  if (result.mapped_count == 0U)
  {
    result.no_joint_match = true;
    return result;
  }

  result.ready = true;
  return result;
}

bool TiltJointIndices::has_roll_pair() const
{
  return (left_hip_roll >= 0) && (right_hip_roll >= 0);
}

bool TiltJointIndices::has_pitch_pair() const
{
  return ((left_hip_pitch >= 0) && (right_hip_pitch >= 0)) ||
         ((left_ankle_pitch >= 0) && (right_ankle_pitch >= 0)) ||
         ((left_knee >= 0) && (right_knee >= 0));
}

bool TiltJointIndices::ready() const
{
  return has_roll_pair() || has_pitch_pair();
}

TiltJointIndices build_tilt_joint_indices(const std::vector<std::string> &joint_names)
{
  TiltJointIndices result;
  if (joint_names.empty())
  {
    return result;
  }

  const auto find_idx = [&joint_names](const std::string &name) -> int
  {
    const auto it = std::find(joint_names.begin(), joint_names.end(), name);
    if (it == joint_names.end())
    {
      return -1;
    }
    return static_cast<int>(std::distance(joint_names.begin(), it));
  };

  result.left_hip_roll = find_idx("left_hip_roll_joint");
  result.right_hip_roll = find_idx("right_hip_roll_joint");
  result.left_ankle_roll = find_idx("left_ankle_roll_joint");
  result.right_ankle_roll = find_idx("right_ankle_roll_joint");
  result.left_hip_pitch = find_idx("left_hip_pitch_joint");
  result.right_hip_pitch = find_idx("right_hip_pitch_joint");
  result.left_ankle_pitch = find_idx("left_ankle_pitch_joint");
  result.right_ankle_pitch = find_idx("right_ankle_pitch_joint");
  result.left_knee = find_idx("left_knee_joint");
  result.right_knee = find_idx("right_knee_joint");
  result.torso = find_idx("torso_joint");
  return result;
}

}  // namespace rb_controller::internal
