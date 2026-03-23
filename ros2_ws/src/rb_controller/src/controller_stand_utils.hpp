#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rb_controller::internal
{

struct StandReferenceBuildResult
{
  bool ready{false};
  bool trim_applied{false};
  bool used_current_pose{false};
  bool q_ref_size_mismatch{false};
  bool trim_size_mismatch{false};
  std::vector<double> values{};
};

StandReferenceBuildResult build_stand_reference(
    const std::vector<std::string> &joint_names,
    const std::vector<double> &latest_joint_positions,
    const std::vector<double> &stand_q_ref_param,
    const std::vector<double> &stand_q_ref_trim_param,
    bool stand_hold_current_on_start);

struct StandControlMaskBuildResult
{
  bool ready{false};
  bool all_joints{false};
  std::size_t matched_count{0U};
  std::vector<std::uint8_t> mask{};
  std::vector<std::string> unknown_names{};
};

StandControlMaskBuildResult build_stand_control_mask(
    const std::vector<std::string> &joint_names,
    const std::vector<std::string> &stand_control_joints_param);

struct StandGainMapBuildResult
{
  bool ready{false};
  std::vector<double> kp_scales{};
  std::vector<double> kd_scales{};
  std::size_t hip_count{0U};
  std::size_t knee_count{0U};
  std::size_t ankle_count{0U};
  std::size_t torso_count{0U};
  std::size_t other_count{0U};
};

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
    double stand_kd_scale_other);

struct LimitAvoidMapBuildResult
{
  bool ready{false};
  bool invalid_table{false};
  bool no_joint_match{false};
  std::size_t mapped_count{0U};
  std::vector<std::uint8_t> mask{};
  std::vector<double> lower_per_joint{};
  std::vector<double> upper_per_joint{};
};

LimitAvoidMapBuildResult build_limit_avoid_map(
    const std::vector<std::string> &joint_names,
    const std::vector<std::string> &limit_avoid_joint_names_param,
    const std::vector<double> &limit_avoid_lower_param,
    const std::vector<double> &limit_avoid_upper_param);

struct TiltJointIndices
{
  int left_hip_roll{-1};
  int right_hip_roll{-1};
  int left_ankle_roll{-1};
  int right_ankle_roll{-1};
  int left_hip_pitch{-1};
  int right_hip_pitch{-1};
  int left_ankle_pitch{-1};
  int right_ankle_pitch{-1};
  int left_knee{-1};
  int right_knee{-1};
  int torso{-1};

  bool has_roll_pair() const;
  bool has_pitch_pair() const;
  bool ready() const;
};

TiltJointIndices build_tilt_joint_indices(const std::vector<std::string> &joint_names);

}  // namespace rb_controller::internal
