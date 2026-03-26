#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include "controller_types.hpp"

namespace rb_controller::internal
{

struct EstimatedStateCache
{
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;
  bool received{false};
  double imu_raw_roll_rad{quiet_nan()};
  double imu_raw_pitch_rad{quiet_nan()};
  double tilt_roll_rad{quiet_nan()};
  double tilt_pitch_rad{quiet_nan()};
  double roll_rate_rad_s{quiet_nan()};
  double pitch_rate_rad_s{quiet_nan()};
  std::string imu_frame_mode{"identity"};
};

struct ControllerRuntimeState
{
  std::vector<double> dt_samples_sec;
  std::size_t miss_count_total{0U};
  std::size_t miss_count_window{0U};
  std::chrono::steady_clock::time_point steady_prev_tick{
      std::chrono::steady_clock::time_point::min()};
  std::chrono::steady_clock::time_point steady_last_log{
      std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point excitation_start_steady{};
  bool excitation_started{false};
  bool control_active_logged{false};
  TiltDebugSnapshot tilt_debug{};
  PitchChainDebugSnapshot pitch_chain{};
  AnkleEffortDebugSnapshot ankle_effort{};
};

}  // namespace rb_controller::internal
