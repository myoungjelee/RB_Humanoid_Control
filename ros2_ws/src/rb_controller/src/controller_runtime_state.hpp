#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include "controller_types.hpp"

namespace rb_controller::internal
{

// controller가 매 tick마다 다시 계산하지 않도록, estimated_state에서 자주 쓰는 값만 캐시해 둔다.
struct EstimatedStateCache
{
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;
  bool received{false};
  bool joint_state_valid{false};
  bool imu_valid{false};
  double imu_raw_roll_rad{quiet_nan()};
  double imu_raw_pitch_rad{quiet_nan()};
  double tilt_roll_rad{quiet_nan()};
  double tilt_pitch_rad{quiet_nan()};
  double roll_rate_rad_s{quiet_nan()};
  double pitch_rate_rad_s{quiet_nan()};
};

// 제어 루프 통계와 debug snapshot을 누적해 loop_stats / sync 로그에 재사용하는 런타임 상태다.
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
