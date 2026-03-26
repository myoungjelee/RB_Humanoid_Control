#pragma once

#include <cstddef>
#include <limits>
#include <string>

namespace rb_controller::internal
{

/**
 * @brief loop debug 기본값으로 쓰는 quiet NaN을 반환한다.
 */
inline double quiet_nan()
{
  return std::numeric_limits<double>::quiet_NaN();
}

/**
 * @brief tilt feedback 한 주기 계산 결과를 담는 공용 타입.
 */
struct TiltFeedbackCommand
{
  bool valid{false};
  double u_roll{0.0};
  double u_pitch{0.0};
  double u_pitch_raw{quiet_nan()};
  double u_pitch_p{quiet_nan()};
  double u_pitch_d{quiet_nan()};
  double roll_w_hip{0.0};
  double roll_w_ankle{0.0};
  double roll_w_torso{0.0};
  double pitch_w_hip{0.0};
  double pitch_w_ankle{0.0};
  double pitch_w_knee{0.0};
  double pitch_w_torso{0.0};
};

/**
 * @brief tilt 보정 루프 자체를 설명하는 디버그 스냅샷.
 */
struct TiltDebugSnapshot
{
  double pitch_input_rad{quiet_nan()};
  double pitch_rate_rad_s{quiet_nan()};
  double u_pitch_p_term{quiet_nan()};
  double u_pitch_d_term{quiet_nan()};
  double u_pitch_raw{quiet_nan()};
  double u_pitch_clamped{quiet_nan()};
  double pitch_alloc_hip{quiet_nan()};
  double pitch_alloc_ankle{quiet_nan()};
  double pitch_alloc_knee{quiet_nan()};
  double pitch_alloc_torso{quiet_nan()};
};

/**
 * @brief pitch chain(qref/meas/bias) 상태를 한 줄 로그로 보기 위한 스냅샷.
 */
struct PitchChainDebugSnapshot
{
  double torso_ref{quiet_nan()};
  double torso_meas{quiet_nan()};
  double hip_ref_avg{quiet_nan()};
  double hip_meas_avg{quiet_nan()};
  double knee_ref_avg{quiet_nan()};
  double knee_meas_avg{quiet_nan()};
  double ankle_ref_avg{quiet_nan()};
  double ankle_meas_avg{quiet_nan()};
  double bias_hip{quiet_nan()};
  double bias_knee{quiet_nan()};
  double bias_ankle{quiet_nan()};
  double bias_torso{quiet_nan()};
};

/**
 * @brief 발목 effort 관련 saturation 관측용 스냅샷.
 */
struct AnkleEffortDebugSnapshot
{
  double pd_avg{quiet_nan()};
  double limit_avg{quiet_nan()};
  double pre_clamp_avg{quiet_nan()};
  double cmd_avg{quiet_nan()};
  std::size_t sat_count{0U};
};

/**
 * @brief 첫 유효 control tick sync marker 출력을 위한 스냅샷.
 */
struct ControlActiveSnapshot
{
  std::size_t joint_count{0U};
  double imu_raw_roll_rad{quiet_nan()};
  double imu_raw_pitch_rad{quiet_nan()};
  double tilt_roll_rad{quiet_nan()};
  double tilt_pitch_rad{quiet_nan()};
  std::string imu_frame_mode{"identity"};
};

/**
 * @brief loop_stats 한 줄에 필요한 모든 값을 모아놓은 스냅샷.
 */
struct LoopStatsSnapshot
{
  double dt_mean{0.0};
  double dt_max{0.0};
  double dt_p95{0.0};
  std::size_t miss_window{0U};
  std::size_t miss_total{0U};
  std::size_t samples{0U};
  double imu_raw_roll_rad{quiet_nan()};
  double imu_raw_pitch_rad{quiet_nan()};
  double tilt_roll_rad{quiet_nan()};
  double tilt_pitch_rad{quiet_nan()};
  double stand_scale{1.0};
  std::string top_error{"-"};
  TiltDebugSnapshot tilt_debug{};
  PitchChainDebugSnapshot pitch_chain{};
  AnkleEffortDebugSnapshot ankle_effort{};
};

}  // namespace rb_controller::internal
