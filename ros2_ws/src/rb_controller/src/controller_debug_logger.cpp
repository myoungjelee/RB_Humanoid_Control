#include "controller_debug_logger.hpp"

#include <array>
#include <cstdio>

namespace rb_controller::internal
{

std::string format_control_active_sync(const ControlActiveSnapshot &snapshot)
{
  std::array<char, 512> buffer{};
  std::snprintf(
      buffer.data(), buffer.size(),
      "[SYNC] CONTROL_ACTIVE joint_count=%zu imu_raw_r=%.3f imu_raw_p=%.3f tilt_r=%.3f tilt_p=%.3f imu_frame_mode=%s",
      snapshot.joint_count,
      snapshot.imu_raw_roll_rad,
      snapshot.imu_raw_pitch_rad,
      snapshot.tilt_roll_rad,
      snapshot.tilt_pitch_rad,
      snapshot.imu_frame_mode.c_str());
  return std::string(buffer.data());
}

std::string format_loop_stats(const LoopStatsSnapshot &snapshot)
{
  std::array<char, 4096> buffer{};
  std::snprintf(
      buffer.data(), buffer.size(),
      "loop_stats dt_mean=%.6f dt_max=%.6f dt_p95=%.6f miss_window=%zu miss_total=%zu samples=%zu "
      "imu_raw_r=%.3f imu_raw_p=%.3f imu_bias_r=%.3f imu_bias_p=%.3f tilt_r=%.3f tilt_p=%.3f "
      "stand_scale=%.2f top_err=%s pitch_rate=%.3f u_pitch_raw=%.3f u_pitch=%.3f u_pitch_p=%.3f u_pitch_d=%.3f "
      "alloc_p(h/a/k/t)=%.3f/%.3f/%.3f/%.3f torso_ref=%.3f torso_meas=%.3f hip_pitch_ref_avg=%.3f hip_pitch_meas_avg=%.3f "
      "knee_ref_avg=%.3f knee_meas_avg=%.3f ankle_pitch_ref_avg=%.3f ankle_pitch_meas_avg=%.3f "
      "pitch_bias_hip=%.3f pitch_bias_knee=%.3f pitch_bias_ankle=%.3f pitch_bias_torso=%.3f "
      "ankle_pd_avg=%.3f ankle_lim_avg=%.3f ankle_pre_avg=%.3f ankle_cmd_avg=%.3f ankle_sat=%zu",
      snapshot.dt_mean, snapshot.dt_max, snapshot.dt_p95,
      snapshot.miss_window, snapshot.miss_total, snapshot.samples,
      snapshot.imu_raw_roll_rad, snapshot.imu_raw_pitch_rad,
      snapshot.imu_bias_roll_rad, snapshot.imu_bias_pitch_rad,
      snapshot.tilt_roll_rad, snapshot.tilt_pitch_rad,
      snapshot.stand_scale, snapshot.top_error.c_str(),
      snapshot.tilt_debug.pitch_rate_rad_s,
      snapshot.tilt_debug.u_pitch_raw, snapshot.tilt_debug.u_pitch_clamped,
      snapshot.tilt_debug.u_pitch_p_term, snapshot.tilt_debug.u_pitch_d_term,
      snapshot.tilt_debug.pitch_alloc_hip, snapshot.tilt_debug.pitch_alloc_ankle,
      snapshot.tilt_debug.pitch_alloc_knee, snapshot.tilt_debug.pitch_alloc_torso,
      snapshot.pitch_chain.torso_ref, snapshot.pitch_chain.torso_meas,
      snapshot.pitch_chain.hip_ref_avg, snapshot.pitch_chain.hip_meas_avg,
      snapshot.pitch_chain.knee_ref_avg, snapshot.pitch_chain.knee_meas_avg,
      snapshot.pitch_chain.ankle_ref_avg, snapshot.pitch_chain.ankle_meas_avg,
      snapshot.pitch_chain.bias_hip, snapshot.pitch_chain.bias_knee,
      snapshot.pitch_chain.bias_ankle, snapshot.pitch_chain.bias_torso,
      snapshot.ankle_effort.pd_avg, snapshot.ankle_effort.limit_avg,
      snapshot.ankle_effort.pre_clamp_avg, snapshot.ankle_effort.cmd_avg,
      snapshot.ankle_effort.sat_count);
  return std::string(buffer.data());
}

}  // namespace rb_controller::internal
