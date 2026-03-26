#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <numeric>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rb_controller/msg/estimated_state.hpp"
#include "controller_debug_logger.hpp"
#include "controller_types.hpp"
#include "controller_stand_utils.hpp"
#include "controller_runtime_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace rbci = rb_controller::internal;

// 200Hz 제어 루프 타이밍을 측정하고, 테스트/스탠드 명령(/rb/command_raw)을 퍼블리시하는 노드
class RbControllerNode : public rclcpp::Node
{
public:
  /**
   * @brief rb_controller 노드를 생성하고 ROS2 통신/타이머를 초기화한다.
   *
   * 파라미터를 선언하고, /rb/estimated_state 구독 + /rb/command_raw 퍼블리시 +
   * wall timer(기본 200Hz)를 연결한다.
   */
  RbControllerNode()
      : Node("rb_controller")
  {
    // 제어 주기/로그/토픽을 런타임 파라미터로 받는다.
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 200.0);
    log_interval_sec_ = this->declare_parameter<double>("log_interval_sec", 5.0);
    miss_ratio_threshold_ =
        this->declare_parameter<double>("miss_ratio_threshold", 1.2);
    input_topic_ =
        this->declare_parameter<std::string>("input_estimated_state_topic", "/rb/estimated_state");
    output_topic_ =
        this->declare_parameter<std::string>("output_command_raw_topic", "/rb/command_raw");
    joint_names_ =
        this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    signal_mode_ = this->declare_parameter<std::string>("signal_mode", "zero");
    target_joint_ = this->declare_parameter<std::string>("target_joint", "torso_joint");
    effort_amplitude_ = this->declare_parameter<double>("effort_amplitude", 0.0);
    effort_frequency_hz_ = this->declare_parameter<double>("effort_frequency_hz", 0.5);
    step_start_sec_ = this->declare_parameter<double>("step_start_sec", 1.0);
    stand_kp_ = this->declare_parameter<double>("stand_kp", 30.0);
    stand_kd_ = this->declare_parameter<double>("stand_kd", 2.0);
    stand_effort_abs_max_ = this->declare_parameter<double>("stand_effort_abs_max", 6.0);
    stand_hold_current_on_start_ = this->declare_parameter<bool>("stand_hold_current_on_start", true);
    // 그룹별 게인 스케일: base stand_kp/kd에 곱해 관절군별로 반응을 다르게 만든다.
    stand_kp_scale_hip_ = this->declare_parameter<double>("stand_kp_scale_hip", 1.0);
    stand_kd_scale_hip_ = this->declare_parameter<double>("stand_kd_scale_hip", 1.0);
    stand_kp_scale_knee_ = this->declare_parameter<double>("stand_kp_scale_knee", 1.0);
    stand_kd_scale_knee_ = this->declare_parameter<double>("stand_kd_scale_knee", 1.0);
    stand_kp_scale_ankle_ = this->declare_parameter<double>("stand_kp_scale_ankle", 1.0);
    stand_kd_scale_ankle_ = this->declare_parameter<double>("stand_kd_scale_ankle", 1.0);
    stand_kp_scale_torso_ = this->declare_parameter<double>("stand_kp_scale_torso", 1.0);
    stand_kd_scale_torso_ = this->declare_parameter<double>("stand_kd_scale_torso", 1.0);
    stand_kp_scale_other_ = this->declare_parameter<double>("stand_kp_scale_other", 1.0);
    stand_kd_scale_other_ = this->declare_parameter<double>("stand_kd_scale_other", 1.0);
    stand_q_ref_param_ =
        this->declare_parameter<std::vector<double>>("stand_q_ref", std::vector<double>{});
    stand_q_ref_trim_param_ =
        this->declare_parameter<std::vector<double>>("stand_q_ref_trim", std::vector<double>{});
    // 비어 있으면 모든 조인트를 제어한다. 값이 있으면 지정한 조인트만 stand_pd를 적용한다.
    stand_control_joints_param_ =
        this->declare_parameter<std::vector<std::string>>("stand_control_joints", std::vector<std::string>{});
    // 오차 클램프: q_ref와 현재 각도 차이가 큰 구간에서 과한 복귀토크를 억제한다.
    stand_pos_error_abs_max_ = this->declare_parameter<double>("stand_pos_error_abs_max", 0.35);
    // tilt cut: 과도 기울기에서는 stand 출력 스케일을 낮춰 관절 한계 충돌을 완화한다.
    stand_tilt_cut_enable_ = this->declare_parameter<bool>("stand_tilt_cut_enable", true);
    stand_tilt_cut_rad_ = this->declare_parameter<double>("stand_tilt_cut_rad", 0.45);
    stand_tilt_recover_rad_ = this->declare_parameter<double>("stand_tilt_recover_rad", 0.35);
    stand_tilt_cut_output_scale_ = this->declare_parameter<double>("stand_tilt_cut_output_scale", 0.0);
    // 관측 로그: loop_stats에 stand 오차 상위 N개 조인트를 함께 출력한다.
    const auto stand_debug_top_error_count_param =
        this->declare_parameter<int64_t>("stand_debug_top_error_count", 3);
    stand_debug_top_error_count_ = static_cast<std::size_t>(std::max<int64_t>(0, stand_debug_top_error_count_param));
    // soft-limit 회피: joint limit 근접 시 한계 방향으로 더 밀지 않도록 보정한다.
    limit_avoid_enable_ = this->declare_parameter<bool>("limit_avoid_enable", false);
    limit_avoid_margin_rad_ = this->declare_parameter<double>("limit_avoid_margin_rad", 0.05);
    limit_avoid_kp_ = this->declare_parameter<double>("limit_avoid_kp", 8.0);
    limit_avoid_kd_ = this->declare_parameter<double>("limit_avoid_kd", 1.0);
    limit_avoid_joint_names_param_ =
        this->declare_parameter<std::vector<std::string>>("limit_avoid_joint_names", std::vector<std::string>{});
    limit_avoid_lower_param_ =
        this->declare_parameter<std::vector<double>>("limit_avoid_lower", std::vector<double>{});
    limit_avoid_upper_param_ =
        this->declare_parameter<std::vector<double>>("limit_avoid_upper", std::vector<double>{});
    enable_tilt_feedback_ = this->declare_parameter<bool>("enable_tilt_feedback", true);
    tilt_kp_roll_ = this->declare_parameter<double>("tilt_kp_roll", 10.0);
    tilt_kd_roll_ = this->declare_parameter<double>("tilt_kd_roll", 2.0);
    tilt_kp_pitch_ = this->declare_parameter<double>("tilt_kp_pitch", 12.0);
    tilt_kd_pitch_ = this->declare_parameter<double>("tilt_kd_pitch", 2.5);
    tilt_deadband_rad_ = this->declare_parameter<double>("tilt_deadband_rad", 0.03);
    tilt_apply_mode_ = this->declare_parameter<std::string>("tilt_apply_mode", "effort");
    tilt_effort_abs_max_ = this->declare_parameter<double>("tilt_effort_abs_max", 1.8);
    tilt_qref_bias_abs_max_ = this->declare_parameter<double>("tilt_qref_bias_abs_max", 0.20);
    tilt_roll_sign_ = this->declare_parameter<double>("tilt_roll_sign", 1.0);
    tilt_pitch_sign_ = this->declare_parameter<double>("tilt_pitch_sign", 1.0);
    // tilt 보정 분배 가중치: 축별로 어느 관절군이 얼마나 보정을 담당할지 결정한다.
    tilt_weight_roll_hip_ = this->declare_parameter<double>("tilt_weight_roll_hip", 0.7);
    tilt_weight_roll_ankle_ = this->declare_parameter<double>("tilt_weight_roll_ankle", 0.3);
    tilt_weight_roll_torso_ = this->declare_parameter<double>("tilt_weight_roll_torso", 0.0);
    tilt_weight_pitch_hip_ = this->declare_parameter<double>("tilt_weight_pitch_hip", 0.6);
    tilt_weight_pitch_ankle_ = this->declare_parameter<double>("tilt_weight_pitch_ankle", 0.3);
    tilt_weight_pitch_knee_ = this->declare_parameter<double>("tilt_weight_pitch_knee", 0.1);
    tilt_weight_pitch_torso_ = this->declare_parameter<double>("tilt_weight_pitch_torso", 0.0);

    if (control_rate_hz_ <= 0.0)
    {
      RCLCPP_WARN(this->get_logger(), "control_rate_hz<=0. forcing 200.0Hz");
      control_rate_hz_ = 200.0;
    }
    if (log_interval_sec_ <= 0.0)
    {
      log_interval_sec_ = 5.0;
    }
    if (miss_ratio_threshold_ < 1.0)
    {
      miss_ratio_threshold_ = 1.2;
    }
    if (effort_frequency_hz_ < 0.0)
    {
      effort_frequency_hz_ = 0.5;
    }
    if (step_start_sec_ < 0.0)
    {
      step_start_sec_ = 1.0;
    }
    if (stand_kp_ < 0.0)
    {
      stand_kp_ = 0.0;
    }
    if (stand_kd_ < 0.0)
    {
      stand_kd_ = 0.0;
    }
    if (stand_effort_abs_max_ < 0.0)
    {
      stand_effort_abs_max_ = std::abs(stand_effort_abs_max_);
    }
    stand_kp_scale_hip_ = std::max(0.0, stand_kp_scale_hip_);
    stand_kd_scale_hip_ = std::max(0.0, stand_kd_scale_hip_);
    stand_kp_scale_knee_ = std::max(0.0, stand_kp_scale_knee_);
    stand_kd_scale_knee_ = std::max(0.0, stand_kd_scale_knee_);
    stand_kp_scale_ankle_ = std::max(0.0, stand_kp_scale_ankle_);
    stand_kd_scale_ankle_ = std::max(0.0, stand_kd_scale_ankle_);
    stand_kp_scale_torso_ = std::max(0.0, stand_kp_scale_torso_);
    stand_kd_scale_torso_ = std::max(0.0, stand_kd_scale_torso_);
    stand_kp_scale_other_ = std::max(0.0, stand_kp_scale_other_);
    stand_kd_scale_other_ = std::max(0.0, stand_kd_scale_other_);
    if (stand_pos_error_abs_max_ < 0.0)
    {
      stand_pos_error_abs_max_ = 0.0;
    }
    if (stand_tilt_cut_rad_ < 0.0)
    {
      stand_tilt_cut_rad_ = 0.0;
    }
    if (stand_tilt_recover_rad_ < 0.0)
    {
      stand_tilt_recover_rad_ = 0.0;
    }
    if (stand_tilt_recover_rad_ > stand_tilt_cut_rad_)
    {
      stand_tilt_recover_rad_ = stand_tilt_cut_rad_;
    }
    stand_tilt_cut_output_scale_ = std::clamp(stand_tilt_cut_output_scale_, 0.0, 1.0);
    if (limit_avoid_margin_rad_ < 0.0)
    {
      limit_avoid_margin_rad_ = 0.0;
    }
    if (limit_avoid_kp_ < 0.0)
    {
      limit_avoid_kp_ = 0.0;
    }
    if (limit_avoid_kd_ < 0.0)
    {
      limit_avoid_kd_ = 0.0;
    }
    if (tilt_kp_roll_ < 0.0)
    {
      tilt_kp_roll_ = 0.0;
    }
    if (tilt_kd_roll_ < 0.0)
    {
      tilt_kd_roll_ = 0.0;
    }
    if (tilt_kp_pitch_ < 0.0)
    {
      tilt_kp_pitch_ = 0.0;
    }
    if (tilt_kd_pitch_ < 0.0)
    {
      tilt_kd_pitch_ = 0.0;
    }
    if (tilt_deadband_rad_ < 0.0)
    {
      tilt_deadband_rad_ = 0.0;
    }
    if (tilt_effort_abs_max_ < 0.0)
    {
      tilt_effort_abs_max_ = std::abs(tilt_effort_abs_max_);
    }
    if (tilt_qref_bias_abs_max_ < 0.0)
    {
      tilt_qref_bias_abs_max_ = std::abs(tilt_qref_bias_abs_max_);
    }
    if (tilt_apply_mode_ != "effort" && tilt_apply_mode_ != "qref_bias")
    {
      RCLCPP_WARN(
          this->get_logger(),
          "unknown tilt_apply_mode '%s'. fallback to effort",
          tilt_apply_mode_.c_str());
      tilt_apply_mode_ = "effort";
    }
    tilt_weight_roll_hip_ = std::max(0.0, tilt_weight_roll_hip_);
    tilt_weight_roll_ankle_ = std::max(0.0, tilt_weight_roll_ankle_);
    tilt_weight_roll_torso_ = std::max(0.0, tilt_weight_roll_torso_);
    tilt_weight_pitch_hip_ = std::max(0.0, tilt_weight_pitch_hip_);
    tilt_weight_pitch_ankle_ = std::max(0.0, tilt_weight_pitch_ankle_);
    tilt_weight_pitch_knee_ = std::max(0.0, tilt_weight_pitch_knee_);
    tilt_weight_pitch_torso_ = std::max(0.0, tilt_weight_pitch_torso_);
    // 200Hz 기준 expected dt = 0.005s
    expected_dt_sec_ = 1.0 / control_rate_hz_;

    // 제어 명령 publish, estimated_state subscribe 배선
    command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output_topic_, 10);
    estimated_state_sub_ = this->create_subscription<rb_controller::msg::EstimatedState>(
        input_topic_, 10,
        std::bind(&RbControllerNode::on_estimated_state, this, std::placeholders::_1));

    // wall-time 기반 고정 주기 타이머(best-effort)
    using namespace std::chrono;
    const auto timer_period = duration_cast<nanoseconds>(duration<double>(expected_dt_sec_));
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&RbControllerNode::on_control_tick, this));
    // 런타임 튜닝을 위해 ros2 param set 변경을 실제 제어 변수에 반영한다.
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&RbControllerNode::on_parameters_changed, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "rb_controller started: rate=%.1fHz expected_dt=%.6fs estimated_state=%s output=%s joint_names(init)=%zu signal_mode=%s target_joint=%s amp=%.3f freq=%.3f stand_kp=%.3f stand_kd=%.3f stand_limit=%.3f",
        control_rate_hz_, expected_dt_sec_, input_topic_.c_str(), output_topic_.c_str(), joint_names_.size(),
        signal_mode_.c_str(), target_joint_.c_str(), effort_amplitude_, effort_frequency_hz_,
        stand_kp_, stand_kd_, stand_effort_abs_max_);
    RCLCPP_INFO(
        this->get_logger(),
        "stand_control_joints configured: count=%zu (empty means all joints)",
        stand_control_joints_param_.size());
    RCLCPP_INFO(
        this->get_logger(),
        "stand_guard error_max=%.3f tilt_cut=%s cut=%.3f recover=%.3f scale=%.2f top_err_n=%zu",
        stand_pos_error_abs_max_,
        stand_tilt_cut_enable_ ? "on" : "off",
        stand_tilt_cut_rad_, stand_tilt_recover_rad_, stand_tilt_cut_output_scale_,
        stand_debug_top_error_count_);
    RCLCPP_INFO(
        this->get_logger(),
        "limit_avoid=%s margin=%.4f kp=%.3f kd=%.3f table=%zu",
        limit_avoid_enable_ ? "on" : "off",
        limit_avoid_margin_rad_, limit_avoid_kp_, limit_avoid_kd_, limit_avoid_joint_names_param_.size());
    RCLCPP_INFO(
        this->get_logger(),
        "stand_group_scales kp(hip/knee/ankle/torso/other)=%.2f/%.2f/%.2f/%.2f/%.2f kd=%.2f/%.2f/%.2f/%.2f/%.2f",
        stand_kp_scale_hip_, stand_kp_scale_knee_, stand_kp_scale_ankle_, stand_kp_scale_torso_, stand_kp_scale_other_,
        stand_kd_scale_hip_, stand_kd_scale_knee_, stand_kd_scale_ankle_, stand_kd_scale_torso_, stand_kd_scale_other_);
    RCLCPP_INFO(
        this->get_logger(),
        "tilt_feedback=%s mode=%s g1_frame_comp=on kp_roll=%.3f kd_roll=%.3f kp_pitch=%.3f kd_pitch=%.3f deadband=%.4f max_effort=%.3f qref_bias_max=%.3f sign(r/p)=%.1f/%.1f weights roll(h/a/t)=%.2f/%.2f/%.2f pitch(h/a/k/t)=%.2f/%.2f/%.2f/%.2f",
        enable_tilt_feedback_ ? "on" : "off", tilt_apply_mode_.c_str(),
        tilt_kp_roll_, tilt_kd_roll_, tilt_kp_pitch_, tilt_kd_pitch_,
        tilt_deadband_rad_, tilt_effort_abs_max_, tilt_qref_bias_abs_max_, tilt_roll_sign_, tilt_pitch_sign_,
        tilt_weight_roll_hip_, tilt_weight_roll_ankle_, tilt_weight_roll_torso_,
        tilt_weight_pitch_hip_, tilt_weight_pitch_ankle_, tilt_weight_pitch_knee_, tilt_weight_pitch_torso_);
  }

private:
  /**
   * @brief /rb/estimated_state에서 조인트/기울기 상태를 캐시한다.
   * @param msg 수신된 EstimatedState 메시지
   */
  void on_estimated_state(const rb_controller::msg::EstimatedState::SharedPtr msg)
  {
    if (!msg || msg->joint_names.empty())
    {
      return;
    }

    if (joint_names_ != msg->joint_names)
    {
      joint_names_ = msg->joint_names;
      stand_reference_ready_ = false;
      stand_control_mask_ready_ = false;
      stand_gain_map_ready_ = false;
      tilt_joint_index_ready_ = false;
      tilt_joint_index_warned_ = false;
      limit_avoid_map_ready_ = false;
      limit_avoid_invalid_warned_ = false;
      RCLCPP_INFO(
          this->get_logger(),
          "joint_names updated from /rb/estimated_state: count=%zu", joint_names_.size());
    }

    estimated_state_.joint_positions.assign(joint_names_.size(), 0.0);
    const std::size_t pos_count = std::min(joint_names_.size(), msg->joint_positions.size());
    for (std::size_t i = 0; i < pos_count; ++i)
    {
      estimated_state_.joint_positions[i] = msg->joint_positions[i];
    }

    estimated_state_.joint_velocities.assign(joint_names_.size(), 0.0);
    const std::size_t vel_count = std::min(joint_names_.size(), msg->joint_velocities.size());
    for (std::size_t i = 0; i < vel_count; ++i)
    {
      estimated_state_.joint_velocities[i] = msg->joint_velocities[i];
    }

    estimated_state_.joint_state_valid = msg->joint_state_valid;
    estimated_state_.imu_valid = msg->imu_valid;
    estimated_state_.imu_raw_roll_rad = msg->raw_roll_rad;
    estimated_state_.imu_raw_pitch_rad = msg->raw_pitch_rad;
    estimated_state_.tilt_roll_rad = msg->tilt_roll_rad;
    estimated_state_.tilt_pitch_rad = msg->tilt_pitch_rad;
    estimated_state_.roll_rate_rad_s = msg->roll_rate_rad_s;
    estimated_state_.pitch_rate_rad_s = msg->pitch_rate_rad_s;
    estimated_state_.received = true;
  }

  /**
   * @brief 고정 주기 제어 콜백.
   *
   * 루프 dt를 측정해 통계를 누적하고, 현재 signal_mode에 맞는 명령을 /rb/command_raw로 퍼블리시한다.
   * 주기적으로 timing 통계 로그를 출력한다.
   */
  void on_control_tick()
  {
    const auto now_steady = std::chrono::steady_clock::now();

    // 루프 간격(dt) 측정 및 miss 카운트
    if (runtime_state_.steady_prev_tick != std::chrono::steady_clock::time_point::min())
    {
      const double dt_sec = std::chrono::duration<double>(now_steady - runtime_state_.steady_prev_tick).count();
      runtime_state_.dt_samples_sec.push_back(dt_sec);
      if (dt_sec > expected_dt_sec_ * miss_ratio_threshold_)
      {
        ++runtime_state_.miss_count_total;
        ++runtime_state_.miss_count_window;
      }
    }
    runtime_state_.steady_prev_tick = now_steady;

    // Sim-to-Real 트랙은 effort-only 명령 경로를 기준으로 한다.
    // position/velocity 배열을 0으로 채우면 articulation이 다중 인터페이스 명령을 동시에
    // 받게 되므로, effort 외 필드는 비워둔다.
    sensor_msgs::msg::JointState cmd_msg;
    cmd_msg.header.stamp = this->now();
    cmd_msg.name = joint_names_;

    const std::size_t joint_count = joint_names_.size();
    cmd_msg.effort.assign(joint_count, 0.0);
    last_stand_output_scale_ = 1.0;
    // 관측용 스냅샷은 매 주기 초기화 후, 실제로 적용된 값만 채운다.
    runtime_state_.tilt_debug = rbci::TiltDebugSnapshot{};
    reset_pitch_chain_debug_snapshot();
    reset_ankle_effort_debug_snapshot();

    if (!runtime_state_.control_active_logged &&
        estimated_state_.received &&
        estimated_state_.joint_state_valid &&
        estimated_state_.imu_valid)
    {
      runtime_state_.control_active_logged = true;
      rbci::ControlActiveSnapshot snapshot;
      snapshot.joint_count = joint_names_.size();
      snapshot.imu_raw_roll_rad = estimated_state_.imu_raw_roll_rad;
      snapshot.imu_raw_pitch_rad = estimated_state_.imu_raw_pitch_rad;
      snapshot.tilt_roll_rad = estimated_state_.tilt_roll_rad;
      snapshot.tilt_pitch_rad = estimated_state_.tilt_pitch_rad;
      RCLCPP_INFO(this->get_logger(), "%s", rbci::format_control_active_sync(snapshot).c_str());
    }

    apply_excitation(cmd_msg, now_steady);

    if (joint_count == 0U)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "joint_names is empty. Waiting for /rb/estimated_state before meaningful command publication.");
    }

    command_pub_->publish(cmd_msg);

    // 설정한 주기(log_interval_sec)마다 타이밍 통계를 출력
    const double log_elapsed = std::chrono::duration<double>(now_steady - runtime_state_.steady_last_log).count();
    if (log_elapsed >= log_interval_sec_)
    {
      log_timing_stats();
      runtime_state_.steady_last_log = now_steady;
      runtime_state_.dt_samples_sec.clear();
      runtime_state_.miss_count_window = 0;
    }
  }

  /**
   * @brief 누적된 dt 샘플의 통계를 계산해 로그로 출력한다.
   *
   * 출력 항목: dt_mean, dt_max, dt_p95, miss_window, miss_total, samples
   */
  std::string summarize_top_stand_error_joints(std::size_t top_n) const
  {
    if (top_n == 0U || signal_mode_ != "stand_pd" ||
        !estimated_state_.received || !stand_reference_ready_ || !stand_control_mask_ready_)
    {
      return "-";
    }

    const std::size_t usable_count = std::min(
        std::min(joint_names_.size(), estimated_state_.joint_positions.size()),
        std::min(stand_q_ref_active_.size(), stand_control_mask_.size()));
    if (usable_count == 0U)
    {
      return "-";
    }

    std::vector<std::pair<double, std::size_t>> abs_error_and_index;
    abs_error_and_index.reserve(usable_count);
    for (std::size_t i = 0; i < usable_count; ++i)
    {
      if (stand_control_mask_[i] == 0U)
      {
        continue;
      }
      const double error = stand_q_ref_active_[i] - estimated_state_.joint_positions[i];
      abs_error_and_index.emplace_back(std::abs(error), i);
    }
    if (abs_error_and_index.empty())
    {
      return "-";
    }

    std::sort(
        abs_error_and_index.begin(), abs_error_and_index.end(),
        [](const auto & a, const auto & b)
        { return a.first > b.first; });

    const std::size_t take_n = std::min(top_n, abs_error_and_index.size());
    std::ostringstream oss;
    for (std::size_t rank = 0; rank < take_n; ++rank)
    {
      const std::size_t i = abs_error_and_index[rank].second;
      const double signed_error = stand_q_ref_active_[i] - estimated_state_.joint_positions[i];
      if (rank > 0U)
      {
        oss << ",";
      }
      oss << joint_names_[i] << ":" << signed_error;
    }
    return oss.str();
  }

  void log_timing_stats()
  {
    // 수집된 dt 샘플이 없으면 통계 생략
    if (runtime_state_.dt_samples_sec.empty())
    {
      RCLCPP_WARN(this->get_logger(), "timing window has no samples yet");
      return;
    }

    // dt_mean / dt_max / dt_p95 / miss 카운트 출력
    const double sum = std::accumulate(runtime_state_.dt_samples_sec.begin(), runtime_state_.dt_samples_sec.end(), 0.0);
    const double mean = sum / static_cast<double>(runtime_state_.dt_samples_sec.size());
    const double max_dt = *std::max_element(runtime_state_.dt_samples_sec.begin(), runtime_state_.dt_samples_sec.end());
    const double p95 = percentile(runtime_state_.dt_samples_sec, 0.95);
    const std::string top_error = summarize_top_stand_error_joints(stand_debug_top_error_count_);

    rbci::LoopStatsSnapshot snapshot;
    snapshot.dt_mean = mean;
    snapshot.dt_max = max_dt;
    snapshot.dt_p95 = p95;
    snapshot.miss_window = runtime_state_.miss_count_window;
    snapshot.miss_total = runtime_state_.miss_count_total;
    snapshot.samples = runtime_state_.dt_samples_sec.size();
    snapshot.imu_raw_roll_rad = estimated_state_.imu_raw_roll_rad;
    snapshot.imu_raw_pitch_rad = estimated_state_.imu_raw_pitch_rad;
    snapshot.tilt_roll_rad = estimated_state_.tilt_roll_rad;
    snapshot.tilt_pitch_rad = estimated_state_.tilt_pitch_rad;
    snapshot.stand_scale = last_stand_output_scale_;
    snapshot.top_error = top_error;
    snapshot.tilt_debug = runtime_state_.tilt_debug;
    snapshot.pitch_chain = runtime_state_.pitch_chain;
    snapshot.ankle_effort = runtime_state_.ankle_effort;

    RCLCPP_INFO(this->get_logger(), "%s", rbci::format_loop_stats(snapshot).c_str());
  }

  /**
   * @brief percentile 값을 계산한다.
   * @param values 입력 샘플 벡터
   * @param ratio 백분위 비율(0.0~1.0)
   * @return 계산된 백분위 값
   */
  static double percentile(const std::vector<double> &values, double ratio)
  {
    // 간단한 percentile 계산 유틸(정렬 후 인덱스 선택)
    if (values.empty())
    {
      return 0.0;
    }
    std::vector<double> sorted(values.begin(), values.end());
    std::sort(sorted.begin(), sorted.end());

    if (ratio <= 0.0)
    {
      return sorted.front();
    }
    if (ratio >= 1.0)
    {
      return sorted.back();
    }

    const std::size_t n = sorted.size();
    const auto idx = static_cast<std::size_t>(std::ceil(ratio * static_cast<double>(n)) - 1.0);
    return sorted[std::min(idx, n - 1U)];
  }

  /**
   * @brief ros2 param set 요청을 검증하고 런타임 변수에 즉시 반영한다.
   * @param params 변경 요청된 파라미터 목록
   * @return 적용 성공/실패 결과
   */
  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
      const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";

    for (const auto &param : params)
    {
      const std::string &name = param.get_name();

      if (name == "control_rate_hz" || name == "input_estimated_state_topic" || name == "output_command_raw_topic")
      {
        result.successful = false;
        result.reason = "runtime update not supported for control_rate_hz/topic parameters (restart required)";
        return result;
      }
      if (name == "signal_mode")
      {
        const std::string mode = param.as_string();
        if (mode != "zero" && mode != "sine_effort" && mode != "step_effort" && mode != "stand_pd")
        {
          result.successful = false;
          result.reason = "signal_mode must be one of zero|sine_effort|step_effort|stand_pd";
          return result;
        }
        signal_mode_ = mode;
        runtime_state_.excitation_started = false;
        continue;
      }
      if (name == "target_joint")
      {
        target_joint_ = param.as_string();
        continue;
      }
      if (name == "effort_amplitude")
      {
        effort_amplitude_ = param.as_double();
        continue;
      }
      if (name == "effort_frequency_hz")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "effort_frequency_hz must be >= 0";
          return result;
        }
        effort_frequency_hz_ = v;
        runtime_state_.excitation_started = false;
        continue;
      }
      if (name == "step_start_sec")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "step_start_sec must be >= 0";
          return result;
        }
        step_start_sec_ = v;
        runtime_state_.excitation_started = false;
        continue;
      }
      if (name == "stand_kp")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_kp must be >= 0";
          return result;
        }
        stand_kp_ = v;
        continue;
      }
      if (name == "stand_kd")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_kd must be >= 0";
          return result;
        }
        stand_kd_ = v;
        continue;
      }
      if (name == "stand_effort_abs_max")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_effort_abs_max must be >= 0";
          return result;
        }
        stand_effort_abs_max_ = v;
        continue;
      }
      if (name == "stand_kp_scale_hip")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_kp_scale_hip must be >= 0";
          return result;
        }
        stand_kp_scale_hip_ = v;
        stand_gain_map_ready_ = false;
        continue;
      }
      if (name == "stand_kd_scale_hip")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_kd_scale_hip must be >= 0";
          return result;
        }
        stand_kd_scale_hip_ = v;
        stand_gain_map_ready_ = false;
        continue;
      }
      if (name == "stand_kp_scale_knee")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_kp_scale_knee must be >= 0";
          return result;
        }
        stand_kp_scale_knee_ = v;
        stand_gain_map_ready_ = false;
        continue;
      }
      if (name == "stand_kd_scale_knee")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_kd_scale_knee must be >= 0";
          return result;
        }
        stand_kd_scale_knee_ = v;
        stand_gain_map_ready_ = false;
        continue;
      }
      if (name == "stand_kp_scale_ankle")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_kp_scale_ankle must be >= 0";
          return result;
        }
        stand_kp_scale_ankle_ = v;
        stand_gain_map_ready_ = false;
        continue;
      }
      if (name == "stand_kd_scale_ankle")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_kd_scale_ankle must be >= 0";
          return result;
        }
        stand_kd_scale_ankle_ = v;
        stand_gain_map_ready_ = false;
        continue;
      }
      if (name == "stand_kp_scale_torso")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_kp_scale_torso must be >= 0";
          return result;
        }
        stand_kp_scale_torso_ = v;
        stand_gain_map_ready_ = false;
        continue;
      }
      if (name == "stand_kd_scale_torso")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_kd_scale_torso must be >= 0";
          return result;
        }
        stand_kd_scale_torso_ = v;
        stand_gain_map_ready_ = false;
        continue;
      }
      if (name == "stand_kp_scale_other")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_kp_scale_other must be >= 0";
          return result;
        }
        stand_kp_scale_other_ = v;
        stand_gain_map_ready_ = false;
        continue;
      }
      if (name == "stand_kd_scale_other")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_kd_scale_other must be >= 0";
          return result;
        }
        stand_kd_scale_other_ = v;
        stand_gain_map_ready_ = false;
        continue;
      }
      if (name == "stand_hold_current_on_start")
      {
        stand_hold_current_on_start_ = param.as_bool();
        stand_reference_ready_ = false;
        stand_ref_param_size_warned_ = false;
        continue;
      }
      if (name == "stand_q_ref")
      {
        stand_q_ref_param_ = param.as_double_array();
        stand_reference_ready_ = false;
        stand_ref_param_size_warned_ = false;
        stand_ref_trim_param_size_warned_ = false;
        continue;
      }
      if (name == "stand_q_ref_trim")
      {
        stand_q_ref_trim_param_ = param.as_double_array();
        stand_reference_ready_ = false;
        stand_ref_param_size_warned_ = false;
        stand_ref_trim_param_size_warned_ = false;
        continue;
      }
      if (name == "stand_control_joints")
      {
        stand_control_joints_param_ = param.as_string_array();
        stand_control_mask_ready_ = false;
        stand_control_unknown_warned_ = false;
        stand_gain_map_ready_ = false;
        continue;
      }
      if (name == "stand_pos_error_abs_max")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_pos_error_abs_max must be >= 0";
          return result;
        }
        stand_pos_error_abs_max_ = v;
        continue;
      }
      if (name == "stand_tilt_cut_enable")
      {
        stand_tilt_cut_enable_ = param.as_bool();
        if (!stand_tilt_cut_enable_)
        {
          stand_tilt_cut_active_ = false;
        }
        continue;
      }
      if (name == "stand_tilt_cut_rad")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_tilt_cut_rad must be >= 0";
          return result;
        }
        stand_tilt_cut_rad_ = v;
        if (stand_tilt_recover_rad_ > stand_tilt_cut_rad_)
        {
          stand_tilt_recover_rad_ = stand_tilt_cut_rad_;
        }
        continue;
      }
      if (name == "stand_tilt_recover_rad")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "stand_tilt_recover_rad must be >= 0";
          return result;
        }
        stand_tilt_recover_rad_ = std::min(v, stand_tilt_cut_rad_);
        continue;
      }
      if (name == "stand_tilt_cut_output_scale")
      {
        const double v = param.as_double();
        if (v < 0.0 || v > 1.0)
        {
          result.successful = false;
          result.reason = "stand_tilt_cut_output_scale must be in [0,1]";
          return result;
        }
        stand_tilt_cut_output_scale_ = v;
        continue;
      }
      if (name == "stand_debug_top_error_count")
      {
        const int64_t v = param.as_int();
        if (v < 0)
        {
          result.successful = false;
          result.reason = "stand_debug_top_error_count must be >= 0";
          return result;
        }
        stand_debug_top_error_count_ = static_cast<std::size_t>(v);
        continue;
      }
      if (name == "limit_avoid_enable")
      {
        limit_avoid_enable_ = param.as_bool();
        continue;
      }
      if (name == "limit_avoid_margin_rad")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "limit_avoid_margin_rad must be >= 0";
          return result;
        }
        limit_avoid_margin_rad_ = v;
        continue;
      }
      if (name == "limit_avoid_kp")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "limit_avoid_kp must be >= 0";
          return result;
        }
        limit_avoid_kp_ = v;
        continue;
      }
      if (name == "limit_avoid_kd")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "limit_avoid_kd must be >= 0";
          return result;
        }
        limit_avoid_kd_ = v;
        continue;
      }
      if (name == "limit_avoid_joint_names")
      {
        limit_avoid_joint_names_param_ = param.as_string_array();
        limit_avoid_map_ready_ = false;
        limit_avoid_invalid_warned_ = false;
        continue;
      }
      if (name == "limit_avoid_lower")
      {
        limit_avoid_lower_param_ = param.as_double_array();
        limit_avoid_map_ready_ = false;
        limit_avoid_invalid_warned_ = false;
        continue;
      }
      if (name == "limit_avoid_upper")
      {
        limit_avoid_upper_param_ = param.as_double_array();
        limit_avoid_map_ready_ = false;
        limit_avoid_invalid_warned_ = false;
        continue;
      }
      if (name == "enable_tilt_feedback")
      {
        enable_tilt_feedback_ = param.as_bool();
        continue;
      }
      if (name == "tilt_kp_roll")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_kp_roll must be >= 0";
          return result;
        }
        tilt_kp_roll_ = v;
        continue;
      }
      if (name == "tilt_kd_roll")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_kd_roll must be >= 0";
          return result;
        }
        tilt_kd_roll_ = v;
        continue;
      }
      if (name == "tilt_kp_pitch")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_kp_pitch must be >= 0";
          return result;
        }
        tilt_kp_pitch_ = v;
        continue;
      }
      if (name == "tilt_kd_pitch")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_kd_pitch must be >= 0";
          return result;
        }
        tilt_kd_pitch_ = v;
        continue;
      }
      if (name == "tilt_deadband_rad")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_deadband_rad must be >= 0";
          return result;
        }
        tilt_deadband_rad_ = v;
        continue;
      }
      if (name == "tilt_effort_abs_max")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_effort_abs_max must be >= 0";
          return result;
        }
        tilt_effort_abs_max_ = v;
        continue;
      }
      if (name == "tilt_apply_mode")
      {
        const std::string v = param.as_string();
        if (v != "effort" && v != "qref_bias")
        {
          result.successful = false;
          result.reason = "tilt_apply_mode must be 'effort' or 'qref_bias'";
          return result;
        }
        tilt_apply_mode_ = v;
        continue;
      }
      if (name == "tilt_qref_bias_abs_max")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_qref_bias_abs_max must be >= 0";
          return result;
        }
        tilt_qref_bias_abs_max_ = v;
        continue;
      }
      if (name == "tilt_roll_sign")
      {
        tilt_roll_sign_ = param.as_double();
        continue;
      }
      if (name == "tilt_pitch_sign")
      {
        tilt_pitch_sign_ = param.as_double();
        continue;
      }
      if (name == "tilt_weight_roll_hip")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_weight_roll_hip must be >= 0";
          return result;
        }
        tilt_weight_roll_hip_ = v;
        continue;
      }
      if (name == "tilt_weight_roll_ankle")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_weight_roll_ankle must be >= 0";
          return result;
        }
        tilt_weight_roll_ankle_ = v;
        continue;
      }
      if (name == "tilt_weight_roll_torso")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_weight_roll_torso must be >= 0";
          return result;
        }
        tilt_weight_roll_torso_ = v;
        continue;
      }
      if (name == "tilt_weight_pitch_hip")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_weight_pitch_hip must be >= 0";
          return result;
        }
        tilt_weight_pitch_hip_ = v;
        continue;
      }
      if (name == "tilt_weight_pitch_ankle")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_weight_pitch_ankle must be >= 0";
          return result;
        }
        tilt_weight_pitch_ankle_ = v;
        continue;
      }
      if (name == "tilt_weight_pitch_knee")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_weight_pitch_knee must be >= 0";
          return result;
        }
        tilt_weight_pitch_knee_ = v;
        continue;
      }
      if (name == "tilt_weight_pitch_torso")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_weight_pitch_torso must be >= 0";
          return result;
        }
        tilt_weight_pitch_torso_ = v;
        continue;
      }
      if (name == "log_interval_sec")
      {
        const double v = param.as_double();
        if (v <= 0.0)
        {
          result.successful = false;
          result.reason = "log_interval_sec must be > 0";
          return result;
        }
        log_interval_sec_ = v;
        continue;
      }
      if (name == "miss_ratio_threshold")
      {
        const double v = param.as_double();
        if (v < 1.0)
        {
          result.successful = false;
          result.reason = "miss_ratio_threshold must be >= 1.0";
          return result;
        }
        miss_ratio_threshold_ = v;
        continue;
      }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "runtime parameters updated: signal_mode=%s stand_kp=%.3f stand_kd=%.3f stand_limit=%.3f hold=%s q_ref_len=%zu ctrl_joint_count=%zu stand_scales kp(h/k/a/t/o)=%.2f/%.2f/%.2f/%.2f/%.2f kd=%.2f/%.2f/%.2f/%.2f/%.2f tilt_fb=%s mode=%s g1_frame_comp=on kp(r/p)=%.3f/%.3f kd(r/p)=%.3f/%.3f deadband=%.4f max_effort=%.3f qref_bias_max=%.3f sign(r/p)=%.1f/%.1f w_roll(h/a/t)=%.2f/%.2f/%.2f w_pitch(h/a/k/t)=%.2f/%.2f/%.2f/%.2f",
        signal_mode_.c_str(), stand_kp_, stand_kd_, stand_effort_abs_max_,
        stand_hold_current_on_start_ ? "true" : "false",
        stand_q_ref_param_.size(), stand_control_joints_param_.size(),
        stand_kp_scale_hip_, stand_kp_scale_knee_, stand_kp_scale_ankle_, stand_kp_scale_torso_, stand_kp_scale_other_,
        stand_kd_scale_hip_, stand_kd_scale_knee_, stand_kd_scale_ankle_, stand_kd_scale_torso_, stand_kd_scale_other_,
        enable_tilt_feedback_ ? "on" : "off", tilt_apply_mode_.c_str(),
        tilt_kp_roll_, tilt_kp_pitch_, tilt_kd_roll_, tilt_kd_pitch_,
        tilt_deadband_rad_, tilt_effort_abs_max_, tilt_qref_bias_abs_max_, tilt_roll_sign_, tilt_pitch_sign_,
        tilt_weight_roll_hip_, tilt_weight_roll_ankle_, tilt_weight_roll_torso_,
        tilt_weight_pitch_hip_, tilt_weight_pitch_ankle_, tilt_weight_pitch_knee_, tilt_weight_pitch_torso_);
    RCLCPP_INFO(
        this->get_logger(),
        "runtime limit_avoid: enable=%s margin=%.4f kp=%.3f kd=%.3f table=%zu",
        limit_avoid_enable_ ? "on" : "off",
        limit_avoid_margin_rad_, limit_avoid_kp_, limit_avoid_kd_, limit_avoid_joint_names_param_.size());
    RCLCPP_INFO(
        this->get_logger(),
        "runtime stand_guard: err_max=%.3f tilt_cut=%s cut=%.3f recover=%.3f scale=%.2f top_err_n=%zu",
        stand_pos_error_abs_max_,
        stand_tilt_cut_enable_ ? "on" : "off",
        stand_tilt_cut_rad_, stand_tilt_recover_rad_, stand_tilt_cut_output_scale_,
        stand_debug_top_error_count_);
    return result;
  }

  /**
   * @brief signal_mode에 맞는 제어 신호를 command 메시지에 주입한다.
   * @param cmd_msg publish할 JointState 명령
   * @param now_steady 현재 steady clock 시각
   */
  void apply_excitation(
      sensor_msgs::msg::JointState &cmd_msg,
      const std::chrono::steady_clock::time_point &now_steady)
  {
    const std::size_t joint_count = cmd_msg.name.size();
    if (joint_count == 0U || signal_mode_ == "zero")
    {
      return;
    }

    if (signal_mode_ == "stand_pd")
    {
      apply_stand_pd(cmd_msg);
      return;
    }

    const auto it = std::find(cmd_msg.name.begin(), cmd_msg.name.end(), target_joint_);
    if (it == cmd_msg.name.end())
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "target_joint '%s' not found in joint_names", target_joint_.c_str());
      return;
    }
    const std::size_t idx = static_cast<std::size_t>(std::distance(cmd_msg.name.begin(), it));

    if (!runtime_state_.excitation_started)
    {
      runtime_state_.excitation_started = true;
      runtime_state_.excitation_start_steady = now_steady;
    }
    const double elapsed_sec =
        std::chrono::duration<double>(now_steady - runtime_state_.excitation_start_steady).count();

    if (signal_mode_ == "sine_effort")
    {
      constexpr double kTwoPi = 6.283185307179586;
      const double cmd = effort_amplitude_ * std::sin(kTwoPi * effort_frequency_hz_ * elapsed_sec);
      cmd_msg.effort[idx] = cmd;
      return;
    }

    if (signal_mode_ == "step_effort")
    {
      cmd_msg.effort[idx] = (elapsed_sec >= step_start_sec_) ? effort_amplitude_ : 0.0;
      return;
    }

    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "unknown signal_mode '%s' (supported: zero|sine_effort|step_effort|stand_pd)", signal_mode_.c_str());
  }

  // estimated_state의 tilt/rate를 제어 입력으로 바꿔, 각 관절 체인에 어떻게 분배할지까지 계산한다.
  bool compute_tilt_feedback_command(rbci::TiltFeedbackCommand &cmd)
  {
    if (!enable_tilt_feedback_)
    {
      return false;
    }
    if (!estimated_state_.received)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "tilt feedback waiting for /rb/estimated_state samples");
      return false;
    }
    if (!estimated_state_.imu_valid)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "tilt feedback waiting for valid IMU state in /rb/estimated_state");
      return false;
    }
    if (!ensure_tilt_joint_index_ready())
    {
      return false;
    }

    double roll = estimated_state_.tilt_roll_rad;
    double pitch = estimated_state_.tilt_pitch_rad;
    // 아주 작은 기울기는 노이즈로 보고 feedback을 넣지 않는다.
    if (std::abs(roll) < tilt_deadband_rad_)
    {
      roll = 0.0;
    }
    if (std::abs(pitch) < tilt_deadband_rad_)
    {
      pitch = 0.0;
    }

    const double u_roll_raw =
        tilt_roll_sign_ * ((tilt_kp_roll_ * (-roll)) + (tilt_kd_roll_ * (-estimated_state_.roll_rate_rad_s)));
    const double u_pitch_p = tilt_pitch_sign_ * (tilt_kp_pitch_ * (-pitch));
    const double u_pitch_d = tilt_pitch_sign_ * (tilt_kd_pitch_ * (-estimated_state_.pitch_rate_rad_s));
    const double u_pitch_raw = u_pitch_p + u_pitch_d;

    double u_roll = u_roll_raw;
    double u_pitch = u_pitch_raw;
    if (tilt_apply_mode_ == "effort")
    {
      if (tilt_effort_abs_max_ > 0.0)
      {
        u_roll = std::clamp(u_roll, -tilt_effort_abs_max_, tilt_effort_abs_max_);
        u_pitch = std::clamp(u_pitch, -tilt_effort_abs_max_, tilt_effort_abs_max_);
      }
    }
    else
    {
      if (tilt_qref_bias_abs_max_ > 0.0)
      {
        u_roll = std::clamp(u_roll, -tilt_qref_bias_abs_max_, tilt_qref_bias_abs_max_);
        u_pitch = std::clamp(u_pitch, -tilt_qref_bias_abs_max_, tilt_qref_bias_abs_max_);
      }
    }

    // 실제로는 "roll/pitch 보정량 하나"를 계산한 뒤, 관절군별 weight로 분배한다.
    const double roll_w_sum = tilt_weight_roll_hip_ + tilt_weight_roll_ankle_ + tilt_weight_roll_torso_;
    const double pitch_w_sum =
        tilt_weight_pitch_hip_ + tilt_weight_pitch_ankle_ + tilt_weight_pitch_knee_ + tilt_weight_pitch_torso_;
    const double roll_norm = (roll_w_sum > 1e-9) ? (1.0 / roll_w_sum) : 0.0;
    const double pitch_norm = (pitch_w_sum > 1e-9) ? (1.0 / pitch_w_sum) : 0.0;

    cmd.valid = true;
    cmd.u_roll = u_roll;
    cmd.u_pitch = u_pitch;
    cmd.u_pitch_raw = u_pitch_raw;
    cmd.u_pitch_p = u_pitch_p;
    cmd.u_pitch_d = u_pitch_d;
    cmd.roll_w_hip = tilt_weight_roll_hip_ * roll_norm;
    cmd.roll_w_ankle = tilt_weight_roll_ankle_ * roll_norm;
    cmd.roll_w_torso = tilt_weight_roll_torso_ * roll_norm;
    cmd.pitch_w_hip = tilt_weight_pitch_hip_ * pitch_norm;
    cmd.pitch_w_ankle = tilt_weight_pitch_ankle_ * pitch_norm;
    cmd.pitch_w_knee = tilt_weight_pitch_knee_ * pitch_norm;
    cmd.pitch_w_torso = tilt_weight_pitch_torso_ * pitch_norm;

    runtime_state_.tilt_debug.pitch_input_rad = pitch;
    runtime_state_.tilt_debug.pitch_rate_rad_s = estimated_state_.pitch_rate_rad_s;
    runtime_state_.tilt_debug.u_pitch_p_term = u_pitch_p;
    runtime_state_.tilt_debug.u_pitch_d_term = u_pitch_d;
    runtime_state_.tilt_debug.u_pitch_raw = u_pitch_raw;
    runtime_state_.tilt_debug.u_pitch_clamped = u_pitch;
    runtime_state_.tilt_debug.pitch_alloc_hip = u_pitch * cmd.pitch_w_hip;
    runtime_state_.tilt_debug.pitch_alloc_ankle = u_pitch * cmd.pitch_w_ankle;
    runtime_state_.tilt_debug.pitch_alloc_knee = u_pitch * cmd.pitch_w_knee;
    runtime_state_.tilt_debug.pitch_alloc_torso = u_pitch * cmd.pitch_w_torso;
    return true;
  }

  // effort 모드에서는 PD 결과에 tilt 보정 effort를 직접 더한다.
  void apply_tilt_feedback_effort(
      sensor_msgs::msg::JointState &cmd_msg, std::size_t usable_count, const rbci::TiltFeedbackCommand &tilt_cmd)
  {
    const auto add_effort = [&](int idx, double delta)
    {
      if (idx < 0)
      {
        return;
      }
      const std::size_t i = static_cast<std::size_t>(idx);
      if (i >= usable_count)
      {
        return;
      }
      if (i >= stand_control_mask_.size() || stand_control_mask_[i] == 0U)
      {
        return;
      }
      cmd_msg.effort[i] += delta;
      if (stand_effort_abs_max_ > 0.0)
      {
        cmd_msg.effort[i] = std::clamp(cmd_msg.effort[i], -stand_effort_abs_max_, stand_effort_abs_max_);
      }
    };

    add_effort(idx_left_hip_roll_, +tilt_cmd.u_roll * tilt_cmd.roll_w_hip);
    add_effort(idx_right_hip_roll_, -tilt_cmd.u_roll * tilt_cmd.roll_w_hip);
    add_effort(idx_left_ankle_roll_, +tilt_cmd.u_roll * tilt_cmd.roll_w_ankle);
    add_effort(idx_right_ankle_roll_, -tilt_cmd.u_roll * tilt_cmd.roll_w_ankle);
    add_effort(idx_torso_, +tilt_cmd.u_roll * tilt_cmd.roll_w_torso);

    add_effort(idx_left_hip_pitch_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_hip);
    add_effort(idx_right_hip_pitch_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_hip);
    add_effort(idx_left_ankle_pitch_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_ankle);
    add_effort(idx_right_ankle_pitch_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_ankle);
    add_effort(idx_left_knee_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_knee);
    add_effort(idx_right_knee_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_knee);
    add_effort(idx_torso_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_torso);
  }

  // qref_bias 모드에서는 effort를 바로 더하지 않고, 목표 자세(q_ref) 자체를 살짝 움직여 복원 방향을 만든다.
  void apply_tilt_feedback_qref_bias(
      std::vector<double> &effective_q_ref, const rbci::TiltFeedbackCommand &tilt_cmd)
  {
    const auto add_bias = [&](int idx, double delta)
    {
      if (idx < 0)
      {
        return;
      }
      const std::size_t i = static_cast<std::size_t>(idx);
      if (i >= effective_q_ref.size())
      {
        return;
      }
      if (i >= stand_control_mask_.size() || stand_control_mask_[i] == 0U)
      {
        return;
      }
      effective_q_ref[i] += delta;
    };

    add_bias(idx_left_hip_roll_, +tilt_cmd.u_roll * tilt_cmd.roll_w_hip);
    add_bias(idx_right_hip_roll_, -tilt_cmd.u_roll * tilt_cmd.roll_w_hip);
    add_bias(idx_left_ankle_roll_, +tilt_cmd.u_roll * tilt_cmd.roll_w_ankle);
    add_bias(idx_right_ankle_roll_, -tilt_cmd.u_roll * tilt_cmd.roll_w_ankle);
    add_bias(idx_torso_, +tilt_cmd.u_roll * tilt_cmd.roll_w_torso);

    add_bias(idx_left_hip_pitch_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_hip);
    add_bias(idx_right_hip_pitch_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_hip);
    add_bias(idx_left_ankle_pitch_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_ankle);
    add_bias(idx_right_ankle_pitch_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_ankle);
    add_bias(idx_left_knee_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_knee);
    add_bias(idx_right_knee_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_knee);
    add_bias(idx_torso_, +tilt_cmd.u_pitch * tilt_cmd.pitch_w_torso);
  }

  void reset_pitch_chain_debug_snapshot()
  {
    runtime_state_.pitch_chain = rbci::PitchChainDebugSnapshot{};
  }

  void reset_ankle_effort_debug_snapshot()
  {
    runtime_state_.ankle_effort = rbci::AnkleEffortDebugSnapshot{};
  }

  static double read_joint_value_or_nan(const std::vector<double> &values, int idx)
  {
    if (idx < 0)
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const std::size_t i = static_cast<std::size_t>(idx);
    if (i >= values.size())
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return values[i];
  }

  static double average_joint_pair_or_nan(const std::vector<double> &values, int idx_a, int idx_b)
  {
    double sum = 0.0;
    std::size_t count = 0U;
    const double value_a = read_joint_value_or_nan(values, idx_a);
    if (!std::isnan(value_a))
    {
      sum += value_a;
      ++count;
    }
    const double value_b = read_joint_value_or_nan(values, idx_b);
    if (!std::isnan(value_b))
    {
      sum += value_b;
      ++count;
    }
    if (count == 0U)
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return sum / static_cast<double>(count);
  }

  double applied_qref_bias_or_nan(int idx, double delta) const
  {
    if (idx < 0)
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const std::size_t i = static_cast<std::size_t>(idx);
    if (i >= stand_control_mask_.size())
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return (stand_control_mask_[i] == 0U) ? 0.0 : delta;
  }

  double average_applied_qref_bias_or_nan(int idx_a, int idx_b, double delta) const
  {
    double sum = 0.0;
    std::size_t count = 0U;
    const double bias_a = applied_qref_bias_or_nan(idx_a, delta);
    if (!std::isnan(bias_a))
    {
      sum += bias_a;
      ++count;
    }
    const double bias_b = applied_qref_bias_or_nan(idx_b, delta);
    if (!std::isnan(bias_b))
    {
      sum += bias_b;
      ++count;
    }
    if (count == 0U)
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return sum / static_cast<double>(count);
  }

  // loop_stats에서 바로 볼 수 있게 실제 적용된 pitch-chain ref/meas/bias를 스냅샷으로 저장한다.
  void capture_pitch_chain_debug_snapshot(
      const std::vector<double> &effective_q_ref, const rbci::TiltFeedbackCommand *tilt_cmd)
  {
    runtime_state_.pitch_chain.torso_ref = read_joint_value_or_nan(effective_q_ref, idx_torso_);
    runtime_state_.pitch_chain.torso_meas = read_joint_value_or_nan(estimated_state_.joint_positions, idx_torso_);
    runtime_state_.pitch_chain.hip_ref_avg =
        average_joint_pair_or_nan(effective_q_ref, idx_left_hip_pitch_, idx_right_hip_pitch_);
    runtime_state_.pitch_chain.hip_meas_avg =
        average_joint_pair_or_nan(estimated_state_.joint_positions, idx_left_hip_pitch_, idx_right_hip_pitch_);
    runtime_state_.pitch_chain.knee_ref_avg =
        average_joint_pair_or_nan(effective_q_ref, idx_left_knee_, idx_right_knee_);
    runtime_state_.pitch_chain.knee_meas_avg =
        average_joint_pair_or_nan(estimated_state_.joint_positions, idx_left_knee_, idx_right_knee_);
    runtime_state_.pitch_chain.ankle_ref_avg =
        average_joint_pair_or_nan(effective_q_ref, idx_left_ankle_pitch_, idx_right_ankle_pitch_);
    runtime_state_.pitch_chain.ankle_meas_avg =
        average_joint_pair_or_nan(estimated_state_.joint_positions, idx_left_ankle_pitch_, idx_right_ankle_pitch_);
    if (tilt_cmd == nullptr)
    {
      runtime_state_.pitch_chain.bias_torso = 0.0;
      runtime_state_.pitch_chain.bias_hip = 0.0;
      runtime_state_.pitch_chain.bias_knee = 0.0;
      runtime_state_.pitch_chain.bias_ankle = 0.0;
      return;
    }

    runtime_state_.pitch_chain.bias_hip = average_applied_qref_bias_or_nan(
        idx_left_hip_pitch_, idx_right_hip_pitch_, tilt_cmd->u_pitch * tilt_cmd->pitch_w_hip);
    runtime_state_.pitch_chain.bias_knee = average_applied_qref_bias_or_nan(
        idx_left_knee_, idx_right_knee_, tilt_cmd->u_pitch * tilt_cmd->pitch_w_knee);
    runtime_state_.pitch_chain.bias_ankle = average_applied_qref_bias_or_nan(
        idx_left_ankle_pitch_, idx_right_ankle_pitch_, tilt_cmd->u_pitch * tilt_cmd->pitch_w_ankle);
    runtime_state_.pitch_chain.bias_torso =
        applied_qref_bias_or_nan(idx_torso_, tilt_cmd->u_pitch * tilt_cmd->pitch_w_torso);
  }

  /**
   * @brief 최소 스탠드 제어(PD)로 effort 명령을 계산한다.
   *
   * q_ref를 기준으로 (kp*(q_ref-q) - kd*dq) 를 각 조인트 effort로 출력한다.
   */
  void apply_stand_pd(sensor_msgs::msg::JointState &cmd_msg)
  {
    last_stand_output_scale_ = 1.0;
    if (!estimated_state_.received)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "stand_pd waiting for /rb/estimated_state samples");
      return;
    }
    if (!estimated_state_.joint_state_valid)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "stand_pd waiting for valid joint state in /rb/estimated_state");
      return;
    }

    if (!ensure_stand_reference_ready())
    {
      return;
    }
    if (!ensure_stand_control_mask_ready())
    {
      return;
    }
    if (!ensure_stand_gain_map_ready())
    {
      return;
    }
    if (limit_avoid_enable_ && !ensure_limit_avoid_map_ready())
    {
      return;
    }

    // tilt cut이 걸리면 stand PD 전체 출력을 비율로 낮춰 급격한 발산을 막는다.
    const double stand_output_scale = compute_stand_output_scale();
    last_stand_output_scale_ = stand_output_scale;
    std::vector<double> effective_q_ref = stand_q_ref_active_;

    const std::size_t usable_count = std::min(
        std::min(cmd_msg.effort.size(), estimated_state_.joint_positions.size()),
        std::min(estimated_state_.joint_velocities.size(), stand_q_ref_active_.size()));

    rbci::TiltFeedbackCommand tilt_cmd;
    const bool has_tilt_cmd = compute_tilt_feedback_command(tilt_cmd);
    if (has_tilt_cmd && tilt_apply_mode_ == "qref_bias")
    {
      apply_tilt_feedback_qref_bias(effective_q_ref, tilt_cmd);
    }
    const rbci::TiltFeedbackCommand *pitch_bias_debug_cmd =
        (has_tilt_cmd && tilt_apply_mode_ == "qref_bias") ? &tilt_cmd : nullptr;
    capture_pitch_chain_debug_snapshot(effective_q_ref, pitch_bias_debug_cmd);

    double ankle_pd_sum = 0.0;
    double ankle_limit_sum = 0.0;
    double ankle_pre_clamp_sum = 0.0;
    std::size_t ankle_sample_count = 0U;

    // 핵심 흐름:
    // 1) q_ref와 현재 joint 상태로 기본 PD effort 계산
    // 2) limit avoid를 더함
    // 3) tilt cut scale을 곱함
    // 4) 필요하면 tilt feedback을 추가 반영
    for (std::size_t i = 0; i < usable_count; ++i)
    {
      if (i >= stand_control_mask_.size() || stand_control_mask_[i] == 0U)
      {
        cmd_msg.effort[i] = 0.0;
        continue;
      }

      double position_error = effective_q_ref[i] - estimated_state_.joint_positions[i];
      if (stand_pos_error_abs_max_ > 0.0)
      {
        position_error = std::clamp(position_error, -stand_pos_error_abs_max_, stand_pos_error_abs_max_);
      }
      const double damping_term = -estimated_state_.joint_velocities[i];
      const double kp_eff = stand_kp_ * stand_kp_scale_per_joint_[i];
      const double kd_eff = stand_kd_ * stand_kd_scale_per_joint_[i];
      const double effort_pd = (kp_eff * position_error) + (kd_eff * damping_term);
      const double effort_limit_avoid =
          compute_limit_avoid_effort(i, estimated_state_.joint_positions[i], estimated_state_.joint_velocities[i]);
      const double effort_pre_clamp = (effort_pd + effort_limit_avoid) * stand_output_scale;
      double effort_cmd = effort_pre_clamp;
      if (stand_effort_abs_max_ > 0.0)
      {
        effort_cmd = std::clamp(effort_cmd, -stand_effort_abs_max_, stand_effort_abs_max_);
      }
      cmd_msg.effort[i] = effort_cmd;

      if (static_cast<int>(i) == idx_left_ankle_pitch_ || static_cast<int>(i) == idx_right_ankle_pitch_)
      {
        ankle_pd_sum += effort_pd;
        ankle_limit_sum += effort_limit_avoid;
        ankle_pre_clamp_sum += effort_pre_clamp;
        ++ankle_sample_count;
      }
    }

    // effort 모드는 기본 PD 출력이 완성된 뒤 마지막에 additive effort로 얹는다.
    if (has_tilt_cmd && tilt_apply_mode_ == "effort")
    {
      apply_tilt_feedback_effort(cmd_msg, usable_count, tilt_cmd);
    }

    if (ankle_sample_count == 0U)
    {
      return;
    }

    double ankle_cmd_sum = 0.0;
    std::size_t ankle_sat_count = 0U;
    const double sat_threshold =
        (stand_effort_abs_max_ > 0.0) ? (stand_effort_abs_max_ - 1e-6) : std::numeric_limits<double>::infinity();
    const auto accumulate_final_ankle = [&](int idx)
    {
      if (idx < 0)
      {
        return;
      }
      const std::size_t joint_idx = static_cast<std::size_t>(idx);
      if (joint_idx >= cmd_msg.effort.size())
      {
        return;
      }
      const double effort = cmd_msg.effort[joint_idx];
      ankle_cmd_sum += effort;
      if (stand_effort_abs_max_ > 0.0 && std::abs(effort) >= sat_threshold)
      {
        ++ankle_sat_count;
      }
    };
    accumulate_final_ankle(idx_left_ankle_pitch_);
    accumulate_final_ankle(idx_right_ankle_pitch_);

    const double ankle_count = static_cast<double>(ankle_sample_count);
    runtime_state_.ankle_effort.pd_avg = ankle_pd_sum / ankle_count;
    runtime_state_.ankle_effort.limit_avg = ankle_limit_sum / ankle_count;
    runtime_state_.ankle_effort.pre_clamp_avg = ankle_pre_clamp_sum / ankle_count;
    runtime_state_.ankle_effort.cmd_avg = ankle_cmd_sum / ankle_count;
    runtime_state_.ankle_effort.sat_count = ankle_sat_count;
  }

  /**
   * @brief IMU 기울기에 따라 stand 출력 스케일을 계산한다.
   *
   * cut/recover 히스테리시스를 사용해 기울기 급증 구간에서 출력을 선제적으로 낮춘다.
   * @return stand 출력 스케일(0~1)
   */
  double compute_stand_output_scale()
  {
    if (!stand_tilt_cut_enable_)
    {
      stand_tilt_cut_active_ = false;
      return 1.0;
    }
    if (!estimated_state_.received)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "stand_tilt_cut enabled but /rb/estimated_state is not ready yet");
      return 1.0;
    }
    if (!estimated_state_.imu_valid)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "stand_tilt_cut enabled but IMU state in /rb/estimated_state is invalid");
      return 1.0;
    }

    // roll/pitch 중 더 큰 축을 기준으로 현재 기울기 위험도를 본다.
    const double tilt_abs = std::max(
        std::abs(estimated_state_.tilt_roll_rad), std::abs(estimated_state_.tilt_pitch_rad));
    if (!stand_tilt_cut_active_)
    {
      if (tilt_abs >= stand_tilt_cut_rad_)
      {
        stand_tilt_cut_active_ = true;
        RCLCPP_WARN(
            this->get_logger(),
            "stand_tilt_cut active: tilt=%.3f rad (cut=%.3f, recover=%.3f, scale=%.2f)",
            tilt_abs, stand_tilt_cut_rad_, stand_tilt_recover_rad_, stand_tilt_cut_output_scale_);
      }
    }
    else
    {
      if (tilt_abs <= stand_tilt_recover_rad_)
      {
        stand_tilt_cut_active_ = false;
        RCLCPP_INFO(
            this->get_logger(),
            "stand_tilt_cut cleared: tilt=%.3f rad", tilt_abs);
      }
    }
    return stand_tilt_cut_active_ ? stand_tilt_cut_output_scale_ : 1.0;
  }

  /**
   * @brief 조인트별 lower/upper limit 테이블을 joint_names 순서로 매핑한다.
   * @return 매핑이 준비되면 true
   */
  bool ensure_limit_avoid_map_ready()
  {
    if (limit_avoid_map_ready_)
    {
      return true;
    }

    const auto result = rbci::build_limit_avoid_map(
        joint_names_, limit_avoid_joint_names_param_, limit_avoid_lower_param_, limit_avoid_upper_param_);
    if (result.invalid_table)
    {
      if (!limit_avoid_invalid_warned_)
      {
        limit_avoid_invalid_warned_ = true;
        RCLCPP_WARN(
            this->get_logger(),
            "limit_avoid table invalid: names=%zu lower=%zu upper=%zu",
            limit_avoid_joint_names_param_.size(),
            limit_avoid_lower_param_.size(),
            limit_avoid_upper_param_.size());
      }
      return false;
    }
    if (result.no_joint_match)
    {
      if (!limit_avoid_invalid_warned_)
      {
        limit_avoid_invalid_warned_ = true;
        RCLCPP_WARN(
            this->get_logger(),
            "limit_avoid table has no joint match with current joint_names");
      }
      return false;
    }
    if (!result.ready)
    {
      return false;
    }

    limit_avoid_lower_per_joint_ = result.lower_per_joint;
    limit_avoid_upper_per_joint_ = result.upper_per_joint;
    limit_avoid_mask_ = result.mask;
    limit_avoid_map_ready_ = true;
    RCLCPP_INFO(
        this->get_logger(),
        "limit_avoid map ready: mapped=%zu/%zu margin=%.4f kp=%.3f kd=%.3f",
        result.mapped_count, joint_names_.size(), limit_avoid_margin_rad_, limit_avoid_kp_, limit_avoid_kd_);
    return true;
  }

  /**
   * @brief lower/upper guard를 기준으로 한 soft-limit 회피 effort를 계산한다.
   * @param idx 조인트 인덱스
   * @param q 현재 조인트 각도
   * @param dq 현재 조인트 속도
   * @return limit 회피 보정 effort
   */
  double compute_limit_avoid_effort(std::size_t idx, double q, double dq)
  {
    if (!limit_avoid_enable_ || !limit_avoid_map_ready_)
    {
      return 0.0;
    }
    if (idx >= limit_avoid_mask_.size() || limit_avoid_mask_[idx] == 0U)
    {
      return 0.0;
    }
    if (idx >= limit_avoid_lower_per_joint_.size() || idx >= limit_avoid_upper_per_joint_.size())
    {
      return 0.0;
    }

    const double lo = limit_avoid_lower_per_joint_[idx];
    const double hi = limit_avoid_upper_per_joint_[idx];
    const double lo_guard = lo + limit_avoid_margin_rad_;
    const double hi_guard = hi - limit_avoid_margin_rad_;
    if (hi_guard <= lo_guard)
    {
      return 0.0;
    }

    double u = 0.0;
    // lower guard 안쪽이면 +방향으로 밀고, 더 내려가는 속도를 감쇠한다.
    if (q < lo_guard)
    {
      const double pos_err = lo_guard - q;
      const double vel_err = std::max(0.0, -dq);
      u += (limit_avoid_kp_ * pos_err) + (limit_avoid_kd_ * vel_err);
    }
    // upper guard 안쪽이면 -방향으로 밀고, 더 올라가는 속도를 감쇠한다.
    if (q > hi_guard)
    {
      const double pos_err = q - hi_guard;
      const double vel_err = std::max(0.0, dq);
      u -= (limit_avoid_kp_ * pos_err) + (limit_avoid_kd_ * vel_err);
    }
    return u;
  }

  /**
   * @brief stand_pd 기준자세(q_ref)를 준비한다.
   *
   * 우선순위:
   * 1) stand_q_ref baseline 파라미터(길이 일치 시)
   *    + stand_q_ref_trim 파라미터(길이 일치 시 baseline에 더함)
   * 2) stand_hold_current_on_start=true 이면 현재 joint position 스냅샷
   */
  bool ensure_stand_reference_ready()
  {
    if (stand_reference_ready_)
    {
      return true;
    }

    const auto result = rbci::build_stand_reference(
        joint_names_, estimated_state_.joint_positions, stand_q_ref_param_, stand_q_ref_trim_param_, stand_hold_current_on_start_);
    if (result.q_ref_size_mismatch && !stand_ref_param_size_warned_)
    {
      stand_ref_param_size_warned_ = true;
      RCLCPP_WARN(
          this->get_logger(),
          "stand_q_ref size mismatch: ref=%zu joint_count=%zu. fallback to current pose=%s",
          stand_q_ref_param_.size(), joint_names_.size(),
          stand_hold_current_on_start_ ? "enabled" : "disabled");
    }
    if (result.trim_size_mismatch && !stand_ref_trim_param_size_warned_)
    {
      stand_ref_trim_param_size_warned_ = true;
      RCLCPP_WARN(
          this->get_logger(),
          "stand_q_ref_trim size mismatch: trim=%zu joint_count=%zu. ignore trim and use baseline stand_q_ref only",
          stand_q_ref_trim_param_.size(), joint_names_.size());
    }
    if (!result.ready)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "stand_pd reference is not ready (provide stand_q_ref or enable stand_hold_current_on_start)");
      return false;
    }

    stand_q_ref_active_ = result.values;
    stand_reference_ready_ = true;
    if (result.used_current_pose)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "stand_pd reference captured from current pose (count=%zu)",
          stand_q_ref_active_.size());
    }
    else
    {
      RCLCPP_INFO(
          this->get_logger(),
          "stand_pd reference loaded from stand_q_ref baseline%s (count=%zu)",
          result.trim_applied ? " + stand_q_ref_trim" : "",
          stand_q_ref_active_.size());
    }
    return true;
  }

  /**
   * @brief stand_pd 대상 조인트 마스크를 준비한다.
   *
   * stand_control_joints 파라미터가 비어 있으면 모든 조인트를 제어 대상으로 본다.
   */
  bool ensure_stand_control_mask_ready()
  {
    if (stand_control_mask_ready_)
    {
      return true;
    }

    const auto result = rbci::build_stand_control_mask(joint_names_, stand_control_joints_param_);
    if (!result.ready)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "stand_control_joints has no match with current joint_names. stand_pd outputs stay zero.");
      return false;
    }

    stand_control_mask_ = result.mask;
    if (!result.unknown_names.empty() && !stand_control_unknown_warned_)
    {
      stand_control_unknown_warned_ = true;
      RCLCPP_WARN(
          this->get_logger(),
          "stand_control_joints contains %zu unknown names (ignored). matched=%zu/%zu",
          result.unknown_names.size(), result.matched_count, joint_names_.size());
    }

    stand_control_mask_ready_ = true;
    if (!result.all_joints)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "stand_control_joints active: matched=%zu / total=%zu",
          result.matched_count, joint_names_.size());
    }
    return true;
  }

  /**
   * @brief 조인트 이름 기반으로 그룹별 PD 게인 스케일 맵을 준비한다.
   *
   * 분류 규칙:
   * - hip: 이름에 "_hip_" 포함
   * - knee: 이름에 "_knee_" 포함
   * - ankle: 이름에 "_ankle_" 포함
   * - torso: 이름이 "torso_joint"
   * - 기타: 나머지
   */
  bool ensure_stand_gain_map_ready()
  {
    if (stand_gain_map_ready_)
    {
      return true;
    }

    const auto result = rbci::build_stand_gain_map(
        joint_names_,
        stand_kp_scale_hip_, stand_kp_scale_knee_, stand_kp_scale_ankle_, stand_kp_scale_torso_, stand_kp_scale_other_,
        stand_kd_scale_hip_, stand_kd_scale_knee_, stand_kd_scale_ankle_, stand_kd_scale_torso_, stand_kd_scale_other_);
    if (!result.ready)
    {
      return false;
    }

    stand_kp_scale_per_joint_ = result.kp_scales;
    stand_kd_scale_per_joint_ = result.kd_scales;
    stand_gain_map_ready_ = true;
    RCLCPP_INFO(
        this->get_logger(),
        "stand gain map ready: hip=%zu knee=%zu ankle=%zu torso=%zu other=%zu kp_scales=%.2f/%.2f/%.2f/%.2f/%.2f kd_scales=%.2f/%.2f/%.2f/%.2f/%.2f",
        result.hip_count, result.knee_count, result.ankle_count, result.torso_count, result.other_count,
        stand_kp_scale_hip_, stand_kp_scale_knee_, stand_kp_scale_ankle_, stand_kp_scale_torso_, stand_kp_scale_other_,
        stand_kd_scale_hip_, stand_kd_scale_knee_, stand_kd_scale_ankle_, stand_kd_scale_torso_, stand_kd_scale_other_);
    return true;
  }

  /**
   * @brief tilt 보정용 핵심 조인트 인덱스를 joint_names 기준으로 캐시한다.
   * @return 하나 이상의 roll/pitch 축 제어 조인트를 찾으면 true
   */
  bool ensure_tilt_joint_index_ready()
  {
    if (tilt_joint_index_ready_)
    {
      return true;
    }

    const auto indices = rbci::build_tilt_joint_indices(joint_names_);
    if (!indices.ready())
    {
      if (!tilt_joint_index_warned_)
      {
        tilt_joint_index_warned_ = true;
        RCLCPP_WARN(
            this->get_logger(),
            "tilt feedback joint indices are not available in current joint_names");
      }
      return false;
    }

    idx_left_hip_roll_ = indices.left_hip_roll;
    idx_right_hip_roll_ = indices.right_hip_roll;
    idx_left_ankle_roll_ = indices.left_ankle_roll;
    idx_right_ankle_roll_ = indices.right_ankle_roll;
    idx_left_hip_pitch_ = indices.left_hip_pitch;
    idx_right_hip_pitch_ = indices.right_hip_pitch;
    idx_left_ankle_pitch_ = indices.left_ankle_pitch;
    idx_right_ankle_pitch_ = indices.right_ankle_pitch;
    idx_left_knee_ = indices.left_knee;
    idx_right_knee_ = indices.right_knee;
    idx_torso_ = indices.torso;

    tilt_joint_index_ready_ = true;
    RCLCPP_INFO(
        this->get_logger(),
        "tilt feedback joint map ready: roll(hip %d/%d, ankle %d/%d, torso %d), pitch(hip %d/%d, ankle %d/%d, knee %d/%d, torso %d)",
        idx_left_hip_roll_, idx_right_hip_roll_, idx_left_ankle_roll_, idx_right_ankle_roll_,
        idx_torso_,
        idx_left_hip_pitch_, idx_right_hip_pitch_, idx_left_ankle_pitch_, idx_right_ankle_pitch_,
        idx_left_knee_, idx_right_knee_, idx_torso_);
    return true;
  }

  /**
   * @brief IMU roll/pitch를 이용해 stand effort에 미세 보정 항을 더한다.
   *
   * roll은 좌/우 hip_roll/ankle_roll에 반대 부호로 분배,
   * pitch는 좌/우 hip_pitch/ankle_pitch에 같은 부호로 분배한다.
   */
  // ===== 런타임 파라미터/설정값 =====
  double control_rate_hz_{200.0};
  double log_interval_sec_{5.0};
  double miss_ratio_threshold_{1.2};
  double expected_dt_sec_{0.005};
  std::string input_topic_;
  std::string output_topic_;
  std::string signal_mode_{"zero"};
  std::string target_joint_{"torso_joint"};
  double effort_amplitude_{0.0};
  double effort_frequency_hz_{0.5};
  double step_start_sec_{1.0};
  double stand_kp_{30.0};
  double stand_kd_{2.0};
  double stand_effort_abs_max_{6.0};
  bool stand_hold_current_on_start_{true};
  double stand_kp_scale_hip_{1.0};
  double stand_kd_scale_hip_{1.0};
  double stand_kp_scale_knee_{1.0};
  double stand_kd_scale_knee_{1.0};
  double stand_kp_scale_ankle_{1.0};
  double stand_kd_scale_ankle_{1.0};
  double stand_kp_scale_torso_{1.0};
  double stand_kd_scale_torso_{1.0};
  double stand_kp_scale_other_{1.0};
  double stand_kd_scale_other_{1.0};
  double stand_pos_error_abs_max_{0.35};
  bool stand_tilt_cut_enable_{true};
  double stand_tilt_cut_rad_{0.45};
  double stand_tilt_recover_rad_{0.35};
  double stand_tilt_cut_output_scale_{0.0};
  std::size_t stand_debug_top_error_count_{3};
  double last_stand_output_scale_{1.0};
  std::vector<double> stand_q_ref_param_;
  std::vector<double> stand_q_ref_trim_param_;
  std::vector<std::string> stand_control_joints_param_;
  bool limit_avoid_enable_{false};
  double limit_avoid_margin_rad_{0.05};
  double limit_avoid_kp_{8.0};
  double limit_avoid_kd_{1.0};
  std::vector<std::string> limit_avoid_joint_names_param_;
  std::vector<double> limit_avoid_lower_param_;
  std::vector<double> limit_avoid_upper_param_;
  bool enable_tilt_feedback_{true};
  double tilt_kp_roll_{10.0};
  double tilt_kd_roll_{2.0};
  double tilt_kp_pitch_{12.0};
  double tilt_kd_pitch_{2.5};
  double tilt_deadband_rad_{0.03};
  std::string tilt_apply_mode_{"effort"};
  double tilt_effort_abs_max_{1.8};
  double tilt_qref_bias_abs_max_{0.20};
  double tilt_roll_sign_{1.0};
  double tilt_pitch_sign_{1.0};
  double tilt_weight_roll_hip_{0.7};
  double tilt_weight_roll_ankle_{0.3};
  double tilt_weight_roll_torso_{0.0};
  double tilt_weight_pitch_hip_{0.6};
  double tilt_weight_pitch_ankle_{0.3};
  double tilt_weight_pitch_knee_{0.1};
  double tilt_weight_pitch_torso_{0.0};

  // ===== 상태 캐시/통계 버퍼 =====
  std::vector<std::string> joint_names_;
  rbci::EstimatedStateCache estimated_state_{};
  rbci::ControllerRuntimeState runtime_state_{};
  std::vector<double> stand_q_ref_active_;
  std::vector<double> stand_kp_scale_per_joint_;
  std::vector<double> stand_kd_scale_per_joint_;
  std::vector<std::uint8_t> stand_control_mask_;
  std::vector<std::uint8_t> limit_avoid_mask_;
  std::vector<double> limit_avoid_lower_per_joint_;
  std::vector<double> limit_avoid_upper_per_joint_;

  bool stand_reference_ready_{false};
  bool stand_ref_param_size_warned_{false};
  bool stand_ref_trim_param_size_warned_{false};
  bool stand_control_mask_ready_{false};
  bool stand_gain_map_ready_{false};
  bool stand_control_unknown_warned_{false};
  bool stand_tilt_cut_active_{false};
  bool limit_avoid_map_ready_{false};
  bool limit_avoid_invalid_warned_{false};
  bool tilt_joint_index_ready_{false};
  bool tilt_joint_index_warned_{false};

  int idx_left_hip_roll_{-1};
  int idx_right_hip_roll_{-1};
  int idx_left_ankle_roll_{-1};
  int idx_right_ankle_roll_{-1};
  int idx_left_hip_pitch_{-1};
  int idx_right_hip_pitch_{-1};
  int idx_left_ankle_pitch_{-1};
  int idx_right_ankle_pitch_{-1};
  int idx_left_knee_{-1};
  int idx_right_knee_{-1};
  int idx_torso_{-1};

  // ===== ROS2 통신 핸들 =====
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Subscription<rb_controller::msg::EstimatedState>::SharedPtr estimated_state_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char **argv)
{
  // ROS2 초기화 -> 노드 생성/스핀 -> 종료
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RbControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
