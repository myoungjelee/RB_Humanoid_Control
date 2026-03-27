#ifndef RB_STANDING_CONTROLLER__RB_STANDING_CONTROLLER_HPP_
#define RB_STANDING_CONTROLLER__RB_STANDING_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rb_interfaces/msg/estimated_state.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace rb_standing_controller
{

/**
 * @brief non-RT subscription callback에서 RT update() loop로 넘길 최소 추정 상태 묶음.
 *
 * controller plugin은 joint position/velocity를 state interface에서 직접 읽고,
 * 여기에는 tilt feedback에 필요한 IMU/tilt 값만 따로 담는다.
 */
struct EstimatedStateInput
{
  bool received{false};
  bool imu_valid{false};
  double tilt_roll_rad{0.0};
  double tilt_pitch_rad{0.0};
  double roll_rate_rad_s{0.0};
  double pitch_rate_rad_s{0.0};
};

/**
 * @brief tilt feedback 계산 결과를 체인 분배 비율과 함께 담는 중간 표현.
 *
 * 같은 tilt 계산 결과를 `effort additive` 모드와 `q_ref bias` 모드에서
 * 공통으로 재사용하기 위해 별도 구조체로 분리한다.
 */
struct TiltFeedbackCommand
{
  bool valid{false};
  double u_roll{0.0};
  double u_pitch{0.0};
  double roll_w_hip{0.0};
  double roll_w_ankle{0.0};
  double roll_w_torso{0.0};
  double pitch_w_hip{0.0};
  double pitch_w_ankle{0.0};
  double pitch_w_knee{0.0};
  double pitch_w_torso{0.0};
};

/**
 * @brief ros2_control 안에서 native stand_pd를 수행하는 custom controller plugin.
 *
 * - joint state는 ros2_control state interface에서 직접 읽는다.
 * - tilt/rate 같은 보조 추정값은 /rb/estimated_state로 받는다.
 * - 최종 effort는 command interface에 바로 써서 hardware_interface로 내려보낸다.
 */
class RBStandingController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void on_estimated_state(const rb_interfaces::msg::EstimatedState::SharedPtr msg);
  void zero_all_commands();
  bool build_interface_index_maps();
  bool build_current_state_vectors();
  bool ensure_stand_reference_ready();
  bool ensure_stand_control_mask_ready();
  bool ensure_stand_gain_map_ready();
  bool ensure_limit_avoid_map_ready();
  bool ensure_tilt_joint_index_ready();
  double compute_stand_output_scale(const EstimatedStateInput & estimated_state);
  bool compute_tilt_feedback_command(
    const EstimatedStateInput & estimated_state, TiltFeedbackCommand & cmd) const;
  void apply_tilt_feedback_effort(
    const TiltFeedbackCommand & tilt_cmd, std::size_t usable_count);
  void apply_tilt_feedback_qref_bias(
    std::vector<double> & effective_q_ref, const TiltFeedbackCommand & tilt_cmd) const;
  double compute_limit_avoid_effort(std::size_t idx, double q, double dq) const;

  std::vector<std::string> joint_names_;
  std::string estimated_state_topic_{"/rb/estimated_state"};
  std::string command_interface_name_{"effort"};
  std::vector<std::string> state_interface_names_{"position", "velocity", "effort"};
  double stand_kp_{6.0};
  double stand_kd_{8.0};
  double stand_effort_abs_max_{1.4};
  bool stand_hold_current_on_start_{true};
  std::vector<double> stand_q_ref_param_;
  std::vector<double> stand_q_ref_trim_param_;
  std::vector<std::string> stand_control_joints_param_;
  double stand_pos_error_abs_max_{0.18};
  bool stand_tilt_cut_enable_{true};
  double stand_tilt_cut_rad_{0.40};
  double stand_tilt_recover_rad_{0.30};
  double stand_tilt_cut_output_scale_{0.30};
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
  bool limit_avoid_enable_{false};
  double limit_avoid_margin_rad_{0.08};
  double limit_avoid_kp_{10.0};
  double limit_avoid_kd_{2.0};
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

  std::vector<int> command_interface_indices_;
  std::vector<int> position_state_indices_;
  std::vector<int> velocity_state_indices_;
  std::vector<double> current_positions_;
  std::vector<double> current_velocities_;
  std::vector<double> stand_q_ref_active_;
  std::vector<double> stand_kp_scale_per_joint_;
  std::vector<double> stand_kd_scale_per_joint_;
  std::vector<std::uint8_t> stand_control_mask_;
  std::vector<std::uint8_t> limit_avoid_mask_;
  std::vector<double> limit_avoid_lower_per_joint_;
  std::vector<double> limit_avoid_upper_per_joint_;
  std::size_t position_state_offset_{0U};
  std::size_t velocity_state_offset_{1U};
  std::size_t state_stride_{3U};

  bool stand_reference_ready_{false};
  bool stand_ref_param_size_warned_{false};
  bool stand_ref_trim_param_size_warned_{false};
  bool stand_control_mask_ready_{false};
  bool stand_control_unknown_warned_{false};
  bool stand_gain_map_ready_{false};
  bool stand_tilt_cut_active_{false};
  bool limit_avoid_map_ready_{false};
  bool limit_avoid_invalid_warned_{false};
  bool tilt_joint_index_ready_{false};
  bool tilt_joint_index_warned_{false};
  bool estimator_wait_warned_{false};

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

  realtime_tools::RealtimeBuffer<EstimatedStateInput> estimated_state_buffer_;
  rclcpp::Subscription<rb_interfaces::msg::EstimatedState>::SharedPtr estimated_state_sub_;
};

}  // namespace rb_standing_controller

#endif  // RB_STANDING_CONTROLLER__RB_STANDING_CONTROLLER_HPP_
