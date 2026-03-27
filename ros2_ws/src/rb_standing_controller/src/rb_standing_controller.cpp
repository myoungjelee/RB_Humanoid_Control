#include "rb_standing_controller/rb_standing_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace
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

struct StandControlMaskBuildResult
{
  bool ready{false};
  bool all_joints{false};
  std::size_t matched_count{0U};
  std::vector<std::uint8_t> mask{};
  std::vector<std::string> unknown_names{};
};

struct StandGainMapBuildResult
{
  bool ready{false};
  std::vector<double> kp_scales{};
  std::vector<double> kd_scales{};
};

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

  bool ready() const
  {
    return ((left_hip_roll >= 0) && (right_hip_roll >= 0)) ||
           ((left_hip_pitch >= 0) && (right_hip_pitch >= 0)) ||
           ((left_ankle_pitch >= 0) && (right_ankle_pitch >= 0)) ||
           ((left_knee >= 0) && (right_knee >= 0));
  }
};

StandReferenceBuildResult build_stand_reference(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & latest_joint_positions,
  const std::vector<double> & stand_q_ref_param,
  const std::vector<double> & stand_q_ref_trim_param,
  bool stand_hold_current_on_start)
{
  // stand reference는 "파라미터 기준 자세" 또는 "activate 시점 현재 자세 hold" 둘 중 하나를 택한다.
  // vendor q_ref가 아직 안 정리됐을 때도 current-pose hold로 안전하게 시작할 수 있게 만든다.
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
  const std::vector<std::string> & joint_names,
  const std::vector<std::string> & stand_control_joints_param)
{
  // stand_pd 대상이 아닌 joint는 update()에서 완전히 건드리지 않도록 mask를 미리 만든다.
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
  const std::vector<std::string> & joint_names,
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
  // 같은 stand_kp/stand_kd라도 관절군(hip/knee/ankle/torso)별 스케일을 다르게 줄 수 있게 한다.
  StandGainMapBuildResult result;
  if (joint_names.empty())
  {
    return result;
  }

  result.kp_scales.assign(joint_names.size(), stand_kp_scale_other);
  result.kd_scales.assign(joint_names.size(), stand_kd_scale_other);

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    const std::string & joint = joint_names[i];
    if (joint == "torso_joint")
    {
      result.kp_scales[i] = stand_kp_scale_torso;
      result.kd_scales[i] = stand_kd_scale_torso;
      continue;
    }
    if (joint.find("_hip_") != std::string::npos)
    {
      result.kp_scales[i] = stand_kp_scale_hip;
      result.kd_scales[i] = stand_kd_scale_hip;
      continue;
    }
    if (joint.find("_knee_") != std::string::npos)
    {
      result.kp_scales[i] = stand_kp_scale_knee;
      result.kd_scales[i] = stand_kd_scale_knee;
      continue;
    }
    if (joint.find("_ankle_") != std::string::npos)
    {
      result.kp_scales[i] = stand_kp_scale_ankle;
      result.kd_scales[i] = stand_kd_scale_ankle;
      continue;
    }
  }

  result.ready = true;
  return result;
}

LimitAvoidMapBuildResult build_limit_avoid_map(
  const std::vector<std::string> & joint_names,
  const std::vector<std::string> & limit_avoid_joint_names_param,
  const std::vector<double> & limit_avoid_lower_param,
  const std::vector<double> & limit_avoid_upper_param)
{
  // limit avoid는 일부 joint table에만 적용하므로, 이름 -> joint index 매핑을 한 번 캐시한다.
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

TiltJointIndices build_tilt_joint_indices(const std::vector<std::string> & joint_names)
{
  // tilt feedback은 hip/knee/ankle/torso 체인 분배가 핵심이라, 이름 lookup을 매 tick 하지 않게
  // activate 이후 index cache를 만든다.
  TiltJointIndices result;
  if (joint_names.empty())
  {
    return result;
  }

  const auto find_idx = [&joint_names](const std::string & name) -> int
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

}  // namespace

namespace rb_standing_controller
{

controller_interface::CallbackReturn RBStandingController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("estimated_state_topic", "/rb/estimated_state");
  auto_declare<std::string>("command_interface_name", hardware_interface::HW_IF_EFFORT);
  auto_declare<std::vector<std::string>>(
    "state_interface_names",
    std::vector<std::string>{
      hardware_interface::HW_IF_POSITION,
      hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT,
    });

  // 기존 rb_controller 기본 standing 파라미터를 그대로 가져와, native plugin으로 옮겨도
  // 제어 감각이 크게 바뀌지 않도록 한다.
  auto_declare<double>("stand_kp", 6.0);
  auto_declare<double>("stand_kd", 8.0);
  auto_declare<double>("stand_effort_abs_max", 1.4);
  auto_declare<bool>("stand_hold_current_on_start", true);
  auto_declare<std::vector<double>>("stand_q_ref", std::vector<double>{0.0});
  auto_declare<std::vector<double>>("stand_q_ref_trim", std::vector<double>{});
  auto_declare<std::vector<std::string>>("stand_control_joints", std::vector<std::string>{});
  auto_declare<double>("stand_pos_error_abs_max", 0.18);
  auto_declare<bool>("stand_tilt_cut_enable", true);
  auto_declare<double>("stand_tilt_cut_rad", 0.40);
  auto_declare<double>("stand_tilt_recover_rad", 0.30);
  auto_declare<double>("stand_tilt_cut_output_scale", 0.30);
  auto_declare<double>("stand_kp_scale_hip", 1.0);
  auto_declare<double>("stand_kd_scale_hip", 1.0);
  auto_declare<double>("stand_kp_scale_knee", 1.0);
  auto_declare<double>("stand_kd_scale_knee", 1.0);
  auto_declare<double>("stand_kp_scale_ankle", 1.0);
  auto_declare<double>("stand_kd_scale_ankle", 1.0);
  auto_declare<double>("stand_kp_scale_torso", 1.0);
  auto_declare<double>("stand_kd_scale_torso", 1.0);
  auto_declare<double>("stand_kp_scale_other", 1.0);
  auto_declare<double>("stand_kd_scale_other", 1.0);
  auto_declare<bool>("limit_avoid_enable", true);
  auto_declare<double>("limit_avoid_margin_rad", 0.08);
  auto_declare<double>("limit_avoid_kp", 10.0);
  auto_declare<double>("limit_avoid_kd", 2.0);
  auto_declare<std::vector<std::string>>("limit_avoid_joint_names", std::vector<std::string>{});
  auto_declare<std::vector<double>>("limit_avoid_lower", std::vector<double>{});
  auto_declare<std::vector<double>>("limit_avoid_upper", std::vector<double>{});
  auto_declare<bool>("enable_tilt_feedback", true);
  auto_declare<double>("tilt_kp_roll", 10.0);
  auto_declare<double>("tilt_kd_roll", 2.0);
  auto_declare<double>("tilt_kp_pitch", 12.0);
  auto_declare<double>("tilt_kd_pitch", 2.5);
  auto_declare<double>("tilt_deadband_rad", 0.03);
  auto_declare<std::string>("tilt_apply_mode", "effort");
  auto_declare<double>("tilt_effort_abs_max", 1.8);
  auto_declare<double>("tilt_qref_bias_abs_max", 0.20);
  auto_declare<double>("tilt_roll_sign", 1.0);
  auto_declare<double>("tilt_pitch_sign", 1.0);
  auto_declare<double>("tilt_weight_roll_hip", 0.7);
  auto_declare<double>("tilt_weight_roll_ankle", 0.3);
  auto_declare<double>("tilt_weight_roll_torso", 0.0);
  auto_declare<double>("tilt_weight_pitch_hip", 0.6);
  auto_declare<double>("tilt_weight_pitch_ankle", 0.3);
  auto_declare<double>("tilt_weight_pitch_knee", 0.1);
  auto_declare<double>("tilt_weight_pitch_torso", 0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RBStandingController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_)
  {
    config.names.push_back(joint_name + "/" + command_interface_name_);
  }

  return config;
}

controller_interface::InterfaceConfiguration
RBStandingController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_)
  {
    for (const auto & state_interface_name : state_interface_names_)
    {
      config.names.push_back(joint_name + "/" + state_interface_name);
    }
  }

  return config;
}

void RBStandingController::on_estimated_state(
  const rb_interfaces::msg::EstimatedState::SharedPtr msg)
{
  EstimatedStateInput input;
  input.received = true;
  input.imu_valid = msg->imu_valid;
  input.tilt_roll_rad = msg->tilt_roll_rad;
  input.tilt_pitch_rad = msg->tilt_pitch_rad;
  input.roll_rate_rad_s = msg->roll_rate_rad_s;
  input.pitch_rate_rad_s = msg->pitch_rate_rad_s;
  estimated_state_buffer_.writeFromNonRT(input);

  if (!estimator_wait_warned_)
  {
    estimator_wait_warned_ = true;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "rb_standing_controller received first /rb/estimated_state sample");
  }
}

controller_interface::CallbackReturn RBStandingController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // configure 단계에서 parameter snapshot을 잡아둬야, activate/update 단계에서
  // 반복적인 parameter lookup 없이 RT loop가 바로 계산만 수행할 수 있다.
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  estimated_state_topic_ = get_node()->get_parameter("estimated_state_topic").as_string();
  command_interface_name_ = get_node()->get_parameter("command_interface_name").as_string();
  state_interface_names_ = get_node()->get_parameter("state_interface_names").as_string_array();
  stand_kp_ = get_node()->get_parameter("stand_kp").as_double();
  stand_kd_ = get_node()->get_parameter("stand_kd").as_double();
  stand_effort_abs_max_ = std::abs(get_node()->get_parameter("stand_effort_abs_max").as_double());
  stand_hold_current_on_start_ = get_node()->get_parameter("stand_hold_current_on_start").as_bool();
  stand_q_ref_param_ = get_node()->get_parameter("stand_q_ref").as_double_array();
  stand_q_ref_trim_param_ = get_node()->get_parameter("stand_q_ref_trim").as_double_array();
  stand_control_joints_param_ = get_node()->get_parameter("stand_control_joints").as_string_array();
  stand_pos_error_abs_max_ = std::max(0.0, get_node()->get_parameter("stand_pos_error_abs_max").as_double());
  stand_tilt_cut_enable_ = get_node()->get_parameter("stand_tilt_cut_enable").as_bool();
  stand_tilt_cut_rad_ = std::max(0.0, get_node()->get_parameter("stand_tilt_cut_rad").as_double());
  stand_tilt_recover_rad_ = std::max(0.0, get_node()->get_parameter("stand_tilt_recover_rad").as_double());
  stand_tilt_cut_output_scale_ = std::clamp(
    get_node()->get_parameter("stand_tilt_cut_output_scale").as_double(), 0.0, 1.0);
  stand_kp_scale_hip_ = std::max(0.0, get_node()->get_parameter("stand_kp_scale_hip").as_double());
  stand_kd_scale_hip_ = std::max(0.0, get_node()->get_parameter("stand_kd_scale_hip").as_double());
  stand_kp_scale_knee_ = std::max(0.0, get_node()->get_parameter("stand_kp_scale_knee").as_double());
  stand_kd_scale_knee_ = std::max(0.0, get_node()->get_parameter("stand_kd_scale_knee").as_double());
  stand_kp_scale_ankle_ = std::max(0.0, get_node()->get_parameter("stand_kp_scale_ankle").as_double());
  stand_kd_scale_ankle_ = std::max(0.0, get_node()->get_parameter("stand_kd_scale_ankle").as_double());
  stand_kp_scale_torso_ = std::max(0.0, get_node()->get_parameter("stand_kp_scale_torso").as_double());
  stand_kd_scale_torso_ = std::max(0.0, get_node()->get_parameter("stand_kd_scale_torso").as_double());
  stand_kp_scale_other_ = std::max(0.0, get_node()->get_parameter("stand_kp_scale_other").as_double());
  stand_kd_scale_other_ = std::max(0.0, get_node()->get_parameter("stand_kd_scale_other").as_double());
  limit_avoid_enable_ = get_node()->get_parameter("limit_avoid_enable").as_bool();
  limit_avoid_margin_rad_ = std::max(0.0, get_node()->get_parameter("limit_avoid_margin_rad").as_double());
  limit_avoid_kp_ = std::max(0.0, get_node()->get_parameter("limit_avoid_kp").as_double());
  limit_avoid_kd_ = std::max(0.0, get_node()->get_parameter("limit_avoid_kd").as_double());
  limit_avoid_joint_names_param_ = get_node()->get_parameter("limit_avoid_joint_names").as_string_array();
  limit_avoid_lower_param_ = get_node()->get_parameter("limit_avoid_lower").as_double_array();
  limit_avoid_upper_param_ = get_node()->get_parameter("limit_avoid_upper").as_double_array();
  enable_tilt_feedback_ = get_node()->get_parameter("enable_tilt_feedback").as_bool();
  tilt_kp_roll_ = std::max(0.0, get_node()->get_parameter("tilt_kp_roll").as_double());
  tilt_kd_roll_ = std::max(0.0, get_node()->get_parameter("tilt_kd_roll").as_double());
  tilt_kp_pitch_ = std::max(0.0, get_node()->get_parameter("tilt_kp_pitch").as_double());
  tilt_kd_pitch_ = std::max(0.0, get_node()->get_parameter("tilt_kd_pitch").as_double());
  tilt_deadband_rad_ = std::max(0.0, get_node()->get_parameter("tilt_deadband_rad").as_double());
  tilt_apply_mode_ = get_node()->get_parameter("tilt_apply_mode").as_string();
  tilt_effort_abs_max_ = std::abs(get_node()->get_parameter("tilt_effort_abs_max").as_double());
  tilt_qref_bias_abs_max_ = std::abs(get_node()->get_parameter("tilt_qref_bias_abs_max").as_double());
  tilt_roll_sign_ = get_node()->get_parameter("tilt_roll_sign").as_double();
  tilt_pitch_sign_ = get_node()->get_parameter("tilt_pitch_sign").as_double();
  tilt_weight_roll_hip_ = std::max(0.0, get_node()->get_parameter("tilt_weight_roll_hip").as_double());
  tilt_weight_roll_ankle_ = std::max(0.0, get_node()->get_parameter("tilt_weight_roll_ankle").as_double());
  tilt_weight_roll_torso_ = std::max(0.0, get_node()->get_parameter("tilt_weight_roll_torso").as_double());
  tilt_weight_pitch_hip_ = std::max(0.0, get_node()->get_parameter("tilt_weight_pitch_hip").as_double());
  tilt_weight_pitch_ankle_ = std::max(0.0, get_node()->get_parameter("tilt_weight_pitch_ankle").as_double());
  tilt_weight_pitch_knee_ = std::max(0.0, get_node()->get_parameter("tilt_weight_pitch_knee").as_double());
  tilt_weight_pitch_torso_ = std::max(0.0, get_node()->get_parameter("tilt_weight_pitch_torso").as_double());

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "rb_standing_controller: 'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (command_interface_name_.empty() || state_interface_names_.size() < 2U)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "rb_standing_controller: interface parameters are invalid");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (tilt_apply_mode_ != "effort" && tilt_apply_mode_ != "qref_bias")
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "rb_standing_controller: tilt_apply_mode must be 'effort' or 'qref_bias'");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (stand_tilt_recover_rad_ > stand_tilt_cut_rad_)
  {
    stand_tilt_recover_rad_ = stand_tilt_cut_rad_;
  }

  // subscription callback은 non-RT thread에서 돌고, update()는 RT loop에서 돈다.
  // 둘 사이 handoff는 RealtimeBuffer로 분리해 lock 없이 읽는다.
  estimated_state_sub_ = get_node()->create_subscription<rb_interfaces::msg::EstimatedState>(
    estimated_state_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&RBStandingController::on_estimated_state, this, std::placeholders::_1));
  estimated_state_buffer_.writeFromNonRT(EstimatedStateInput{});

  current_positions_.assign(joint_names_.size(), 0.0);
  current_velocities_.assign(joint_names_.size(), 0.0);
  stand_q_ref_active_.clear();
  stand_kp_scale_per_joint_.clear();
  stand_kd_scale_per_joint_.clear();
  stand_control_mask_.clear();
  limit_avoid_mask_.clear();
  limit_avoid_lower_per_joint_.clear();
  limit_avoid_upper_per_joint_.clear();

  stand_reference_ready_ = false;
  stand_ref_param_size_warned_ = false;
  stand_ref_trim_param_size_warned_ = false;
  stand_control_mask_ready_ = false;
  stand_control_unknown_warned_ = false;
  stand_gain_map_ready_ = false;
  stand_tilt_cut_active_ = false;
  limit_avoid_map_ready_ = false;
  limit_avoid_invalid_warned_ = false;
  tilt_joint_index_ready_ = false;
  tilt_joint_index_warned_ = false;
  estimator_wait_warned_ = false;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "rb_standing_controller configured: joints=%zu estimated_state_topic=%s stand_kp=%.2f stand_kd=%.2f limit=%.2f enable_tilt_feedback=%s tilt_mode=%s tilt_kp_roll=%.2f tilt_kd_roll=%.2f tilt_kp_pitch=%.2f tilt_kd_pitch=%.2f tilt_qref_bias_abs_max=%.3f",
    joint_names_.size(),
    estimated_state_topic_.c_str(),
    stand_kp_,
    stand_kd_,
    stand_effort_abs_max_,
    enable_tilt_feedback_ ? "true" : "false",
    tilt_apply_mode_.c_str(),
    tilt_kp_roll_,
    tilt_kd_roll_,
    tilt_kp_pitch_,
    tilt_kd_pitch_,
    tilt_qref_bias_abs_max_);

  return controller_interface::CallbackReturn::SUCCESS;
}

bool RBStandingController::build_interface_index_maps()
{
  // ros2_control이 건네준 flat interface 배열에서, 우리 joint 순서 기준 index table을 만든다.
  // update()에서는 이 테이블만 보고 바로 읽고 쓰도록 해 RT loop의 문자열 비교를 피한다.
  command_interface_indices_.assign(joint_names_.size(), -1);
  position_state_indices_.assign(joint_names_.size(), -1);
  velocity_state_indices_.assign(joint_names_.size(), -1);

  const auto position_it = std::find(
    state_interface_names_.begin(), state_interface_names_.end(), hardware_interface::HW_IF_POSITION);
  const auto velocity_it = std::find(
    state_interface_names_.begin(), state_interface_names_.end(), hardware_interface::HW_IF_VELOCITY);
  if (position_it == state_interface_names_.end() || velocity_it == state_interface_names_.end())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "rb_standing_controller: state_interface_names must include position and velocity");
    return false;
  }
  position_state_offset_ = static_cast<std::size_t>(std::distance(state_interface_names_.begin(), position_it));
  velocity_state_offset_ = static_cast<std::size_t>(std::distance(state_interface_names_.begin(), velocity_it));
  state_stride_ = state_interface_names_.size();

  std::unordered_map<std::string, std::size_t> joint_name_to_index;
  joint_name_to_index.reserve(joint_names_.size());
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    joint_name_to_index.emplace(joint_names_[i], i);
  }

  for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    const auto & interface_handle = command_interfaces_[i];
    if (interface_handle.get_interface_name() != command_interface_name_)
    {
      continue;
    }
    const auto it = joint_name_to_index.find(interface_handle.get_prefix_name());
    if (it != joint_name_to_index.end())
    {
      command_interface_indices_[it->second] = static_cast<int>(i);
    }
  }

  for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
  {
    const auto & interface_handle = state_interfaces_[i];
    const auto joint_it = joint_name_to_index.find(interface_handle.get_prefix_name());
    if (joint_it == joint_name_to_index.end())
    {
      continue;
    }
    if (interface_handle.get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      position_state_indices_[joint_it->second] = static_cast<int>(i);
    }
    if (interface_handle.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      velocity_state_indices_[joint_it->second] = static_cast<int>(i);
    }
  }

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (
      command_interface_indices_[i] < 0 || position_state_indices_[i] < 0 ||
      velocity_state_indices_[i] < 0)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "rb_standing_controller: interface claim incomplete for joint '%s'",
        joint_names_[i].c_str());
      return false;
    }
  }

  return true;
}

controller_interface::CallbackReturn RBStandingController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!build_interface_index_maps())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  zero_all_commands();

  // activate 직전마다 기준 자세/맵을 다시 만들 수 있게 reset한다.
  stand_reference_ready_ = false;
  stand_control_mask_ready_ = false;
  stand_gain_map_ready_ = false;
  limit_avoid_map_ready_ = false;
  tilt_joint_index_ready_ = false;
  stand_tilt_cut_active_ = false;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "rb_standing_controller activate successful: native stand_pd plugin is now driving effort interfaces");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RBStandingController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  zero_all_commands();
  return controller_interface::CallbackReturn::SUCCESS;
}

void RBStandingController::zero_all_commands()
{
  for (auto & command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }
}

bool RBStandingController::build_current_state_vectors()
{
  if (
    position_state_indices_.size() != joint_names_.size() ||
    velocity_state_indices_.size() != joint_names_.size())
  {
    return false;
  }

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    current_positions_[i] = state_interfaces_[static_cast<std::size_t>(position_state_indices_[i])].get_value();
    current_velocities_[i] = state_interfaces_[static_cast<std::size_t>(velocity_state_indices_[i])].get_value();
  }
  return true;
}

bool RBStandingController::ensure_stand_reference_ready()
{
  if (stand_reference_ready_)
  {
    return true;
  }

  const auto result = build_stand_reference(
    joint_names_,
    current_positions_,
    stand_q_ref_param_,
    stand_q_ref_trim_param_,
    stand_hold_current_on_start_);

  if (result.q_ref_size_mismatch && !stand_ref_param_size_warned_)
  {
    stand_ref_param_size_warned_ = true;
    RCLCPP_WARN(
      get_node()->get_logger(),
      "stand_q_ref size mismatch: ref=%zu joint_count=%zu. fallback to current pose=%s",
      stand_q_ref_param_.size(),
      joint_names_.size(),
      stand_hold_current_on_start_ ? "enabled" : "disabled");
  }
  if (result.trim_size_mismatch && !stand_ref_trim_param_size_warned_)
  {
    stand_ref_trim_param_size_warned_ = true;
    RCLCPP_WARN(
      get_node()->get_logger(),
      "stand_q_ref_trim size mismatch: trim=%zu joint_count=%zu. trim is ignored",
      stand_q_ref_trim_param_.size(),
      joint_names_.size());
  }
  if (!result.ready)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      3000,
      "native stand_pd reference is not ready");
    return false;
  }

  stand_q_ref_active_ = result.values;
  stand_reference_ready_ = true;
  RCLCPP_INFO(
    get_node()->get_logger(),
    "native stand_pd reference ready: source=%s count=%zu",
    result.used_current_pose ? "current_pose" : "parameter",
    stand_q_ref_active_.size());
  return true;
}

bool RBStandingController::ensure_stand_control_mask_ready()
{
  if (stand_control_mask_ready_)
  {
    return true;
  }

  const auto result = build_stand_control_mask(joint_names_, stand_control_joints_param_);
  if (!result.ready)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      3000,
      "stand_control_joints has no match with current joint names");
    return false;
  }

  stand_control_mask_ = result.mask;
  stand_control_mask_ready_ = true;
  if (!result.unknown_names.empty() && !stand_control_unknown_warned_)
  {
    stand_control_unknown_warned_ = true;
    RCLCPP_WARN(
      get_node()->get_logger(),
      "stand_control_joints contains %zu unknown names (ignored)",
      result.unknown_names.size());
  }
  return true;
}

bool RBStandingController::ensure_stand_gain_map_ready()
{
  if (stand_gain_map_ready_)
  {
    return true;
  }

  const auto result = build_stand_gain_map(
    joint_names_,
    stand_kp_scale_hip_,
    stand_kp_scale_knee_,
    stand_kp_scale_ankle_,
    stand_kp_scale_torso_,
    stand_kp_scale_other_,
    stand_kd_scale_hip_,
    stand_kd_scale_knee_,
    stand_kd_scale_ankle_,
    stand_kd_scale_torso_,
    stand_kd_scale_other_);
  if (!result.ready)
  {
    return false;
  }

  stand_kp_scale_per_joint_ = result.kp_scales;
  stand_kd_scale_per_joint_ = result.kd_scales;
  stand_gain_map_ready_ = true;
  return true;
}

bool RBStandingController::ensure_limit_avoid_map_ready()
{
  if (!limit_avoid_enable_)
  {
    return true;
  }
  if (limit_avoid_map_ready_)
  {
    return true;
  }

  const auto result = build_limit_avoid_map(
    joint_names_,
    limit_avoid_joint_names_param_,
    limit_avoid_lower_param_,
    limit_avoid_upper_param_);
  if (result.invalid_table)
  {
    if (!limit_avoid_invalid_warned_)
    {
      limit_avoid_invalid_warned_ = true;
      RCLCPP_WARN(
        get_node()->get_logger(),
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
        get_node()->get_logger(),
        "limit_avoid table has no joint match with current joint names");
    }
    return false;
  }

  limit_avoid_mask_ = result.mask;
  limit_avoid_lower_per_joint_ = result.lower_per_joint;
  limit_avoid_upper_per_joint_ = result.upper_per_joint;
  limit_avoid_map_ready_ = result.ready;
  return limit_avoid_map_ready_;
}

bool RBStandingController::ensure_tilt_joint_index_ready()
{
  if (tilt_joint_index_ready_)
  {
    return true;
  }

  const auto indices = build_tilt_joint_indices(joint_names_);
  if (!indices.ready())
  {
    if (!tilt_joint_index_warned_)
    {
      tilt_joint_index_warned_ = true;
      RCLCPP_WARN(
        get_node()->get_logger(),
        "tilt feedback joint indices are not available in current joint names");
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
  return true;
}

double RBStandingController::compute_stand_output_scale(const EstimatedStateInput & estimated_state)
{
  // tilt cut은 "이미 크게 넘어지는 중이면 stand_pd 출력을 줄여 더 위험한 힘을 덜 주는" 안전 장치다.
  if (!stand_tilt_cut_enable_)
  {
    stand_tilt_cut_active_ = false;
    return 1.0;
  }

  if (!estimated_state.received || !estimated_state.imu_valid)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      3000,
      "stand_tilt_cut enabled but /rb/estimated_state IMU is not ready");
    return 1.0;
  }

  const double tilt_abs = std::max(
    std::abs(estimated_state.tilt_roll_rad), std::abs(estimated_state.tilt_pitch_rad));
  if (!stand_tilt_cut_active_)
  {
    if (tilt_abs >= stand_tilt_cut_rad_)
    {
      stand_tilt_cut_active_ = true;
      RCLCPP_WARN(
        get_node()->get_logger(),
        "native stand_tilt_cut active: tilt=%.3f cut=%.3f recover=%.3f scale=%.2f",
        tilt_abs,
        stand_tilt_cut_rad_,
        stand_tilt_recover_rad_,
        stand_tilt_cut_output_scale_);
    }
  }
  else if (tilt_abs <= stand_tilt_recover_rad_)
  {
    stand_tilt_cut_active_ = false;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "native stand_tilt_cut cleared: tilt=%.3f",
      tilt_abs);
  }

  return stand_tilt_cut_active_ ? stand_tilt_cut_output_scale_ : 1.0;
}

bool RBStandingController::compute_tilt_feedback_command(
  const EstimatedStateInput & estimated_state, TiltFeedbackCommand & cmd) const
{
  // tilt feedback은 roll/pitch 복원 명령을 한 번 계산한 뒤,
  // 이를 additive effort로 쓸지 q_ref bias로 쓸지는 update() 마지막 단계에서 결정한다.
  if (!enable_tilt_feedback_ || !estimated_state.received || !estimated_state.imu_valid)
  {
    return false;
  }

  double roll = estimated_state.tilt_roll_rad;
  double pitch = estimated_state.tilt_pitch_rad;
  if (std::abs(roll) < tilt_deadband_rad_)
  {
    roll = 0.0;
  }
  if (std::abs(pitch) < tilt_deadband_rad_)
  {
    pitch = 0.0;
  }

  const double u_roll_raw =
    tilt_roll_sign_ * ((tilt_kp_roll_ * (-roll)) + (tilt_kd_roll_ * (-estimated_state.roll_rate_rad_s)));
  const double u_pitch_raw = tilt_pitch_sign_ *
    ((tilt_kp_pitch_ * (-pitch)) + (tilt_kd_pitch_ * (-estimated_state.pitch_rate_rad_s)));

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
  else if (tilt_qref_bias_abs_max_ > 0.0)
  {
    u_roll = std::clamp(u_roll, -tilt_qref_bias_abs_max_, tilt_qref_bias_abs_max_);
    u_pitch = std::clamp(u_pitch, -tilt_qref_bias_abs_max_, tilt_qref_bias_abs_max_);
  }

  const double roll_sum = tilt_weight_roll_hip_ + tilt_weight_roll_ankle_ + tilt_weight_roll_torso_;
  const double pitch_sum =
    tilt_weight_pitch_hip_ + tilt_weight_pitch_ankle_ + tilt_weight_pitch_knee_ + tilt_weight_pitch_torso_;
  const double roll_norm = (roll_sum > 1e-9) ? (1.0 / roll_sum) : 0.0;
  const double pitch_norm = (pitch_sum > 1e-9) ? (1.0 / pitch_sum) : 0.0;

  cmd.valid = true;
  cmd.u_roll = u_roll;
  cmd.u_pitch = u_pitch;
  cmd.roll_w_hip = tilt_weight_roll_hip_ * roll_norm;
  cmd.roll_w_ankle = tilt_weight_roll_ankle_ * roll_norm;
  cmd.roll_w_torso = tilt_weight_roll_torso_ * roll_norm;
  cmd.pitch_w_hip = tilt_weight_pitch_hip_ * pitch_norm;
  cmd.pitch_w_ankle = tilt_weight_pitch_ankle_ * pitch_norm;
  cmd.pitch_w_knee = tilt_weight_pitch_knee_ * pitch_norm;
  cmd.pitch_w_torso = tilt_weight_pitch_torso_ * pitch_norm;
  return true;
}

void RBStandingController::apply_tilt_feedback_effort(
  const TiltFeedbackCommand & tilt_cmd, std::size_t usable_count)
{
  // effort mode는 기본 stand_pd effort를 만든 뒤 마지막에 additive effort만 더한다.
  const auto add_effort = [this, usable_count](int idx, double delta)
    {
      if (idx < 0)
      {
        return;
      }
      const std::size_t joint_idx = static_cast<std::size_t>(idx);
      if (joint_idx >= usable_count || stand_control_mask_[joint_idx] == 0U)
      {
        return;
      }
      auto & command_interface =
        command_interfaces_[static_cast<std::size_t>(command_interface_indices_[joint_idx])];
      double value = command_interface.get_value() + delta;
      if (stand_effort_abs_max_ > 0.0)
      {
        value = std::clamp(value, -stand_effort_abs_max_, stand_effort_abs_max_);
      }
      command_interface.set_value(value);
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

void RBStandingController::apply_tilt_feedback_qref_bias(
  std::vector<double> & effective_q_ref, const TiltFeedbackCommand & tilt_cmd) const
{
  // q_ref_bias mode는 effort를 직접 건드리지 않고 목표 자세를 조금 기울여 같은 복원 효과를 만든다.
  const auto add_bias = [this, &effective_q_ref](int idx, double delta)
    {
      if (idx < 0)
      {
        return;
      }
      const std::size_t joint_idx = static_cast<std::size_t>(idx);
      if (joint_idx >= effective_q_ref.size() || stand_control_mask_[joint_idx] == 0U)
      {
        return;
      }
      effective_q_ref[joint_idx] += delta;
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

double RBStandingController::compute_limit_avoid_effort(std::size_t idx, double q, double dq) const
{
  // hard limit 직전에 soft guard band를 두고, joint가 guard band 안으로 들어오면
  // 반대방향 복원 effort를 더한다.
  if (!limit_avoid_enable_ || !limit_avoid_map_ready_)
  {
    return 0.0;
  }
  if (idx >= limit_avoid_mask_.size() || limit_avoid_mask_[idx] == 0U)
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
  if (q < lo_guard)
  {
    const double pos_err = lo_guard - q;
    const double vel_err = std::max(0.0, -dq);
    u += (limit_avoid_kp_ * pos_err) + (limit_avoid_kd_ * vel_err);
  }
  if (q > hi_guard)
  {
    const double pos_err = q - hi_guard;
    const double vel_err = std::max(0.0, dq);
    u -= (limit_avoid_kp_ * pos_err) + (limit_avoid_kd_ * vel_err);
  }
  return u;
}

controller_interface::return_type RBStandingController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // native controller loop:
  // 1) state interface에서 현재 joint 상태를 읽고
  // 2) stand reference / gain / safety cache를 확인한 뒤
  // 3) stand_pd + limit avoid + tilt feedback을 합쳐
  // 4) command interface에 최종 effort를 직접 쓴다.
  if (!build_current_state_vectors())
  {
    zero_all_commands();
    return controller_interface::return_type::ERROR;
  }

  zero_all_commands();

  if (!ensure_stand_reference_ready() || !ensure_stand_control_mask_ready() ||
    !ensure_stand_gain_map_ready() || !ensure_limit_avoid_map_ready() ||
    !ensure_tilt_joint_index_ready())
  {
    return controller_interface::return_type::OK;
  }

  const EstimatedStateInput estimated_state = *estimated_state_buffer_.readFromRT();
  const double stand_output_scale = compute_stand_output_scale(estimated_state);
  // effective_q_ref는 "이번 tick에서 실제로 쓸 목표 자세"다.
  // qref_bias tilt feedback은 원본 stand_q_ref를 덮어쓰지 않고 이 복사본에만 얹는다.
  std::vector<double> effective_q_ref = stand_q_ref_active_;

  TiltFeedbackCommand tilt_cmd;
  const bool has_tilt_cmd = compute_tilt_feedback_command(estimated_state, tilt_cmd);
  if (has_tilt_cmd && tilt_apply_mode_ == "qref_bias")
  {
    // qref_bias 모드는 effort를 바로 더하지 않고 목표 자세를 조금 움직여 복원 방향을 만든다.
    apply_tilt_feedback_qref_bias(effective_q_ref, tilt_cmd);
  }

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (stand_control_mask_[i] == 0U)
    {
      continue;
    }

    double position_error = effective_q_ref[i] - current_positions_[i];
    if (stand_pos_error_abs_max_ > 0.0)
    {
      position_error = std::clamp(position_error, -stand_pos_error_abs_max_, stand_pos_error_abs_max_);
    }
    const double damping_term = -current_velocities_[i];
    const double kp_eff = stand_kp_ * stand_kp_scale_per_joint_[i];
    const double kd_eff = stand_kd_ * stand_kd_scale_per_joint_[i];
    const double effort_pd = (kp_eff * position_error) + (kd_eff * damping_term);
    const double effort_limit =
      compute_limit_avoid_effort(i, current_positions_[i], current_velocities_[i]);
    double effort_cmd = (effort_pd + effort_limit) * stand_output_scale;
    if (stand_effort_abs_max_ > 0.0)
    {
      effort_cmd = std::clamp(effort_cmd, -stand_effort_abs_max_, stand_effort_abs_max_);
    }
    command_interfaces_[static_cast<std::size_t>(command_interface_indices_[i])].set_value(effort_cmd);
  }

  if (has_tilt_cmd && tilt_apply_mode_ == "effort")
  {
    // effort 모드는 기본 PD 결과를 만든 뒤 마지막에 additive effort로 얹는다.
    apply_tilt_feedback_effort(tilt_cmd, joint_names_.size());
  }

  return controller_interface::return_type::OK;
}

}  // namespace rb_standing_controller

PLUGINLIB_EXPORT_CLASS(
  rb_standing_controller::RBStandingController,
  controller_interface::ControllerInterface)
