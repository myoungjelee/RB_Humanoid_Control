#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "controller_tilt_observer.hpp"
#include "rb_controller/msg/estimated_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace rbci = rb_controller::internal;

namespace
{

std::string resolve_imu_frame_mode(const std::string &mode)
{
  if (mode == "identity" || mode == "standard")
  {
    return "identity";
  }
  if (mode == "g1_imu_link" || mode == "swap_rp")
  {
    return "g1_imu_link";
  }
  return "";
}

}  // namespace

class RbEstimatorNode : public rclcpp::Node
{
public:
  RbEstimatorNode()
      : Node("rb_estimator")
  {
    input_joint_states_topic_ =
        this->declare_parameter<std::string>("input_joint_states_topic", "/rb/joint_states");
    input_imu_topic_ =
        this->declare_parameter<std::string>("input_imu_topic", "/rb/imu");
    output_estimated_state_topic_ =
        this->declare_parameter<std::string>("output_estimated_state_topic", "/rb/estimated_state");
    imu_zero_on_start_ = this->declare_parameter<bool>("imu_zero_on_start", false);
    imu_frame_mode_ = this->declare_parameter<std::string>("imu_frame_mode", "identity");
    const auto legacy_tilt_axis_mode =
        this->declare_parameter<std::string>("tilt_axis_mode", "");

    const std::string resolved_imu_frame_mode = resolve_imu_frame_mode(imu_frame_mode_);
    if (resolved_imu_frame_mode.empty())
    {
      RCLCPP_WARN(
          this->get_logger(),
          "unknown imu_frame_mode '%s'. fallback to identity",
          imu_frame_mode_.c_str());
      imu_frame_mode_ = "identity";
    }
    else
    {
      imu_frame_mode_ = resolved_imu_frame_mode;
    }

    if (!legacy_tilt_axis_mode.empty())
    {
      const std::string resolved_legacy_mode = resolve_imu_frame_mode(legacy_tilt_axis_mode);
      if (resolved_legacy_mode.empty())
      {
        RCLCPP_WARN(
            this->get_logger(),
            "deprecated tilt_axis_mode '%s' is unknown. ignore and keep imu_frame_mode=%s",
            legacy_tilt_axis_mode.c_str(), imu_frame_mode_.c_str());
      }
      else if (imu_frame_mode_ == "identity")
      {
        RCLCPP_WARN(
            this->get_logger(),
            "deprecated tilt_axis_mode is set. use imu_frame_mode instead. mapped '%s' -> '%s'",
            legacy_tilt_axis_mode.c_str(), resolved_legacy_mode.c_str());
        imu_frame_mode_ = resolved_legacy_mode;
      }
      else if (resolved_legacy_mode != imu_frame_mode_)
      {
        RCLCPP_WARN(
            this->get_logger(),
            "imu_frame_mode=%s and deprecated tilt_axis_mode=%s disagree. prefer imu_frame_mode",
            imu_frame_mode_.c_str(), legacy_tilt_axis_mode.c_str());
      }
    }

    tilt_observer_.set_zero_on_start(imu_zero_on_start_);
    tilt_observer_.set_frame_mode(imu_frame_mode_);

    estimated_state_pub_ = this->create_publisher<rb_controller::msg::EstimatedState>(
        output_estimated_state_topic_, 20);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        input_joint_states_topic_, 20,
        std::bind(&RbEstimatorNode::on_joint_state, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        input_imu_topic_, 20,
        std::bind(&RbEstimatorNode::on_imu, this, std::placeholders::_1));

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&RbEstimatorNode::on_parameters_changed, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "rb_estimator started: joint_states=%s imu=%s out=%s imu_frame_mode=%s imu_zero_on_start=%s",
        input_joint_states_topic_.c_str(), input_imu_topic_.c_str(),
        output_estimated_state_topic_.c_str(), imu_frame_mode_.c_str(),
        imu_zero_on_start_ ? "on" : "off");
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->name.empty())
    {
      return;
    }

    if (joint_names_ != msg->name)
    {
      joint_names_ = msg->name;
      RCLCPP_INFO(
          this->get_logger(),
          "joint_names updated from /rb/joint_states: count=%zu", joint_names_.size());
    }

    latest_joint_positions_.assign(joint_names_.size(), 0.0);
    const std::size_t pos_count = std::min(joint_names_.size(), msg->position.size());
    for (std::size_t i = 0; i < pos_count; ++i)
    {
      latest_joint_positions_[i] = msg->position[i];
    }

    latest_joint_velocities_.assign(joint_names_.size(), 0.0);
    const std::size_t vel_count = std::min(joint_names_.size(), msg->velocity.size());
    for (std::size_t i = 0; i < vel_count; ++i)
    {
      latest_joint_velocities_[i] = msg->velocity[i];
    }

    joint_state_received_ = true;
    publish_estimated_state(msg->header.stamp);
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    const auto update = tilt_observer_.update_from_imu(*msg);
    if (update.bias_captured_now)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "imu bias captured: raw_roll=%.3f raw_pitch=%.3f bias_roll=%.3f bias_pitch=%.3f",
          update.raw_roll_rad, update.raw_pitch_rad, update.bias_roll_rad, update.bias_pitch_rad);
    }

    publish_estimated_state(msg->header.stamp);
  }

  void publish_estimated_state(const builtin_interfaces::msg::Time &stamp)
  {
    if (!joint_state_received_ || !tilt_observer_.received())
    {
      return;
    }

    rb_controller::msg::EstimatedState msg;
    msg.header.stamp = stamp;
    msg.joint_names = joint_names_;
    msg.joint_positions = latest_joint_positions_;
    msg.joint_velocities = latest_joint_velocities_;
    msg.raw_roll_rad = tilt_observer_.raw_roll_rad();
    msg.raw_pitch_rad = tilt_observer_.raw_pitch_rad();
    msg.bias_roll_rad = tilt_observer_.bias_roll_rad();
    msg.bias_pitch_rad = tilt_observer_.bias_pitch_rad();
    msg.tilt_roll_rad = tilt_observer_.tilt_roll_rad();
    msg.tilt_pitch_rad = tilt_observer_.tilt_pitch_rad();
    msg.roll_rate_rad_s = tilt_observer_.roll_rate_rad_s();
    msg.pitch_rate_rad_s = tilt_observer_.pitch_rate_rad_s();
    msg.imu_frame_mode = imu_frame_mode_;
    estimated_state_pub_->publish(msg);
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
      const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";

    for (const auto &param : params)
    {
      const std::string &name = param.get_name();

      if (name == "input_joint_states_topic" || name == "input_imu_topic" ||
          name == "output_estimated_state_topic")
      {
        result.successful = false;
        result.reason = "runtime update not supported for topic parameters (restart required)";
        return result;
      }
      if (name == "imu_zero_on_start")
      {
        imu_zero_on_start_ = param.as_bool();
        tilt_observer_.set_zero_on_start(imu_zero_on_start_);
        tilt_observer_.reset_bias();
        continue;
      }
      if (name == "imu_frame_mode" || name == "tilt_axis_mode")
      {
        const std::string requested = param.as_string();
        const std::string resolved = resolve_imu_frame_mode(requested);
        if (resolved.empty())
        {
          result.successful = false;
          result.reason = "imu_frame_mode must be 'identity' or 'g1_imu_link' (legacy: standard/swap_rp)";
          return result;
        }
        if (name == "tilt_axis_mode")
        {
          RCLCPP_WARN(
              this->get_logger(),
              "runtime parameter tilt_axis_mode is deprecated. use imu_frame_mode instead.");
        }
        imu_frame_mode_ = resolved;
        tilt_observer_.set_frame_mode(imu_frame_mode_);
        continue;
      }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "runtime estimator parameters updated: imu_frame_mode=%s imu_zero_on_start=%s",
        imu_frame_mode_.c_str(), imu_zero_on_start_ ? "on" : "off");
    return result;
  }

  std::string input_joint_states_topic_{"/rb/joint_states"};
  std::string input_imu_topic_{"/rb/imu"};
  std::string output_estimated_state_topic_{"/rb/estimated_state"};
  bool imu_zero_on_start_{false};
  std::string imu_frame_mode_{"identity"};

  std::vector<std::string> joint_names_;
  std::vector<double> latest_joint_positions_;
  std::vector<double> latest_joint_velocities_;
  bool joint_state_received_{false};

  rbci::TiltObserver tilt_observer_{};

  rclcpp::Publisher<rb_controller::msg::EstimatedState>::SharedPtr estimated_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RbEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
