#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "controller_tilt_observer.hpp"
#include "rb_controller/msg/estimated_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// 내부 helper/observer 타입 이름이 길어서 cpp 내부에서만 짧게 쓴다.
namespace rbci = rb_controller::internal;

namespace
{

// ROS Time(sec/nanosec)를 stale 판정에 쓰기 쉬운 정수 ns로 변환한다.
std::int64_t time_to_nanoseconds(const builtin_interfaces::msg::Time &stamp)
{
  return (static_cast<std::int64_t>(stamp.sec) * 1000000000LL) +
         static_cast<std::int64_t>(stamp.nanosec);
}

}  // namespace

class RbEstimatorNode : public rclcpp::Node
{
public:
  RbEstimatorNode()
      : Node("rb_estimator")
  {
    // estimator는 raw sensor를 직접 제어에 쓰지 않고, /rb/estimated_state 계약으로 한 번 정리해 넘긴다.
    input_joint_states_topic_ =
        this->declare_parameter<std::string>("input_joint_states_topic", "/rb/joint_states");
    input_imu_topic_ =
        this->declare_parameter<std::string>("input_imu_topic", "/rb/imu");
    output_estimated_state_topic_ =
        this->declare_parameter<std::string>("output_estimated_state_topic", "/rb/estimated_state");
    input_stale_timeout_sec_ = this->declare_parameter<double>("input_stale_timeout_sec", 0.10);
    if (input_stale_timeout_sec_ < 0.0)
    {
      input_stale_timeout_sec_ = 0.0;
    }

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
        "rb_estimator started: joint_states=%s imu=%s out=%s g1_frame_comp=on stale_timeout=%.3fs",
        input_joint_states_topic_.c_str(), input_imu_topic_.c_str(),
        output_estimated_state_topic_.c_str(), input_stale_timeout_sec_);
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->name.empty())
    {
      return;
    }

    // 조인트 이름 순서가 바뀌면 이후 controller/safety가 같은 순서로 읽을 수 있게 캐시를 갱신한다.
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

    // estimator는 joint와 IMU 둘 중 어느 콜백에서든 최신 통합 상태를 다시 publish한다.
    joint_state_received_ = true;
    latest_joint_state_stamp_ = msg->header.stamp;
    publish_estimated_state(msg->header.stamp);
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    // IMU raw는 observer 내부에서 G1 기준 tilt/rate로 변환한다.
    imu_received_ = true;
    latest_imu_stamp_ = msg->header.stamp;
    tilt_observer_.update_from_imu(*msg);
    publish_estimated_state(msg->header.stamp);
  }

  void publish_estimated_state(const builtin_interfaces::msg::Time &stamp)
  {
    // 최소한 joint + imu + observer 출력이 모두 준비되어야 제어용 상태 계약을 publish한다.
    if (!joint_state_received_ || !imu_received_ || !tilt_observer_.received())
    {
      return;
    }

    rb_controller::msg::EstimatedState msg;
    msg.header.stamp = stamp;
    // fused state에서도 원본 source stamp를 따로 남겨 consumer가 freshness를 직접 판단할 수 있게 한다.
    msg.joint_state_stamp = latest_joint_state_stamp_;
    msg.imu_stamp = latest_imu_stamp_;
    // stale_timeout을 넘긴 입력은 valid=false로 내려 controller/safety가 그대로 쓰지 않게 한다.
    msg.joint_state_valid = is_source_valid(stamp, latest_joint_state_stamp_);
    msg.imu_valid = is_source_valid(stamp, latest_imu_stamp_);
    msg.joint_names = joint_names_;
    msg.joint_positions = latest_joint_positions_;
    msg.joint_velocities = latest_joint_velocities_;
    msg.raw_roll_rad = tilt_observer_.raw_roll_rad();
    msg.raw_pitch_rad = tilt_observer_.raw_pitch_rad();
    msg.tilt_roll_rad = tilt_observer_.tilt_roll_rad();
    msg.tilt_pitch_rad = tilt_observer_.tilt_pitch_rad();
    msg.roll_rate_rad_s = tilt_observer_.roll_rate_rad_s();
    msg.pitch_rate_rad_s = tilt_observer_.pitch_rate_rad_s();
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
      if (name == "input_stale_timeout_sec")
      {
        const double value = param.as_double();
        if (value < 0.0)
        {
          result.successful = false;
          result.reason = "input_stale_timeout_sec must be >= 0";
          return result;
        }
        input_stale_timeout_sec_ = value;
        continue;
      }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "runtime estimator parameters updated: g1_frame_comp=on stale_timeout=%.3fs",
        input_stale_timeout_sec_);
    return result;
  }

  bool is_source_valid(
      const builtin_interfaces::msg::Time &reference_stamp,
      const builtin_interfaces::msg::Time &source_stamp) const
  {
    // timeout이 0 이하이면 stale 판정을 끄고 항상 valid로 본다.
    if (input_stale_timeout_sec_ <= 0.0)
    {
      return true;
    }
    const std::int64_t delta_ns =
        std::llabs(time_to_nanoseconds(reference_stamp) - time_to_nanoseconds(source_stamp));
    const double delta_sec = static_cast<double>(delta_ns) * 1e-9;
    return delta_sec <= input_stale_timeout_sec_;
  }

  std::string input_joint_states_topic_{"/rb/joint_states"};
  std::string input_imu_topic_{"/rb/imu"};
  std::string output_estimated_state_topic_{"/rb/estimated_state"};
  double input_stale_timeout_sec_{0.10};

  std::vector<std::string> joint_names_;
  std::vector<double> latest_joint_positions_;
  std::vector<double> latest_joint_velocities_;
  bool joint_state_received_{false};
  bool imu_received_{false};
  builtin_interfaces::msg::Time latest_joint_state_stamp_{};
  builtin_interfaces::msg::Time latest_imu_stamp_{};

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
