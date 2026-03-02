#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class RbControllerNode : public rclcpp::Node
{
public:
  RbControllerNode()
      : Node("rb_controller"),
        steady_prev_tick_(std::chrono::steady_clock::time_point::min()),
        steady_last_log_(std::chrono::steady_clock::now())
  {
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 200.0);
    log_interval_sec_ = this->declare_parameter<double>("log_interval_sec", 5.0);
    miss_ratio_threshold_ =
        this->declare_parameter<double>("miss_ratio_threshold", 1.2);
    input_topic_ =
        this->declare_parameter<std::string>("input_joint_states_topic", "/rb/joint_states");
    output_topic_ =
        this->declare_parameter<std::string>("output_command_raw_topic", "/rb/command_raw");
    joint_names_ =
        this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});

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

    expected_dt_sec_ = 1.0 / control_rate_hz_;

    command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output_topic_, 10);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        input_topic_, 10,
        std::bind(&RbControllerNode::on_joint_state, this, std::placeholders::_1));

    using namespace std::chrono;
    const auto timer_period = duration_cast<nanoseconds>(duration<double>(expected_dt_sec_));
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&RbControllerNode::on_control_tick, this));

    RCLCPP_INFO(
        this->get_logger(),
        "rb_controller started: rate=%.1fHz expected_dt=%.6fs input=%s output=%s joint_names(init)=%zu",
        control_rate_hz_, expected_dt_sec_, input_topic_.c_str(), output_topic_.c_str(),
        joint_names_.size());
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
  }

  void on_control_tick()
  {
    const auto now_steady = std::chrono::steady_clock::now();

    if (steady_prev_tick_ != std::chrono::steady_clock::time_point::min())
    {
      const double dt_sec = std::chrono::duration<double>(now_steady - steady_prev_tick_).count();
      dt_samples_sec_.push_back(dt_sec);
      if (dt_sec > expected_dt_sec_ * miss_ratio_threshold_)
      {
        ++miss_count_total_;
        ++miss_count_window_;
      }
    }
    steady_prev_tick_ = now_steady;

    sensor_msgs::msg::JointState cmd_msg;
    cmd_msg.header.stamp = this->now();
    cmd_msg.name = joint_names_;

    const std::size_t joint_count = joint_names_.size();
    cmd_msg.position.assign(joint_count, 0.0);
    cmd_msg.velocity.assign(joint_count, 0.0);
    cmd_msg.effort.assign(joint_count, 0.0);

    if (joint_count == 0U)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "joint_names is empty. Waiting for /rb/joint_states names before meaningful command publication.");
    }

    command_pub_->publish(cmd_msg);

    const double log_elapsed = std::chrono::duration<double>(now_steady - steady_last_log_).count();
    if (log_elapsed >= log_interval_sec_)
    {
      log_timing_stats();
      steady_last_log_ = now_steady;
      dt_samples_sec_.clear();
      miss_count_window_ = 0;
    }
  }

  void log_timing_stats()
  {
    if (dt_samples_sec_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "timing window has no samples yet");
      return;
    }

    const double sum = std::accumulate(dt_samples_sec_.begin(), dt_samples_sec_.end(), 0.0);
    const double mean = sum / static_cast<double>(dt_samples_sec_.size());
    const double max_dt = *std::max_element(dt_samples_sec_.begin(), dt_samples_sec_.end());
    const double p95 = percentile(dt_samples_sec_, 0.95);

    RCLCPP_INFO(
        this->get_logger(),
        "loop_stats dt_mean=%.6f dt_max=%.6f dt_p95=%.6f miss_window=%zu miss_total=%zu samples=%zu",
        mean, max_dt, p95, miss_count_window_, miss_count_total_, dt_samples_sec_.size());
  }

  static double percentile(const std::vector<double> &values, double ratio)
  {
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

  double control_rate_hz_{200.0};
  double log_interval_sec_{5.0};
  double miss_ratio_threshold_{1.2};
  double expected_dt_sec_{0.005};
  std::string input_topic_;
  std::string output_topic_;

  std::vector<std::string> joint_names_;
  std::vector<double> dt_samples_sec_;

  std::size_t miss_count_total_{0};
  std::size_t miss_count_window_{0};

  std::chrono::steady_clock::time_point steady_prev_tick_;
  std::chrono::steady_clock::time_point steady_last_log_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RbControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
