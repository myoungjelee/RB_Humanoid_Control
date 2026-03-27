#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// 기존 controller/safety가 만드는 JointState effort 명령을
// ros2_control forward_command_controller 입력(Float64MultiArray)으로 바꿔주는 어댑터다.
class RbCommandBridgeNode : public rclcpp::Node
{
public:
  RbCommandBridgeNode()
  : Node("rb_command_bridge")
  {
    input_command_topic_ = this->declare_parameter<std::string>(
      "input_command_topic", "/rb/command_safe_source");
    output_command_topic_ = this->declare_parameter<std::string>(
      "output_command_topic", "/rb_effort_forward_controller/commands");
    target_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "target_joint_names", std::vector<std::string>{});
    command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_command_topic_, 10,
      std::bind(&RbCommandBridgeNode::on_command, this, std::placeholders::_1));
    command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      output_command_topic_, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "rb_command_bridge started: in=%s out=%s target_joints=%zu",
      input_command_topic_.c_str(), output_command_topic_.c_str(), target_joint_names_.size());
  }

private:
  void on_command(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->name.empty())
    {
      return;
    }

    // target_joint_names를 비워두면 입력 메시지 순서를 그대로 사용한다.
    if (target_joint_names_.empty())
    {
      target_joint_names_ = msg->name;
      RCLCPP_INFO(
        this->get_logger(),
        "target_joint_names was empty. Using incoming command order (%zu joints).",
        target_joint_names_.size());
    }

    std::unordered_map<std::string, double> effort_by_name;
    effort_by_name.reserve(msg->name.size());
    const std::size_t effort_count = std::min(msg->name.size(), msg->effort.size());
    for (std::size_t index = 0; index < effort_count; ++index)
    {
      effort_by_name[msg->name[index]] = msg->effort[index];
    }

    std_msgs::msg::Float64MultiArray out;
    out.data.assign(target_joint_names_.size(), 0.0);
    std::size_t mapped_count = 0;
    for (std::size_t index = 0; index < target_joint_names_.size(); ++index)
    {
      const auto & joint_name = target_joint_names_[index];
      const auto it = effort_by_name.find(joint_name);
      if (it == effort_by_name.end())
      {
        continue;
      }
      out.data[index] = it->second;
      ++mapped_count;
    }

    if (!mapping_logged_)
    {
      mapping_logged_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "first command mapped: source_names=%zu source_effort=%zu mapped=%zu target=%zu",
        msg->name.size(), msg->effort.size(), mapped_count, target_joint_names_.size());
    }

    command_pub_->publish(out);
  }

  std::string input_command_topic_{"/rb/command_safe_source"};
  std::string output_command_topic_{"/rb_effort_forward_controller/commands"};
  std::vector<std::string> target_joint_names_{};
  bool mapping_logged_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RbCommandBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
