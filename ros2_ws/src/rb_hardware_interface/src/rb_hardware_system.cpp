#include "rb_hardware_interface/rb_hardware_system.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

namespace rb_hardware_interface
{

namespace
{

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const hardware_interface::InterfaceInfo & interface_info)
    {
      return interface_info.name == name;
    });
}

}  // namespace

CallbackReturn RBHardwareSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joint_names_.clear();
  position_states_.clear();
  velocity_states_.clear();
  effort_states_.clear();
  effort_commands_.clear();

  joint_names_.reserve(info_.joints.size());
  for (std::size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index)
  {
    const auto & joint = info_.joints[joint_index];
    // Step 1 skeleton은 effort command + pos/vel/effort state만 지원한다.
    if (!has_interface(joint.command_interfaces, hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("rb_hardware_interface"),
        "joint '%s' is missing effort command interface",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (!has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
        !has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY) ||
        !has_interface(joint.state_interfaces, hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("rb_hardware_interface"),
        "joint '%s' must expose position/velocity/effort state interfaces",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
    joint_name_to_index_[joint.name] = joint_index;
  }

  const auto joint_state_topic_it = info_.hardware_parameters.find("joint_state_topic");
  if (joint_state_topic_it != info_.hardware_parameters.end() && !joint_state_topic_it->second.empty())
  {
    joint_state_topic_ = joint_state_topic_it->second;
  }
  const auto command_topic_it = info_.hardware_parameters.find("command_topic");
  if (command_topic_it != info_.hardware_parameters.end() && !command_topic_it->second.empty())
  {
    command_topic_ = command_topic_it->second;
  }
  const auto bridge_enabled_it = info_.hardware_parameters.find("bridge_enabled");
  if (bridge_enabled_it != info_.hardware_parameters.end())
  {
    bridge_enabled_ =
      (bridge_enabled_it->second == "true" || bridge_enabled_it->second == "1");
  }

  position_states_.assign(joint_names_.size(), 0.0);
  velocity_states_.assign(joint_names_.size(), 0.0);
  effort_states_.assign(joint_names_.size(), 0.0);
  effort_commands_.assign(joint_names_.size(), 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("rb_hardware_interface"),
    "RBHardwareSystem initialized: joints=%zu command=effort states=position/velocity/effort bridge=%s js_topic=%s cmd_topic=%s",
    joint_names_.size(), bridge_enabled_ ? "on" : "off", joint_state_topic_.c_str(),
    command_topic_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn RBHardwareSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!bridge_enabled_)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("rb_hardware_interface"),
      "RBHardwareSystem configured: topic bridge disabled (control_manager skeleton mode)");
    return CallbackReturn::SUCCESS;
  }

  bridge_node_ = std::make_shared<rclcpp::Node>("rb_hardware_bridge");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  joint_state_sub_ = bridge_node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(msg); });

  command_pub_ = bridge_node_->create_publisher<sensor_msgs::msg::JointState>(
    command_topic_, rclcpp::SystemDefaultsQoS());

  executor_->add_node(bridge_node_);
  spin_thread_ = std::thread([executor = executor_]() { executor->spin(); });

  RCLCPP_INFO(
    rclcpp::get_logger("rb_hardware_interface"),
    "RBHardwareSystem configured: subscribed=%s published=%s",
    joint_state_topic_.c_str(), command_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn RBHardwareSystem::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (bridge_enabled_)
  {
    stop_bridge_thread();
  }
  std::scoped_lock lock(state_mutex_);
  joint_state_received_ = false;
  bridge_active_ = false;
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
  std::fill(effort_states_.begin(), effort_states_.end(), 0.0);
  std::fill(effort_commands_.begin(), effort_commands_.end(), 0.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RBHardwareSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::scoped_lock lock(state_mutex_);
  bridge_active_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn RBHardwareSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::scoped_lock lock(state_mutex_);
  bridge_active_ = false;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RBHardwareSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joint_names_.size() * 3);

  for (std::size_t index = 0; index < joint_names_.size(); ++index)
  {
    state_interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_POSITION, &position_states_[index]);
    state_interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_VELOCITY, &velocity_states_[index]);
    state_interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_EFFORT, &effort_states_[index]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RBHardwareSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_names_.size());

  for (std::size_t index = 0; index < joint_names_.size(); ++index)
  {
    command_interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_EFFORT, &effort_commands_[index]);
  }

  return command_interfaces;
}

void RBHardwareSystem::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::scoped_lock lock(state_mutex_);
  for (std::size_t source_index = 0; source_index < msg->name.size(); ++source_index)
  {
    const auto mapping_it = joint_name_to_index_.find(msg->name[source_index]);
    if (mapping_it == joint_name_to_index_.end())
    {
      continue;
    }
    const auto target_index = mapping_it->second;
    if (source_index < msg->position.size())
    {
      position_states_[target_index] = msg->position[source_index];
    }
    if (source_index < msg->velocity.size())
    {
      velocity_states_[target_index] = msg->velocity[source_index];
    }
    if (source_index < msg->effort.size())
    {
      effort_states_[target_index] = msg->effort[source_index];
    }
  }
  joint_state_received_ = true;
}

void RBHardwareSystem::stop_bridge_thread()
{
  if (executor_ && bridge_node_)
  {
    executor_->remove_node(bridge_node_);
  }
  if (executor_)
  {
    executor_->cancel();
  }
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }
  joint_state_sub_.reset();
  command_pub_.reset();
  bridge_node_.reset();
  executor_.reset();
}

return_type RBHardwareSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // joint state callback이 최신 값을 state buffer에 반영한다.
  // read()는 controller_manager update loop에서 별도 계산 없이 그 버퍼를 그대로 사용한다.
  return return_type::OK;
}

return_type RBHardwareSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  std::scoped_lock lock(state_mutex_);
  if (!bridge_enabled_ || !bridge_active_ || !command_pub_ || !joint_state_received_)
  {
    return return_type::OK;
  }

  sensor_msgs::msg::JointState command_msg;
  command_msg.header.stamp = time;
  command_msg.name = joint_names_;
  command_msg.effort = effort_commands_;
  command_pub_->publish(command_msg);

  return return_type::OK;
}

}  // namespace rb_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  rb_hardware_interface::RBHardwareSystem,
  hardware_interface::SystemInterface)
