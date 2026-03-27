#ifndef RB_HARDWARE_INTERFACE__RB_HARDWARE_SYSTEM_HPP_
#define RB_HARDWARE_INTERFACE__RB_HARDWARE_SYSTEM_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace rb_hardware_interface
{

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using hardware_interface::return_type;

  // ros2_control이 보는 "하드웨어" 자리다.
  // 현재 단계에서는 Isaac read/write를 아직 연결하지 않고,
  // joint/interface 명세와 controller_manager 구동을 확인하는 최소 skeleton만 제공한다.
  class RBHardwareSystem : public hardware_interface::SystemInterface
  {
  public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

    void stop_bridge_thread();

    // control-only description 기준 현재 노출하는 최소 인터페이스:
    // state = position/velocity/effort, command = effort
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, std::size_t> joint_name_to_index_;
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> effort_states_;
    std::vector<double> effort_commands_;

    bool bridge_enabled_ = false;
    std::string joint_state_topic_ = "/rb/joint_states";
    std::string command_topic_ = "/rb/command_ros2_control";

    std::shared_ptr<rclcpp::Node> bridge_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;

    std::mutex state_mutex_;
    bool joint_state_received_ = false;
    bool bridge_active_ = false;
  };

} // namespace rb_hardware_interface

#endif // RB_HARDWARE_INTERFACE__RB_HARDWARE_SYSTEM_HPP_
