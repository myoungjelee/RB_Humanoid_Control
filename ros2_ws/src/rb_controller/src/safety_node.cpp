#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// 컨트롤러 출력에 안전 필터를 적용해 최종 /rb/command_safe를 퍼블리시하는 노드
class RbSafetyNode : public rclcpp::Node
{
public:
  /**
   * @brief rb_safety 노드를 생성하고 ROS2 통신/타이머를 초기화한다.
   */
  RbSafetyNode()
      : Node("rb_safety"),
        node_start_steady_(std::chrono::steady_clock::now())
  {
    input_command_topic_ = this->declare_parameter<std::string>(
        "input_command_topic", "/rb/command_raw");
    output_command_topic_ = this->declare_parameter<std::string>(
        "output_command_topic", "/rb/command_safe");
    input_joint_states_topic_ = this->declare_parameter<std::string>(
        "input_joint_states_topic", "/rb/joint_states");
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/rb/imu");

    safety_enabled_ = this->declare_parameter<bool>("safety_enabled", true);
    safe_mode_action_ = this->declare_parameter<std::string>("safe_mode_action", "zero_effort");
    effort_abs_max_default_ = this->declare_parameter<double>("effort_abs_max_default", 8.0);
    joint_limit_margin_rad_ = this->declare_parameter<double>("joint_limit_margin_rad", 0.05);
    input_timeout_sec_ = this->declare_parameter<double>("input_timeout_sec", 0.15);
    tilt_limit_roll_rad_ = this->declare_parameter<double>("tilt_limit_roll_rad", 0.35);
    tilt_limit_pitch_rad_ = this->declare_parameter<double>("tilt_limit_pitch_rad", 0.35);

    if (effort_abs_max_default_ < 0.0)
    {
      effort_abs_max_default_ = std::abs(effort_abs_max_default_);
    }
    if (joint_limit_margin_rad_ < 0.0)
    {
      joint_limit_margin_rad_ = 0.0;
    }
    if (input_timeout_sec_ < 0.0)
    {
      input_timeout_sec_ = 0.0;
    }
    if (tilt_limit_roll_rad_ < 0.0)
    {
      tilt_limit_roll_rad_ = 0.0;
    }
    if (tilt_limit_pitch_rad_ < 0.0)
    {
      tilt_limit_pitch_rad_ = 0.0;
    }

    command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        output_command_topic_, 10);
    command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        input_command_topic_, 10,
        std::bind(&RbSafetyNode::on_command, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        input_joint_states_topic_, 10,
        std::bind(&RbSafetyNode::on_joint_state, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10,
        std::bind(&RbSafetyNode::on_imu, this, std::placeholders::_1));

    // safety.yaml의 joint_limits.* 파라미터를 로딩한다.
    load_joint_limits_from_overrides();

    RCLCPP_INFO(
        this->get_logger(),
        "rb_safety started: in=%s out=%s js=%s imu=%s safety=%s limits=%zu",
        input_command_topic_.c_str(), output_command_topic_.c_str(),
        input_joint_states_topic_.c_str(), imu_topic_.c_str(),
        (safety_enabled_ ? "on" : "off"), joint_limits_.size());
  }

private:
  enum class SafetyReason : std::uint8_t
  {
    NORMAL = 0,
    CLAMP = 1,
    JOINT_LIMIT = 2,
    TIMEOUT = 3,
    TILT = 4
  };

  struct JointLimitBound
  {
    double lower{0.0};
    double upper{0.0};
    bool has_lower{false};
    bool has_upper{false};
  };

  /**
   * @brief /rb/command_raw 수신 콜백: 안전 필터를 적용해 /rb/command_safe로 전달한다.
   */
  void on_command(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    sensor_msgs::msg::JointState out = *msg;
    const auto now_steady = std::chrono::steady_clock::now();
    apply_safety_filters(out, now_steady);
    command_pub_->publish(out);
  }

  /**
   * @brief /rb/joint_states 수신 콜백: joint position과 input freshness를 갱신한다.
   */
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->name.empty())
    {
      return;
    }

    latest_joint_positions_.clear();
    const std::size_t position_count = std::min(msg->name.size(), msg->position.size());
    for (std::size_t i = 0; i < position_count; ++i)
    {
      latest_joint_positions_[msg->name[i]] = msg->position[i];
    }
    latest_joint_state_received_ = true;
    latest_joint_state_time_steady_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief /rb/imu 수신 콜백: roll/pitch 추정을 위한 최신 자세를 저장한다.
   */
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    const double x = msg->orientation.x;
    const double y = msg->orientation.y;
    const double z = msg->orientation.z;
    const double w = msg->orientation.w;

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    latest_roll_rad_ = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    latest_pitch_rad_ = std::asin(std::clamp(sinp, -1.0, 1.0));

    latest_imu_received_ = true;
    latest_imu_time_steady_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief 안전 필터 체인을 적용해 최종 제어 명령을 보호한다.
   *
   * 우선순위: TILT > TIMEOUT > JOINT_LIMIT > CLAMP > NORMAL
   */
  void apply_safety_filters(
      sensor_msgs::msg::JointState &cmd_msg,
      const std::chrono::steady_clock::time_point &now_steady)
  {
    if (!safety_enabled_)
    {
      set_safety_reason(SafetyReason::NORMAL);
      return;
    }

    const bool clamp_active = apply_effort_clamp(cmd_msg);
    const bool joint_limit_active = apply_joint_limit_guard(cmd_msg);
    const bool timeout_active = is_input_timeout(now_steady);
    const bool tilt_active = is_tilt_exceeded();

    SafetyReason reason = SafetyReason::NORMAL;
    if (tilt_active)
    {
      reason = SafetyReason::TILT;
      force_safe_action(cmd_msg);
    }
    else if (timeout_active)
    {
      reason = SafetyReason::TIMEOUT;
      force_safe_action(cmd_msg);
    }
    else if (joint_limit_active)
    {
      reason = SafetyReason::JOINT_LIMIT;
    }
    else if (clamp_active)
    {
      reason = SafetyReason::CLAMP;
    }

    set_safety_reason(reason);
  }

  /**
   * @brief effort 절대값 상한을 적용한다.
   */
  bool apply_effort_clamp(sensor_msgs::msg::JointState &cmd_msg)
  {
    if (effort_abs_max_default_ <= 0.0)
    {
      return false;
    }

    std::size_t hit_count = 0;
    for (double & effort_cmd : cmd_msg.effort)
    {
      const double before = effort_cmd;
      effort_cmd = std::clamp(effort_cmd, -effort_abs_max_default_, effort_abs_max_default_);
      if (before != effort_cmd)
      {
        ++hit_count;
      }
    }

    if (hit_count > 0U)
    {
      clamp_hit_total_ += hit_count;
      clamp_hit_window_ += hit_count;
      return true;
    }
    return false;
  }

  /**
   * @brief 조인트 각도 한계 근처에서 바깥 방향 effort를 차단한다.
   */
  bool apply_joint_limit_guard(sensor_msgs::msg::JointState &cmd_msg)
  {
    if (joint_limits_.empty() || latest_joint_positions_.empty())
    {
      return false;
    }

    std::size_t hit_count = 0;
    for (std::size_t i = 0; i < cmd_msg.name.size(); ++i)
    {
      const auto limit_it = joint_limits_.find(cmd_msg.name[i]);
      if (limit_it == joint_limits_.end())
      {
        continue;
      }

      const auto pos_it = latest_joint_positions_.find(cmd_msg.name[i]);
      if (pos_it == latest_joint_positions_.end())
      {
        continue;
      }

      const double lower = limit_it->second.lower;
      const double upper = limit_it->second.upper;
      double soft_lower = lower + joint_limit_margin_rad_;
      double soft_upper = upper - joint_limit_margin_rad_;
      if (soft_lower > soft_upper)
      {
        const double mid = 0.5 * (lower + upper);
        soft_lower = mid;
        soft_upper = mid;
      }

      const double position = pos_it->second;
      double & effort_cmd = cmd_msg.effort[i];
      bool limited = false;

      if (position <= soft_lower && effort_cmd < 0.0)
      {
        effort_cmd = 0.0;
        limited = true;
      }
      if (position >= soft_upper && effort_cmd > 0.0)
      {
        effort_cmd = 0.0;
        limited = true;
      }
      if ((position < lower || position > upper) && effort_cmd != 0.0)
      {
        effort_cmd = 0.0;
        limited = true;
      }

      if (limited)
      {
        ++hit_count;
      }
    }

    if (hit_count > 0U)
    {
      joint_limit_hit_total_ += hit_count;
      joint_limit_hit_window_ += hit_count;
      return true;
    }
    return false;
  }

  /**
   * @brief 입력 신선도(현재는 /rb/joint_states)를 기준으로 timeout 여부를 판단한다.
   */
  bool is_input_timeout(const std::chrono::steady_clock::time_point &now_steady) const
  {
    if (input_timeout_sec_ <= 0.0)
    {
      return false;
    }

    if (!latest_joint_state_received_)
    {
      const double startup_elapsed = std::chrono::duration<double>(now_steady - node_start_steady_).count();
      return startup_elapsed > input_timeout_sec_;
    }

    const double input_stale_sec =
        std::chrono::duration<double>(now_steady - latest_joint_state_time_steady_).count();
    return input_stale_sec > input_timeout_sec_;
  }

  /**
   * @brief 최신 IMU 기울기가 임계치를 넘는지 판단한다.
   */
  bool is_tilt_exceeded()
  {
    if (tilt_limit_roll_rad_ <= 0.0 && tilt_limit_pitch_rad_ <= 0.0)
    {
      return false;
    }
    if (!latest_imu_received_)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "tilt check enabled but /rb/imu has no samples yet");
      return false;
    }

    const bool roll_exceeded =
        (tilt_limit_roll_rad_ > 0.0) && (std::abs(latest_roll_rad_) > tilt_limit_roll_rad_);
    const bool pitch_exceeded =
        (tilt_limit_pitch_rad_ > 0.0) && (std::abs(latest_pitch_rad_) > tilt_limit_pitch_rad_);
    return roll_exceeded || pitch_exceeded;
  }

  /**
   * @brief 안전 모드 액션을 명령에 적용한다.
   */
  void force_safe_action(sensor_msgs::msg::JointState &cmd_msg)
  {
    if (safe_mode_action_ != "zero_effort")
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "unknown safe_mode_action '%s'. fallback=zero_effort", safe_mode_action_.c_str());
    }

    std::fill(cmd_msg.effort.begin(), cmd_msg.effort.end(), 0.0);
    std::fill(cmd_msg.velocity.begin(), cmd_msg.velocity.end(), 0.0);
    std::fill(cmd_msg.position.begin(), cmd_msg.position.end(), 0.0);
  }

  /**
   * @brief 안전 상태 전환을 기록하고 이벤트 카운터를 갱신한다.
   */
  void set_safety_reason(SafetyReason reason)
  {
    if (reason == last_safety_reason_)
    {
      return;
    }

    if (reason == SafetyReason::TIMEOUT)
    {
      ++timeout_event_total_;
      ++timeout_event_window_;
    }
    else if (reason == SafetyReason::TILT)
    {
      ++tilt_event_total_;
      ++tilt_event_window_;
    }

    if (reason == SafetyReason::NORMAL)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "safety cleared: prev=%s", safety_reason_to_string(last_safety_reason_));
    }
    else
    {
      RCLCPP_WARN(
          this->get_logger(),
          "safety active: reason=%s", safety_reason_to_string(reason));
    }
    last_safety_reason_ = reason;
  }

  /**
   * @brief safety reason enum을 문자열로 변환한다.
   */
  static const char * safety_reason_to_string(SafetyReason reason)
  {
    switch (reason)
    {
      case SafetyReason::CLAMP:
        return "CLAMP";
      case SafetyReason::JOINT_LIMIT:
        return "JOINT_LIMIT";
      case SafetyReason::TIMEOUT:
        return "TIMEOUT";
      case SafetyReason::TILT:
        return "TILT";
      case SafetyReason::NORMAL:
      default:
        return "NORMAL";
    }
  }

  /**
   * @brief 파라미터 override에서 joint_limits.*.lower/upper를 읽어 캐시에 적재한다.
   */
  void load_joint_limits_from_overrides()
  {
    joint_limits_.clear();
    const auto & overrides = this->get_node_parameters_interface()->get_parameter_overrides();
    const std::string prefix = "joint_limits.";
    const std::string lower_suffix = ".lower";
    const std::string upper_suffix = ".upper";

    for (const auto & kv : overrides)
    {
      const std::string & full_name = kv.first;
      const rclcpp::ParameterValue & value = kv.second;
      if (full_name.rfind(prefix, 0) != 0)
      {
        continue;
      }

      const bool is_lower = full_name.size() > lower_suffix.size() &&
          full_name.compare(full_name.size() - lower_suffix.size(), lower_suffix.size(), lower_suffix) == 0;
      const bool is_upper = full_name.size() > upper_suffix.size() &&
          full_name.compare(full_name.size() - upper_suffix.size(), upper_suffix.size(), upper_suffix) == 0;
      if (!is_lower && !is_upper)
      {
        continue;
      }

      const std::string joint_name = full_name.substr(
          prefix.size(),
          full_name.size() - prefix.size() - (is_lower ? lower_suffix.size() : upper_suffix.size()));
      if (joint_name.empty())
      {
        continue;
      }

      auto & bound = joint_limits_[joint_name];
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        if (is_lower)
        {
          bound.lower = value.get<double>();
          bound.has_lower = true;
        }
        else
        {
          bound.upper = value.get<double>();
          bound.has_upper = true;
        }
      }
      else if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        if (is_lower)
        {
          bound.lower = static_cast<double>(value.get<int64_t>());
          bound.has_lower = true;
        }
        else
        {
          bound.upper = static_cast<double>(value.get<int64_t>());
          bound.has_upper = true;
        }
      }
    }

    for (auto it = joint_limits_.begin(); it != joint_limits_.end();)
    {
      if (!it->second.has_lower && !it->second.has_upper)
      {
        it = joint_limits_.erase(it);
        continue;
      }
      if (!it->second.has_lower)
      {
        it->second.lower = it->second.upper;
      }
      if (!it->second.has_upper)
      {
        it->second.upper = it->second.lower;
      }
      if (it->second.lower > it->second.upper)
      {
        std::swap(it->second.lower, it->second.upper);
      }
      ++it;
    }

    if (joint_limits_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "joint_limits are empty. joint limit guard disabled");
    }
  }

  std::string input_command_topic_{"/rb/command_raw"};
  std::string output_command_topic_{"/rb/command_safe"};
  std::string input_joint_states_topic_{"/rb/joint_states"};
  std::string imu_topic_{"/rb/imu"};

  bool safety_enabled_{true};
  std::string safe_mode_action_{"zero_effort"};
  double effort_abs_max_default_{8.0};
  double joint_limit_margin_rad_{0.05};
  double input_timeout_sec_{0.15};
  double tilt_limit_roll_rad_{0.35};
  double tilt_limit_pitch_rad_{0.35};

  std::unordered_map<std::string, double> latest_joint_positions_;
  std::unordered_map<std::string, JointLimitBound> joint_limits_;

  std::size_t clamp_hit_total_{0};
  std::size_t clamp_hit_window_{0};
  std::size_t joint_limit_hit_total_{0};
  std::size_t joint_limit_hit_window_{0};
  std::size_t timeout_event_total_{0};
  std::size_t timeout_event_window_{0};
  std::size_t tilt_event_total_{0};
  std::size_t tilt_event_window_{0};
  SafetyReason last_safety_reason_{SafetyReason::NORMAL};

  std::chrono::steady_clock::time_point node_start_steady_;
  std::chrono::steady_clock::time_point latest_joint_state_time_steady_;
  std::chrono::steady_clock::time_point latest_imu_time_steady_;

  bool latest_joint_state_received_{false};
  bool latest_imu_received_{false};
  double latest_roll_rad_{0.0};
  double latest_pitch_rad_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RbSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
