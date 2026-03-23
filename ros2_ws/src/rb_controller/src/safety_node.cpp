#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rb_controller/msg/estimated_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
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
    input_estimated_state_topic_ = this->declare_parameter<std::string>(
        "input_estimated_state_topic", "/rb/estimated_state");
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/rb/imu");

    safety_enabled_ = this->declare_parameter<bool>("safety_enabled", true);
    safe_mode_action_ = this->declare_parameter<std::string>("safe_mode_action", "zero_effort");
    effort_abs_max_default_ = this->declare_parameter<double>("effort_abs_max_default", 8.0);
    joint_limit_margin_rad_ = this->declare_parameter<double>("joint_limit_margin_rad", 0.05);
    input_timeout_sec_ = this->declare_parameter<double>("input_timeout_sec", 0.3);
    tilt_limit_roll_rad_ = this->declare_parameter<double>("tilt_limit_roll_rad", 0.6);
    tilt_limit_pitch_rad_ = this->declare_parameter<double>("tilt_limit_pitch_rad", 0.6);
    velocity_limit_default_rad_s_ = this->declare_parameter<double>("velocity_limit_default_rad_s", 8.0);
    velocity_limit_ankle_rad_s_ = this->declare_parameter<double>("velocity_limit_ankle_rad_s", 12.0);
    watchdog_rate_hz_ = this->declare_parameter<double>("watchdog_rate_hz", 100.0);
    joint_limit_debug_log_ = this->declare_parameter<bool>("joint_limit_debug_log", true);
    // TILT 원인 분석용 상세 로그 on/off
    tilt_debug_log_ = this->declare_parameter<bool>("tilt_debug_log", true);

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
    if (velocity_limit_default_rad_s_ < 0.0)
    {
      velocity_limit_default_rad_s_ = 0.0;
    }
    if (velocity_limit_ankle_rad_s_ < 0.0)
    {
      velocity_limit_ankle_rad_s_ = 0.0;
    }
    if (watchdog_rate_hz_ <= 0.0)
    {
      watchdog_rate_hz_ = 100.0;
    }

    command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        output_command_topic_, 10);
    command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        input_command_topic_, 10,
        std::bind(&RbSafetyNode::on_command, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        input_joint_states_topic_, 10,
        std::bind(&RbSafetyNode::on_joint_state, this, std::placeholders::_1));
    estimated_state_sub_ = this->create_subscription<rb_controller::msg::EstimatedState>(
        input_estimated_state_topic_, 10,
        std::bind(&RbSafetyNode::on_estimated_state, this, std::placeholders::_1));
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / watchdog_rate_hz_)),
        std::bind(&RbSafetyNode::on_watchdog_timer, this));

    // safety.yaml의 joint_limits.* 파라미터를 로딩한다.
    load_joint_limits_from_overrides();

    // 런타임 ros2 param set 요청을 안전하게 반영한다.
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&RbSafetyNode::on_parameters_changed, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "rb_safety started: in=%s out=%s js=%s est=%s safety=%s limits=%zu timeout=%.3fs vel_limit(default/ankle)=%.3f/%.3f watchdog=%.1fHz",
        input_command_topic_.c_str(), output_command_topic_.c_str(),
        input_joint_states_topic_.c_str(), input_estimated_state_topic_.c_str(),
        (safety_enabled_ ? "on" : "off"), joint_limits_.size(),
        input_timeout_sec_, velocity_limit_default_rad_s_, velocity_limit_ankle_rad_s_, watchdog_rate_hz_);
  }

private:
  enum class SafetyReason : std::uint8_t
  {
    NORMAL = 0,
    CLAMP = 1,
    JOINT_LIMIT = 2,
    TIMEOUT = 3,
    TILT = 4,
    VELOCITY_LIMIT = 5
  };

  struct JointLimitBound
  {
    double lower{0.0};
    double upper{0.0};
    bool has_lower{false};
    bool has_upper{false};
  };

  // JOINT_LIMIT 원인 파악용 최근 히트 1건 디버그 정보
  struct JointLimitDebugInfo
  {
    bool valid{false};
    std::string joint_name{};
    std::string trigger{};
    double position{0.0};
    double effort_before{0.0};
    double soft_lower{0.0};
    double soft_upper{0.0};
    double hard_lower{0.0};
    double hard_upper{0.0};
  };

  struct VelocityLimitDebugInfo
  {
    bool valid{false};
    std::string joint_name{};
    double velocity_rad_s{0.0};
    double limit_rad_s{0.0};
  };

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

      if (name == "input_command_topic" || name == "output_command_topic" ||
          name == "input_joint_states_topic" || name == "input_estimated_state_topic" || name == "imu_topic")
      {
        result.successful = false;
        result.reason = "runtime update not supported for topic parameters (restart required)";
        return result;
      }
      if (name.rfind("joint_limits.", 0) == 0)
      {
        result.successful = false;
        result.reason = "runtime update not supported for joint_limits.* (restart required)";
        return result;
      }

      if (name == "safety_enabled")
      {
        safety_enabled_ = param.as_bool();
        continue;
      }
      if (name == "watchdog_rate_hz")
      {
        result.successful = false;
        result.reason = "runtime update not supported for watchdog_rate_hz (restart required)";
        return result;
      }
      if (name == "safe_mode_action")
      {
        const std::string action = param.as_string();
        if (action != "zero_effort")
        {
          result.successful = false;
          result.reason = "safe_mode_action currently supports only zero_effort";
          return result;
        }
        safe_mode_action_ = action;
        continue;
      }
      if (name == "effort_abs_max_default")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "effort_abs_max_default must be >= 0";
          return result;
        }
        effort_abs_max_default_ = v;
        continue;
      }
      if (name == "joint_limit_margin_rad")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "joint_limit_margin_rad must be >= 0";
          return result;
        }
        joint_limit_margin_rad_ = v;
        continue;
      }
      if (name == "input_timeout_sec")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "input_timeout_sec must be >= 0";
          return result;
        }
        input_timeout_sec_ = v;
        continue;
      }
      if (name == "tilt_limit_roll_rad")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_limit_roll_rad must be >= 0";
          return result;
        }
        tilt_limit_roll_rad_ = v;
        continue;
      }
      if (name == "tilt_limit_pitch_rad")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "tilt_limit_pitch_rad must be >= 0";
          return result;
        }
        tilt_limit_pitch_rad_ = v;
        continue;
      }
      if (name == "velocity_limit_default_rad_s")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "velocity_limit_default_rad_s must be >= 0";
          return result;
        }
        velocity_limit_default_rad_s_ = v;
        continue;
      }
      if (name == "velocity_limit_ankle_rad_s")
      {
        const double v = param.as_double();
        if (v < 0.0)
        {
          result.successful = false;
          result.reason = "velocity_limit_ankle_rad_s must be >= 0";
          return result;
        }
        velocity_limit_ankle_rad_s_ = v;
        continue;
      }
      if (name == "joint_limit_debug_log")
      {
        joint_limit_debug_log_ = param.as_bool();
        continue;
      }
      if (name == "tilt_debug_log")
      {
        tilt_debug_log_ = param.as_bool();
        continue;
      }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "runtime safety parameters updated: safety=%s safe_mode=%s effort_max=%.3f margin=%.4f timeout=%.3f tilt_limit=[%.3f,%.3f] vel_limit(default/ankle)=%.3f/%.3f debug(joint/tilt)=%s/%s",
        safety_enabled_ ? "on" : "off",
        safe_mode_action_.c_str(),
        effort_abs_max_default_,
        joint_limit_margin_rad_,
        input_timeout_sec_,
        tilt_limit_roll_rad_,
        tilt_limit_pitch_rad_,
        velocity_limit_default_rad_s_,
        velocity_limit_ankle_rad_s_,
        joint_limit_debug_log_ ? "on" : "off",
        tilt_debug_log_ ? "on" : "off");
    return result;
  }

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
    latest_raw_command_ = out;
    latest_command_received_ = true;
    latest_command_time_steady_ = now_steady;
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
    latest_joint_velocities_.clear();
    latest_joint_names_ = msg->name;
    const std::size_t position_count = std::min(msg->name.size(), msg->position.size());
    for (std::size_t i = 0; i < position_count; ++i)
    {
      latest_joint_positions_[msg->name[i]] = msg->position[i];
    }
    const std::size_t velocity_count = std::min(msg->name.size(), msg->velocity.size());
    for (std::size_t i = 0; i < velocity_count; ++i)
    {
      latest_joint_velocities_[msg->name[i]] = msg->velocity[i];
    }
    latest_joint_state_received_ = true;
    latest_joint_state_time_steady_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief /rb/imu 수신 콜백: roll/pitch 추정을 위한 최신 자세를 저장한다.
   */
  void on_estimated_state(const rb_controller::msg::EstimatedState::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    latest_roll_rad_ = msg->tilt_roll_rad;
    latest_pitch_rad_ = msg->tilt_pitch_rad;
    latest_estimated_state_received_ = true;
    latest_estimated_state_time_steady_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief command 입력이 끊겼을 때도 주기적으로 safe command를 퍼블리시한다.
   */
  void on_watchdog_timer()
  {
    if (!safety_enabled_)
    {
      return;
    }

    const auto now_steady = std::chrono::steady_clock::now();
    if (!is_input_timeout(now_steady))
    {
      return;
    }

    sensor_msgs::msg::JointState safe_cmd = latest_command_received_
        ? latest_raw_command_
        : build_zero_command_template();
    if (safe_cmd.name.empty() && safe_cmd.effort.empty())
    {
      set_safety_reason(SafetyReason::TIMEOUT);
      return;
    }

    force_safe_action(safe_cmd);
    command_pub_->publish(safe_cmd);
    set_safety_reason(SafetyReason::TIMEOUT);
  }

  /**
   * @brief 안전 필터 체인을 적용해 최종 제어 명령을 보호한다.
   *
   * 우선순위: TILT > TIMEOUT > VELOCITY_LIMIT > JOINT_LIMIT > CLAMP > NORMAL
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
    const bool velocity_limit_active = is_velocity_exceeded();
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
    else if (velocity_limit_active)
    {
      reason = SafetyReason::VELOCITY_LIMIT;
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

    joint_limit_debug_info_.valid = false;
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
      const double effort_before = effort_cmd;
      bool limited = false;
      std::string trigger = "";

      if (position <= soft_lower && effort_cmd < 0.0)
      {
        effort_cmd = 0.0;
        limited = true;
        if (trigger.empty())
        {
          trigger = "SOFT_LOWER_OUTWARD";
        }
      }
      if (position >= soft_upper && effort_cmd > 0.0)
      {
        effort_cmd = 0.0;
        limited = true;
        if (trigger.empty())
        {
          trigger = "SOFT_UPPER_OUTWARD";
        }
      }
      if ((position < lower || position > upper) && effort_cmd != 0.0)
      {
        effort_cmd = 0.0;
        limited = true;
        if (trigger.empty())
        {
          trigger = "HARD_LIMIT";
        }
      }

      if (limited)
      {
        ++hit_count;
        if (!joint_limit_debug_info_.valid)
        {
          joint_limit_debug_info_.valid = true;
          joint_limit_debug_info_.joint_name = cmd_msg.name[i];
          joint_limit_debug_info_.trigger = trigger;
          joint_limit_debug_info_.position = position;
          joint_limit_debug_info_.effort_before = effort_before;
          joint_limit_debug_info_.soft_lower = soft_lower;
          joint_limit_debug_info_.soft_upper = soft_upper;
          joint_limit_debug_info_.hard_lower = lower;
          joint_limit_debug_info_.hard_upper = upper;
        }
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
   * @brief 최근 command_raw 수신 시각을 기준으로 timeout 여부를 판단한다.
   */
  bool is_input_timeout(const std::chrono::steady_clock::time_point &now_steady) const
  {
    if (input_timeout_sec_ <= 0.0)
    {
      return false;
    }

    if (!latest_command_received_)
    {
      const double startup_elapsed = std::chrono::duration<double>(now_steady - node_start_steady_).count();
      return startup_elapsed > input_timeout_sec_;
    }

    const double command_stale_sec =
        std::chrono::duration<double>(now_steady - latest_command_time_steady_).count();
    return command_stale_sec > input_timeout_sec_;
  }

  /**
   * @brief 관절 속도가 허용 한계를 넘었는지 판단한다.
   */
  bool is_velocity_exceeded()
  {
    if (velocity_limit_default_rad_s_ <= 0.0 && velocity_limit_ankle_rad_s_ <= 0.0)
    {
      return false;
    }
    if (latest_joint_velocities_.empty())
    {
      return false;
    }

    velocity_limit_debug_info_.valid = false;
    std::size_t hit_count = 0;
    for (const auto &kv : latest_joint_velocities_)
    {
      const double limit = velocity_limit_for_joint(kv.first);
      if (limit <= 0.0)
      {
        continue;
      }
      const double abs_velocity = std::abs(kv.second);
      if (abs_velocity <= limit)
      {
        continue;
      }

      ++hit_count;
      if (!velocity_limit_debug_info_.valid)
      {
        velocity_limit_debug_info_.valid = true;
        velocity_limit_debug_info_.joint_name = kv.first;
        velocity_limit_debug_info_.velocity_rad_s = kv.second;
        velocity_limit_debug_info_.limit_rad_s = limit;
      }
    }

    if (hit_count > 0U)
    {
      velocity_limit_hit_total_ += hit_count;
      velocity_limit_hit_window_ += hit_count;
      return true;
    }
    return false;
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
    if (!latest_estimated_state_received_)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "tilt check enabled but /rb/estimated_state has no samples yet");
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

    // effort-only 명령 경로를 유지하기 위해 safe action도 effort만 0으로 만든다.
    // position/velocity는 비워 articulation에 추가 제약을 넣지 않는다.
    std::fill(cmd_msg.effort.begin(), cmd_msg.effort.end(), 0.0);
    cmd_msg.velocity.clear();
    cmd_msg.position.clear();
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
    else if (reason == SafetyReason::VELOCITY_LIMIT)
    {
      ++velocity_limit_event_total_;
      ++velocity_limit_event_window_;
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
      if (reason == SafetyReason::JOINT_LIMIT && joint_limit_debug_log_ && joint_limit_debug_info_.valid)
      {
        RCLCPP_WARN(
            this->get_logger(),
            "safety active: reason=JOINT_LIMIT joint=%s trigger=%s pos=%.4f effort_before=%.4f soft=[%.4f,%.4f] hard=[%.4f,%.4f]",
            joint_limit_debug_info_.joint_name.c_str(),
            joint_limit_debug_info_.trigger.c_str(),
            joint_limit_debug_info_.position,
            joint_limit_debug_info_.effort_before,
            joint_limit_debug_info_.soft_lower,
            joint_limit_debug_info_.soft_upper,
            joint_limit_debug_info_.hard_lower,
            joint_limit_debug_info_.hard_upper);
      }
      else if (reason == SafetyReason::TIMEOUT)
      {
        const double command_age_sec = latest_command_received_
            ? std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_command_time_steady_).count()
            : -1.0;
        RCLCPP_WARN(
            this->get_logger(),
            "safety active: reason=TIMEOUT command_age=%.3fs limit=%.3fs command_received=%s",
            command_age_sec,
            input_timeout_sec_,
            latest_command_received_ ? "yes" : "no");
      }
      else if (reason == SafetyReason::VELOCITY_LIMIT && velocity_limit_debug_info_.valid)
      {
        RCLCPP_WARN(
            this->get_logger(),
            "safety active: reason=VELOCITY_LIMIT joint=%s velocity=%.3f limit=%.3f",
            velocity_limit_debug_info_.joint_name.c_str(),
            velocity_limit_debug_info_.velocity_rad_s,
            velocity_limit_debug_info_.limit_rad_s);
      }
      else if (reason == SafetyReason::TILT && tilt_debug_log_)
      {
        // TILT는 관절명이 바로 나오지 않으므로 축별 초과 정보를 함께 남긴다.
        const double abs_roll = std::abs(latest_roll_rad_);
        const double abs_pitch = std::abs(latest_pitch_rad_);
        const bool roll_exceeded = (tilt_limit_roll_rad_ > 0.0) && (abs_roll > tilt_limit_roll_rad_);
        const bool pitch_exceeded = (tilt_limit_pitch_rad_ > 0.0) && (abs_pitch > tilt_limit_pitch_rad_);
        const char * axis =
            (roll_exceeded && pitch_exceeded) ? "ROLL+PITCH" :
            (roll_exceeded ? "ROLL" : (pitch_exceeded ? "PITCH" : "UNKNOWN"));
        const double estimated_state_age_sec = latest_estimated_state_received_
            ? std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_estimated_state_time_steady_).count()
            : -1.0;
        RCLCPP_WARN(
            this->get_logger(),
            "safety active: reason=TILT axis=%s roll=%.3f pitch=%.3f abs=[%.3f,%.3f] limit=[%.3f,%.3f] estimated_state_age=%.3fs",
            axis,
            latest_roll_rad_,
            latest_pitch_rad_,
            abs_roll,
            abs_pitch,
            tilt_limit_roll_rad_,
            tilt_limit_pitch_rad_,
            estimated_state_age_sec);
      }
      else
      {
        RCLCPP_WARN(
            this->get_logger(),
            "safety active: reason=%s", safety_reason_to_string(reason));
      }
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
      case SafetyReason::VELOCITY_LIMIT:
        return "VELOCITY_LIMIT";
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

  double velocity_limit_for_joint(const std::string &joint_name) const
  {
    if (joint_name.find("ankle") != std::string::npos)
    {
      return velocity_limit_ankle_rad_s_;
    }
    return velocity_limit_default_rad_s_;
  }

  sensor_msgs::msg::JointState build_zero_command_template() const
  {
    sensor_msgs::msg::JointState msg;
    if (!latest_joint_names_.empty())
    {
      msg.name = latest_joint_names_;
      msg.effort.assign(latest_joint_names_.size(), 0.0);
    }
    return msg;
  }

  static double normalize_angle_rad(const double angle_rad)
  {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
  }

  std::string input_command_topic_{"/rb/command_raw"};
  std::string output_command_topic_{"/rb/command_safe"};
  std::string input_joint_states_topic_{"/rb/joint_states"};
  std::string input_estimated_state_topic_{"/rb/estimated_state"};
  std::string imu_topic_{"/rb/imu"};

  bool safety_enabled_{true};
  std::string safe_mode_action_{"zero_effort"};
  double effort_abs_max_default_{8.0};
  double joint_limit_margin_rad_{0.05};
  double input_timeout_sec_{0.3};
  double tilt_limit_roll_rad_{0.6};
  double tilt_limit_pitch_rad_{0.6};
  double velocity_limit_default_rad_s_{8.0};
  double velocity_limit_ankle_rad_s_{12.0};
  double watchdog_rate_hz_{100.0};
  bool joint_limit_debug_log_{true};
  bool tilt_debug_log_{true};

  std::unordered_map<std::string, double> latest_joint_positions_;
  std::unordered_map<std::string, double> latest_joint_velocities_;
  std::unordered_map<std::string, JointLimitBound> joint_limits_;
  JointLimitDebugInfo joint_limit_debug_info_;
  VelocityLimitDebugInfo velocity_limit_debug_info_;
  std::vector<std::string> latest_joint_names_;

  std::size_t clamp_hit_total_{0};
  std::size_t clamp_hit_window_{0};
  std::size_t joint_limit_hit_total_{0};
  std::size_t joint_limit_hit_window_{0};
  std::size_t velocity_limit_hit_total_{0};
  std::size_t velocity_limit_hit_window_{0};
  std::size_t timeout_event_total_{0};
  std::size_t timeout_event_window_{0};
  std::size_t velocity_limit_event_total_{0};
  std::size_t velocity_limit_event_window_{0};
  std::size_t tilt_event_total_{0};
  std::size_t tilt_event_window_{0};
  SafetyReason last_safety_reason_{SafetyReason::NORMAL};

  std::chrono::steady_clock::time_point node_start_steady_;
  std::chrono::steady_clock::time_point latest_command_time_steady_;
  std::chrono::steady_clock::time_point latest_joint_state_time_steady_;
  std::chrono::steady_clock::time_point latest_estimated_state_time_steady_;

  bool latest_command_received_{false};
  bool latest_joint_state_received_{false};
  bool latest_estimated_state_received_{false};
  double latest_roll_rad_{0.0};
  double latest_pitch_rad_{0.0};
  sensor_msgs::msg::JointState latest_raw_command_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<rb_controller::msg::EstimatedState>::SharedPtr estimated_state_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RbSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
