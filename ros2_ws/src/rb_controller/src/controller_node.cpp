#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// 200Hz 제어 루프 타이밍을 측정하고, 더미 명령(/rb/command_raw)을 퍼블리시하는 M2 검증용 노드
class RbControllerNode : public rclcpp::Node
{
public:
  /**
   * @brief rb_controller 노드를 생성하고 ROS2 통신/타이머를 초기화한다.
   *
   * 파라미터를 선언하고, /rb/joint_states 구독 + /rb/command_raw 퍼블리시 +
   * wall timer(기본 200Hz)를 연결한다.
   */
  RbControllerNode()
      : Node("rb_controller"),
        node_start_steady_(std::chrono::steady_clock::now()),
        steady_prev_tick_(std::chrono::steady_clock::time_point::min()),
        steady_last_log_(std::chrono::steady_clock::now())
  {
    // 제어 주기/로그/토픽을 런타임 파라미터로 받는다.
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
    signal_mode_ = this->declare_parameter<std::string>("signal_mode", "zero");
    target_joint_ = this->declare_parameter<std::string>("target_joint", "torso_joint");
    effort_amplitude_ = this->declare_parameter<double>("effort_amplitude", 0.0);
    effort_frequency_hz_ = this->declare_parameter<double>("effort_frequency_hz", 0.5);
    step_start_sec_ = this->declare_parameter<double>("step_start_sec", 1.0);
    safety_enabled_ = this->declare_parameter<bool>("safety_enabled", false);
    safe_mode_action_ = this->declare_parameter<std::string>("safe_mode_action", "zero_effort");
    effort_abs_max_default_ = this->declare_parameter<double>("effort_abs_max_default", 8.0);
    joint_limit_margin_rad_ = this->declare_parameter<double>("joint_limit_margin_rad", 0.05);
    input_timeout_sec_ = this->declare_parameter<double>("input_timeout_sec", 0.15);
    tilt_limit_roll_rad_ = this->declare_parameter<double>("tilt_limit_roll_rad", 0.35);
    tilt_limit_pitch_rad_ = this->declare_parameter<double>("tilt_limit_pitch_rad", 0.35);
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/rb/imu");

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
    if (effort_frequency_hz_ < 0.0)
    {
      effort_frequency_hz_ = 0.5;
    }
    if (step_start_sec_ < 0.0)
    {
      step_start_sec_ = 1.0;
    }
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

    // 200Hz 기준 expected dt = 0.005s
    expected_dt_sec_ = 1.0 / control_rate_hz_;

    // 제어 명령 publish, joint state subscribe 배선
    command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output_topic_, 10);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        input_topic_, 10,
        std::bind(&RbControllerNode::on_joint_state, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10,
        std::bind(&RbControllerNode::on_imu, this, std::placeholders::_1));

    // safety.yaml의 joint_limits.* 파라미터를 로딩한다.
    load_joint_limits_from_overrides();

    // wall-time 기반 고정 주기 타이머(best-effort)
    using namespace std::chrono;
    const auto timer_period = duration_cast<nanoseconds>(duration<double>(expected_dt_sec_));
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&RbControllerNode::on_control_tick, this));

    RCLCPP_INFO(
        this->get_logger(),
        "rb_controller started: rate=%.1fHz expected_dt=%.6fs input=%s imu=%s output=%s joint_names(init)=%zu signal_mode=%s target_joint=%s amp=%.3f freq=%.3f safety=%s limits=%zu",
        control_rate_hz_, expected_dt_sec_, input_topic_.c_str(), imu_topic_.c_str(), output_topic_.c_str(), joint_names_.size(),
        signal_mode_.c_str(), target_joint_.c_str(), effort_amplitude_, effort_frequency_hz_,
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
   * @brief /rb/joint_states에서 joint name 순서를 받아 내부 캐시를 갱신한다.
   * @param msg 수신된 JointState 메시지
   */
  // /rb/joint_states 수신 콜백: 조인트 이름/순서를 최신 상태로 유지
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // name[]이 비어 있으면 순서 정합에 사용할 수 없어서 무시
    if (!msg || msg->name.empty())
    {
      return;
    }

    // 조인트 이름/순서가 바뀌면 갱신
    if (joint_names_ != msg->name)
    {
      joint_names_ = msg->name;
      RCLCPP_INFO(
          this->get_logger(),
          "joint_names updated from /rb/joint_states: count=%zu", joint_names_.size());
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
   * @brief /rb/imu 수신 콜백: 기울기(roll/pitch) 추정을 위한 최신 자세를 저장한다.
   * @param msg 수신된 IMU 메시지
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
   * @brief 고정 주기 제어 콜백.
   *
   * 루프 dt를 측정해 통계를 누적하고, zero command를 /rb/command_raw로 퍼블리시한다.
   * 주기적으로 timing 통계 로그를 출력한다.
   */
  void on_control_tick()
  {
    const auto now_steady = std::chrono::steady_clock::now();

    // 루프 간격(dt) 측정 및 miss 카운트
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

    // 기본은 zero command, 필요 시 파라미터 기반 excitation을 추가한다.
    sensor_msgs::msg::JointState cmd_msg;
    cmd_msg.header.stamp = this->now();
    cmd_msg.name = joint_names_;

    const std::size_t joint_count = joint_names_.size();
    cmd_msg.position.assign(joint_count, 0.0);
    cmd_msg.velocity.assign(joint_count, 0.0);
    cmd_msg.effort.assign(joint_count, 0.0);
    apply_excitation(cmd_msg, now_steady);
    apply_safety_filters(cmd_msg, now_steady);

    if (joint_count == 0U)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "joint_names is empty. Waiting for /rb/joint_states names before meaningful command publication.");
    }

    command_pub_->publish(cmd_msg);

    // 설정한 주기(log_interval_sec)마다 타이밍 통계를 출력
    const double log_elapsed = std::chrono::duration<double>(now_steady - steady_last_log_).count();
    if (log_elapsed >= log_interval_sec_)
    {
      log_timing_stats();
      steady_last_log_ = now_steady;
      dt_samples_sec_.clear();
      miss_count_window_ = 0;
      clamp_hit_window_ = 0;
      joint_limit_hit_window_ = 0;
      timeout_event_window_ = 0;
      tilt_event_window_ = 0;
    }
  }

  /**
   * @brief 누적된 dt 샘플의 통계를 계산해 로그로 출력한다.
   *
   * 출력 항목: dt_mean, dt_max, dt_p95, miss_window, miss_total, samples
   */
  void log_timing_stats()
  {
    // 수집된 dt 샘플이 없으면 통계 생략
    if (dt_samples_sec_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "timing window has no samples yet");
      return;
    }

    // dt_mean / dt_max / dt_p95 / miss 카운트 출력
    const double sum = std::accumulate(dt_samples_sec_.begin(), dt_samples_sec_.end(), 0.0);
    const double mean = sum / static_cast<double>(dt_samples_sec_.size());
    const double max_dt = *std::max_element(dt_samples_sec_.begin(), dt_samples_sec_.end());
    const double p95 = percentile(dt_samples_sec_, 0.95);

    RCLCPP_INFO(
        this->get_logger(),
        "loop_stats dt_mean=%.6f dt_max=%.6f dt_p95=%.6f miss_window=%zu miss_total=%zu samples=%zu safety=%s clamp_hit_w=%zu clamp_hit_t=%zu joint_limit_hit_w=%zu joint_limit_hit_t=%zu timeout_evt_w=%zu timeout_evt_t=%zu tilt_evt_w=%zu tilt_evt_t=%zu",
        mean, max_dt, p95, miss_count_window_, miss_count_total_, dt_samples_sec_.size(),
        safety_reason_to_string(last_safety_reason_),
        clamp_hit_window_, clamp_hit_total_,
        joint_limit_hit_window_, joint_limit_hit_total_,
        timeout_event_window_, timeout_event_total_,
        tilt_event_window_, tilt_event_total_);
  }

  /**
   * @brief percentile 값을 계산한다.
   * @param values 입력 샘플 벡터
   * @param ratio 백분위 비율(0.0~1.0)
   * @return 계산된 백분위 값
   */
  static double percentile(const std::vector<double> &values, double ratio)
  {
    // 간단한 percentile 계산 유틸(정렬 후 인덱스 선택)
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

  /**
   * @brief 파라미터 기반 테스트 신호를 command 메시지에 주입한다.
   * @param cmd_msg publish할 JointState 명령
   * @param now_steady 현재 steady clock 시각
   */
  void apply_excitation(
      sensor_msgs::msg::JointState &cmd_msg,
      const std::chrono::steady_clock::time_point &now_steady)
  {
    const std::size_t joint_count = cmd_msg.name.size();
    if (joint_count == 0U || signal_mode_ == "zero")
    {
      return;
    }

    const auto it = std::find(cmd_msg.name.begin(), cmd_msg.name.end(), target_joint_);
    if (it == cmd_msg.name.end())
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "target_joint '%s' not found in joint_names", target_joint_.c_str());
      return;
    }
    const std::size_t idx = static_cast<std::size_t>(std::distance(cmd_msg.name.begin(), it));

    if (!excitation_started_)
    {
      excitation_started_ = true;
      excitation_start_steady_ = now_steady;
    }
    const double elapsed_sec =
        std::chrono::duration<double>(now_steady - excitation_start_steady_).count();

    if (signal_mode_ == "sine_effort")
    {
      constexpr double kTwoPi = 6.283185307179586;
      const double cmd = effort_amplitude_ * std::sin(kTwoPi * effort_frequency_hz_ * elapsed_sec);
      cmd_msg.effort[idx] = cmd;
      return;
    }

    if (signal_mode_ == "step_effort")
    {
      cmd_msg.effort[idx] = (elapsed_sec >= step_start_sec_) ? effort_amplitude_ : 0.0;
      return;
    }

    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "unknown signal_mode '%s' (supported: zero|sine_effort|step_effort)", signal_mode_.c_str());
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
   * @return 하나 이상 clamp가 발생했으면 true
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
   * @return 하나 이상 joint limit guard가 동작했으면 true
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

      // 하한 근처에서 추가 음(-) 토크를 막는다.
      if (position <= soft_lower && effort_cmd < 0.0)
      {
        effort_cmd = 0.0;
        limited = true;
      }
      // 상한 근처에서 추가 양(+) 토크를 막는다.
      if (position >= soft_upper && effort_cmd > 0.0)
      {
        effort_cmd = 0.0;
        limited = true;
      }
      // 이미 하드 리밋을 넘은 경우에는 방향과 무관하게 effort를 차단한다.
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

      bool is_lower = false;
      std::size_t suffix_size = 0U;
      if (full_name.size() > lower_suffix.size() &&
          full_name.compare(full_name.size() - lower_suffix.size(), lower_suffix.size(), lower_suffix) == 0)
      {
        is_lower = true;
        suffix_size = lower_suffix.size();
      }
      else if (full_name.size() > upper_suffix.size() &&
               full_name.compare(full_name.size() - upper_suffix.size(), upper_suffix.size(), upper_suffix) == 0)
      {
        is_lower = false;
        suffix_size = upper_suffix.size();
      }
      else
      {
        continue;
      }

      const std::size_t joint_name_len = full_name.size() - prefix.size() - suffix_size;
      if (joint_name_len == 0U)
      {
        continue;
      }
      const std::string joint_name = full_name.substr(prefix.size(), joint_name_len);

      double numeric_value = 0.0;
      const auto type = value.get_type();
      if (type == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        numeric_value = value.get<double>();
      }
      else if (type == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        numeric_value = static_cast<double>(value.get<std::int64_t>());
      }
      else
      {
        continue;
      }

      auto & bound = joint_limits_[joint_name];
      if (is_lower)
      {
        bound.lower = numeric_value;
        bound.has_lower = true;
      }
      else
      {
        bound.upper = numeric_value;
        bound.has_upper = true;
      }
    }

    for (auto it = joint_limits_.begin(); it != joint_limits_.end();)
    {
      if (!(it->second.has_lower && it->second.has_upper))
      {
        it = joint_limits_.erase(it);
      }
      else
      {
        ++it;
      }
    }

    if (joint_limits_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "joint_limits are empty. joint limit guard disabled");
    }
  }

  // ===== 런타임 파라미터/설정값 =====
  double control_rate_hz_{200.0};
  double log_interval_sec_{5.0};
  double miss_ratio_threshold_{1.2};
  double expected_dt_sec_{0.005};
  std::string input_topic_;
  std::string output_topic_;
  std::string signal_mode_{"zero"};
  std::string target_joint_{"torso_joint"};
  double effort_amplitude_{0.0};
  double effort_frequency_hz_{0.5};
  double step_start_sec_{1.0};
  bool safety_enabled_{false};
  std::string safe_mode_action_{"zero_effort"};
  double effort_abs_max_default_{8.0};
  double joint_limit_margin_rad_{0.05};
  double input_timeout_sec_{0.15};
  double tilt_limit_roll_rad_{0.35};
  double tilt_limit_pitch_rad_{0.35};
  std::string imu_topic_{"/rb/imu"};

  // ===== 상태 캐시/통계 버퍼 =====
  std::vector<std::string> joint_names_;
  std::vector<double> dt_samples_sec_;
  std::unordered_map<std::string, double> latest_joint_positions_;
  std::unordered_map<std::string, JointLimitBound> joint_limits_;

  std::size_t miss_count_total_{0};
  std::size_t miss_count_window_{0};
  std::size_t clamp_hit_total_{0};
  std::size_t clamp_hit_window_{0};
  std::size_t joint_limit_hit_total_{0};
  std::size_t joint_limit_hit_window_{0};
  std::size_t timeout_event_total_{0};
  std::size_t timeout_event_window_{0};
  std::size_t tilt_event_total_{0};
  std::size_t tilt_event_window_{0};
  SafetyReason last_safety_reason_{SafetyReason::NORMAL};

  // ===== 시간 기준점 =====
  std::chrono::steady_clock::time_point node_start_steady_;
  std::chrono::steady_clock::time_point steady_prev_tick_;
  std::chrono::steady_clock::time_point steady_last_log_;
  std::chrono::steady_clock::time_point excitation_start_steady_;
  std::chrono::steady_clock::time_point latest_joint_state_time_steady_;
  std::chrono::steady_clock::time_point latest_imu_time_steady_;
  bool excitation_started_{false};
  bool latest_joint_state_received_{false};
  bool latest_imu_received_{false};
  double latest_roll_rad_{0.0};
  double latest_pitch_rad_{0.0};

  // ===== ROS2 통신 핸들 =====
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv)
{
  // ROS2 초기화 -> 노드 생성/스핀 -> 종료
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RbControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
