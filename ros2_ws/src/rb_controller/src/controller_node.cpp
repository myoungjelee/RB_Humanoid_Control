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

    // 200Hz 기준 expected dt = 0.005s
    expected_dt_sec_ = 1.0 / control_rate_hz_;

    // 제어 명령 publish, joint state subscribe 배선
    command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output_topic_, 10);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        input_topic_, 10,
        std::bind(&RbControllerNode::on_joint_state, this, std::placeholders::_1));

    // wall-time 기반 고정 주기 타이머(best-effort)
    using namespace std::chrono;
    const auto timer_period = duration_cast<nanoseconds>(duration<double>(expected_dt_sec_));
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&RbControllerNode::on_control_tick, this));

    RCLCPP_INFO(
        this->get_logger(),
        "rb_controller started: rate=%.1fHz expected_dt=%.6fs input=%s output=%s joint_names(init)=%zu signal_mode=%s target_joint=%s amp=%.3f freq=%.3f",
        control_rate_hz_, expected_dt_sec_, input_topic_.c_str(), output_topic_.c_str(), joint_names_.size(),
        signal_mode_.c_str(), target_joint_.c_str(), effort_amplitude_, effort_frequency_hz_);
  }

private:
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
        "loop_stats dt_mean=%.6f dt_max=%.6f dt_p95=%.6f miss_window=%zu miss_total=%zu samples=%zu",
        mean, max_dt, p95, miss_count_window_, miss_count_total_, dt_samples_sec_.size());
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

  // ===== 상태 캐시/통계 버퍼 =====
  std::vector<std::string> joint_names_;
  std::vector<double> dt_samples_sec_;

  std::size_t miss_count_total_{0};
  std::size_t miss_count_window_{0};

  // ===== 시간 기준점 =====
  std::chrono::steady_clock::time_point steady_prev_tick_;
  std::chrono::steady_clock::time_point steady_last_log_;
  std::chrono::steady_clock::time_point excitation_start_steady_;
  bool excitation_started_{false};

  // ===== ROS2 통신 핸들 =====
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
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
