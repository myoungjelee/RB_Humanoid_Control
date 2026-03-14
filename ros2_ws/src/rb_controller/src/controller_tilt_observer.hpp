#pragma once

#include <string>

#include "sensor_msgs/msg/imu.hpp"

namespace rb_controller::internal
{

/**
 * @brief IMU 한 샘플 처리 결과를 호출자에게 알려준다.
 */
struct TiltObserverUpdateResult
{
  bool bias_captured_now{false};
  double raw_roll_rad{0.0};
  double raw_pitch_rad{0.0};
  double bias_roll_rad{0.0};
  double bias_pitch_rad{0.0};
};

/**
 * @brief IMU raw/bias/frame compensation을 캡슐화하는 경량 관측기.
 *
 * ROS 노드와 분리해 두면 "센서 해석"과 "제어"를 따로 읽고 수정하기 쉽다.
 */
class TiltObserver
{
public:
  /**
   * @brief startup bias 캡처 사용 여부를 설정한다.
   */
  void set_zero_on_start(bool enabled);

  /**
   * @brief IMU frame compensation 모드를 설정한다.
   *
   * 지원값은 `identity`, `g1_imu_link` 두 가지다.
   * legacy alias로 `standard`, `swap_rp`도 받아서 같은 동작으로 정규화한다.
   */
  void set_frame_mode(const std::string &mode);

  /**
   * @brief bias/수신 상태를 초기화한다.
   */
  void reset_bias();

  /**
   * @brief IMU 샘플 하나를 받아 raw/bias/tilt/rate를 갱신한다.
   */
  TiltObserverUpdateResult update_from_imu(const sensor_msgs::msg::Imu &msg);

  [[nodiscard]] bool received() const;
  [[nodiscard]] bool bias_captured() const;
  [[nodiscard]] double tilt_roll_rad() const;
  [[nodiscard]] double tilt_pitch_rad() const;
  [[nodiscard]] double raw_roll_rad() const;
  [[nodiscard]] double raw_pitch_rad() const;
  [[nodiscard]] double bias_roll_rad() const;
  [[nodiscard]] double bias_pitch_rad() const;
  [[nodiscard]] double roll_rate_rad_s() const;
  [[nodiscard]] double pitch_rate_rad_s() const;

private:
  static double normalize_angle_rad(double angle_rad);

  bool zero_on_start_{false};
  std::string frame_mode_{"identity"};
  bool received_{false};
  bool bias_captured_{false};
  double tilt_roll_rad_{0.0};
  double tilt_pitch_rad_{0.0};
  double raw_roll_rad_{0.0};
  double raw_pitch_rad_{0.0};
  double bias_roll_rad_{0.0};
  double bias_pitch_rad_{0.0};
  double roll_rate_rad_s_{0.0};
  double pitch_rate_rad_s_{0.0};
};

}  // namespace rb_controller::internal
