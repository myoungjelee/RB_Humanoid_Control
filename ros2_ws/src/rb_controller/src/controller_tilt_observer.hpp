#pragma once

#include <string>

#include "sensor_msgs/msg/imu.hpp"

namespace rb_controller::internal
{

/**
 * @brief IMU raw/frame compensation을 캡슐화하는 경량 관측기.
 *
 * ROS 노드와 분리해 두면 "센서 해석"과 "제어"를 따로 읽고 수정하기 쉽다.
 */
class TiltObserver
{
public:
  /**
   * @brief IMU frame compensation 모드를 설정한다.
   *
   * 지원값은 `identity`, `g1_imu_link` 두 가지다.
   * legacy alias로 `standard`, `swap_rp`도 받아서 같은 동작으로 정규화한다.
   */
  void set_frame_mode(const std::string &mode);

  /**
   * @brief IMU 샘플 하나를 받아 raw/tilt/rate를 갱신한다.
   */
  void update_from_imu(const sensor_msgs::msg::Imu &msg);

  [[nodiscard]] bool received() const;
  [[nodiscard]] double tilt_roll_rad() const;
  [[nodiscard]] double tilt_pitch_rad() const;
  [[nodiscard]] double raw_roll_rad() const;
  [[nodiscard]] double raw_pitch_rad() const;
  [[nodiscard]] double roll_rate_rad_s() const;
  [[nodiscard]] double pitch_rate_rad_s() const;

private:
  static double normalize_angle_rad(double angle_rad);

  std::string frame_mode_{"identity"};
  bool received_{false};
  double tilt_roll_rad_{0.0};
  double tilt_pitch_rad_{0.0};
  double raw_roll_rad_{0.0};
  double raw_pitch_rad_{0.0};
  double roll_rate_rad_s_{0.0};
  double pitch_rate_rad_s_{0.0};
};

}  // namespace rb_controller::internal
