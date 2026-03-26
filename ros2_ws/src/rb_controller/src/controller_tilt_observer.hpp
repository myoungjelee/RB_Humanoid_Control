#pragma once

#include "sensor_msgs/msg/imu.hpp"

namespace rb_controller::internal
{

/**
 * @brief IMU raw -> G1 control-frame tilt 변환을 캡슐화하는 경량 관측기.
 *
 * ROS 노드와 분리해 두면 "센서 해석"과 "제어"를 따로 읽고 수정하기 쉽다.
 */
class TiltObserver
{
public:
  /**
   * @brief IMU 샘플 하나를 받아 raw/tilt/rate를 갱신한다.
   */
  void update_from_imu(const sensor_msgs::msg::Imu &msg);

  /// 최소 1개 이상의 IMU 샘플을 받아 tilt 결과가 준비됐는지 반환한다.
  [[nodiscard]] bool received() const;
  /// controller가 좌우 기울기 제어에 쓰는 roll 축 tilt 값이다.
  [[nodiscard]] double tilt_roll_rad() const;
  /// controller가 앞뒤 기울기 제어에 쓰는 pitch 축 tilt 값이다.
  [[nodiscard]] double tilt_pitch_rad() const;
  /// IMU quaternion에서 바로 얻은 raw roll 값이다(디버그용).
  [[nodiscard]] double raw_roll_rad() const;
  /// IMU quaternion에서 바로 얻은 raw pitch 값이다(디버그용).
  [[nodiscard]] double raw_pitch_rad() const;
  /// 제어용 roll 축에 대응하는 각속도다.
  [[nodiscard]] double roll_rate_rad_s() const;
  /// 제어용 pitch 축에 대응하는 각속도다.
  [[nodiscard]] double pitch_rate_rad_s() const;

private:
  /// roll 값을 [-pi, pi] 범위로 감아 discontinuity를 줄인다.
  static double normalize_angle_rad(double angle_rad);

  bool received_{false};
  double tilt_roll_rad_{0.0};
  double tilt_pitch_rad_{0.0};
  double raw_roll_rad_{0.0};
  double raw_pitch_rad_{0.0};
  double roll_rate_rad_s_{0.0};
  double pitch_rate_rad_s_{0.0};
};

}  // namespace rb_controller::internal
