#include "controller_tilt_observer.hpp"

#include <cmath>

namespace rb_estimation::internal
{

void TiltObserver::update_from_imu(const sensor_msgs::msg::Imu & msg)
{
  // ROS IMU quaternion을 먼저 일반적인 roll/pitch로 풀어낸다.
  const double x = msg.orientation.x;
  const double y = msg.orientation.y;
  const double z = msg.orientation.z;
  const double w = msg.orientation.w;

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  const double pitch = (std::abs(sinp) >= 1.0)
    ? std::copysign(1.5707963267948966, sinp)
    : std::asin(sinp);

  raw_roll_rad_ = roll;
  raw_pitch_rad_ = pitch;

  // G1 IMU raw frame는 controller가 기대하는 control frame과 x/y 축이 다르다.
  // raw quaternion에서 바로 푼 roll/pitch는 "raw IMU frame" 기준 값이고,
  // 여기서 한 번 x/y 축 보정을 적용한 뒤 control-frame standard roll/pitch로 publish한다.
  // 현재 G1 보정은 x/y swap으로 표현되며,
  // control-frame roll/pitch는 각각 raw_pitch/raw_roll에서 얻어진다.
  const double corrected_roll = normalize_angle_rad(pitch);
  const double corrected_pitch = roll;
  tilt_roll_rad_ = corrected_roll;
  tilt_pitch_rad_ = corrected_pitch;
  roll_rate_rad_s_ = msg.angular_velocity.y;
  pitch_rate_rad_s_ = msg.angular_velocity.x;

  received_ = true;
}

bool TiltObserver::received() const
{
  return received_;
}

double TiltObserver::tilt_roll_rad() const
{
  return tilt_roll_rad_;
}

double TiltObserver::tilt_pitch_rad() const
{
  return tilt_pitch_rad_;
}

double TiltObserver::raw_roll_rad() const
{
  return raw_roll_rad_;
}

double TiltObserver::raw_pitch_rad() const
{
  return raw_pitch_rad_;
}

double TiltObserver::roll_rate_rad_s() const
{
  return roll_rate_rad_s_;
}

double TiltObserver::pitch_rate_rad_s() const
{
  return pitch_rate_rad_s_;
}

double TiltObserver::normalize_angle_rad(const double angle_rad)
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

}  // namespace rb_estimation::internal
