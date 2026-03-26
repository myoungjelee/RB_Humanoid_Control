#include "controller_tilt_observer.hpp"

#include <cmath>

namespace rb_controller::internal
{

void TiltObserver::update_from_imu(const sensor_msgs::msg::Imu &msg)
{
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
  const double corrected_roll = normalize_angle_rad(roll);
  const double corrected_pitch = pitch;

  // G1 imu_link 축을 control-frame roll/pitch 해석에 맞춘 고정 보정.
  tilt_roll_rad_ = normalize_angle_rad(corrected_pitch);
  tilt_pitch_rad_ = corrected_roll;
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

}  // namespace rb_controller::internal
