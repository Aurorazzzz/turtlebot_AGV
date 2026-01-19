#include "turtlebot3_node/devices/motor_pwm_dir.hpp"

#include <pigpiod_if2.h>

#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <cstdlib>

MotorPwmDir::MotorPwmDir(int pi, int pwm_left, int dir_left, int pwm_right, int dir_right, int pwm_freq_hz)
: pi_(pi),
  pwm_left_(pwm_left),
  dir_left_(dir_left),
  pwm_right_(pwm_right),
  dir_right_(dir_right),
  freq_hz_(pwm_freq_hz)
{
}

bool MotorPwmDir::init()
{
  if (initialized_) return true;

  if (pi_ < 0) {
    std::fprintf(stderr, "MotorPwmDir::init(): invalid pi handle (pi=%d)\n", pi_);
    return false;
  }

  // Modes
  if (set_mode(pi_, pwm_left_, PI_OUTPUT) != 0 ||
      set_mode(pi_, pwm_right_, PI_OUTPUT) != 0 ||
      set_mode(pi_, dir_left_, PI_OUTPUT) != 0 ||
      set_mode(pi_, dir_right_, PI_OUTPUT) != 0) {
    std::fprintf(stderr, "MotorPwmDir::init(): set_mode failed\n");
    return false;
  }

  // PWM range (0..255)
  set_PWM_range(pi_, pwm_left_, 255);
  set_PWM_range(pi_, pwm_right_, 255);

  // PWM frequency
  set_PWM_frequency(pi_, pwm_left_, freq_hz_);
  set_PWM_frequency(pi_, pwm_right_, freq_hz_);

  // Etat sûr : stop + direction par défaut
  gpio_write(pi_, dir_left_, 0);
  gpio_write(pi_, dir_right_, 0);
  set_PWM_dutycycle(pi_, pwm_left_, 0);
  set_PWM_dutycycle(pi_, pwm_right_, 0);

  initialized_ = true;
  return true;
}

void MotorPwmDir::setSignedDuty(int left, int right)
{
  if (!initialized_ || pi_ < 0) return;
	
  //right = -right;
  left = clamp255_(left);
  right = clamp255_(right);

  // Direction : 1 = avant, 0 = arrière (à adapter si inversé)
  const int left_dir  = (left >= 0) ? 1 : 0;
  const int right_dir = (right >= 0) ? 0 : 1;

  RCLCPP_INFO(rclcpp::get_logger("MotorPwmDir"),
            "LEFT %s pwm=%d | RIGHT %s pwm=%d",
            left, left_dir,
            right, right_dir);

  const int left_pwm  = std::abs(left);
  const int right_pwm = std::abs(right);

  gpio_write(pi_, dir_left_, left_dir);
  gpio_write(pi_, dir_right_, right_dir);

  set_PWM_dutycycle(pi_, pwm_left_, left_pwm);
  set_PWM_dutycycle(pi_, pwm_right_, right_pwm);
}

void MotorPwmDir::stop()
{
  if (!initialized_ || pi_ < 0) return;
  set_PWM_dutycycle(pi_, pwm_left_, 0);
  set_PWM_dutycycle(pi_, pwm_right_, 0);
}
