#include "turtlebot3_node/devices/motor_pwm_dir.hpp"

#include <pigpio.h>
#include <cstdio>
#include <cstdlib>

MotorPwmDir::MotorPwmDir(int pwm_left, int dir_left, int pwm_right, int dir_right, int pwm_freq_hz)
: pwm_left_(pwm_left),
  dir_left_(dir_left),
  pwm_right_(pwm_right),
  dir_right_(dir_right),
  freq_hz_(pwm_freq_hz)
{
}

bool MotorPwmDir::init()
{
  if (initialized_) return true;

  // Vérifie pigpio global
  if (gpioHardwareRevision() == 0) {
    std::fprintf(stderr, "MotorPwmDir::init(): pigpio not initialized\n");
    return false;
  }

  // Modes
  gpioSetMode(pwm_left_, PI_OUTPUT);
  gpioSetMode(pwm_right_, PI_OUTPUT);
  gpioSetMode(dir_left_, PI_OUTPUT);
  gpioSetMode(dir_right_, PI_OUTPUT);

  // PWM config
  gpioSetPWMrange(pwm_left_, 255);
  gpioSetPWMrange(pwm_right_, 255);

  gpioSetPWMfrequency(pwm_left_, freq_hz_);
  gpioSetPWMfrequency(pwm_right_, freq_hz_);

  // Etat sûr : stop + direction par défaut
  gpioWrite(dir_left_, 0);
  gpioWrite(dir_right_, 0);
  gpioPWM(pwm_left_, 0);
  gpioPWM(pwm_right_, 0);

  initialized_ = true;
  return true;
}

void MotorPwmDir::setSignedDuty(int left, int right)
{
  if (!initialized_) return;

  left = clamp255_(left);
  right = clamp255_(right);

  // Direction : 1 = avant, 0 = arrière (à adapter si votre driver est inversé)
  const int left_dir = (left >= 0) ? 1 : 0;
  const int right_dir = (right >= 0) ? 1 : 0;

  const int left_pwm = std::abs(left);
  const int right_pwm = std::abs(right);

  gpioWrite(dir_left_, left_dir);
  gpioWrite(dir_right_, right_dir);

  gpioPWM(pwm_left_, left_pwm);
  gpioPWM(pwm_right_, right_pwm);
}

void MotorPwmDir::stop()
{
  if (!initialized_) return;
  gpioPWM(pwm_left_, 0);
  gpioPWM(pwm_right_, 0);
}
