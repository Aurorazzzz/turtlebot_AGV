#pragma once

#include <algorithm>
#include <cstdint>

class MotorPwmDir
{
public:
  // pi = handle retourné par pigpio_start() (dans TurtleBot3)
  MotorPwmDir(int pi, int pwm_left, int dir_left, int pwm_right, int dir_right, int pwm_freq_hz = 500);

  bool init();

  // duty signé [-255..255] : signe => direction, magnitude => PWM
  void setSignedDuty(int left, int right);

  void stop();

private:
  int pi_{-1};

  int pwm_left_;
  int dir_left_;
  int pwm_right_;
  int dir_right_;
  int freq_hz_;
  bool initialized_{false};

  static int clamp255_(int x) { return std::clamp(x, -255, 255); }
};

