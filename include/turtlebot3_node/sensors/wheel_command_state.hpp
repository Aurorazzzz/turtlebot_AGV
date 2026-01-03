#pragma once
#include <atomic>

// Sens des roues fourni par votre futur module "MotorCommand"
// -1 = arrière, 0 = stop/neutre, +1 = avant
struct WheelCommandState
{
  std::atomic<int> sign_left{0};
  std::atomic<int> sign_right{0};
};
