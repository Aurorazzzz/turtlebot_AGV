// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim
// Modified for custom robot (pigpio + ADC + wheel freq + PWM motors)
#ifndef TURTLEBOT3_NODE__TURTLEBOT3_HPP_
#define TURTLEBOT3_NODE__TURTLEBOT3_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "turtlebot3_node/twist_subscriber.hpp"

#include "turtlebot3_node/sensors/sensors.hpp"
#include "turtlebot3_node/sensors/joint_state.hpp"
#include "turtlebot3_node/sensors/battery_state.hpp"

#include "turtlebot3_node/sensors/AdcReader.hpp"
#include "turtlebot3_node/sensors/wheel_frequency_reader.hpp"
#include "turtlebot3_node/sensors/wheel_command_state.hpp"

#include "turtlebot3_node/devices/motor_pwm_dir.hpp"

namespace robotis
{
namespace turtlebot3
{

class TurtleBot3 : public rclcpp::Node
{
public:
  explicit TurtleBot3(const std::string & usb_port = "");
  ~TurtleBot3() override;

  struct Wheels
  {
    float separation = 0.52f;
    float radius = 0.0855f;
  };

  Wheels * get_wheels();

private:
  void init_hardware();
  void add_wheels();
  void add_sensors();

  void run();
  void publish_timer(const std::chrono::milliseconds timeout);
  void cmd_vel_callback();

private:
  std::shared_ptr<rclcpp::Node> node_handle_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<TwistSubscriber> cmd_vel_sub_;

  Wheels wheels_;

  // pigpiod_if2 handle (pigpio_start / pigpio_stop)
  int pi_{-1};

  // ✅ no leak: sensors owned here
  std::vector<std::unique_ptr<sensors::Sensors>> sensors_;

  // --- custom hardware backends ---
  std::shared_ptr<AdcReader> adc_reader_;
  std::shared_ptr<WheelFrequencyReader> wheel_reader_;
  std::shared_ptr<WheelCommandState> wheel_cmd_state_;

  std::unique_ptr<MotorPwmDir> motor_driver_;
};

}  // namespace turtlebot3
}  // namespace robotis

#endif  // TURTLEBOT3_NODE__TURTLEBOT3_HPP_
