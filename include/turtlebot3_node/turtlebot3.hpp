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

// TB3 utility subscriber (already used in your cpp)
#include "turtlebot3_node/subscriber/twist_subscriber.hpp"

// Sensors base + concrete sensors you instantiate
#include "turtlebot3_node/sensors/sensors.hpp"
#include "turtlebot3_node/sensors/joint_state.hpp"
#include "turtlebot3_node/sensors/battery_state.hpp"

// Your custom backends
#include "turtlebot3_node/sensors/AdcReader.hpp"
#include "turtlebot3_node/sensors/wheel_frequency_reader.hpp"
#include "turtlebot3_node/sensors/wheel_command_state.hpp"

// Motor driver (PWM + DIR)
#include "turtlebot3_node/devices/motor_pwm_dir.hpp"

namespace robotis
{
namespace turtlebot3
{

class TurtleBot3 : public rclcpp::Node
{
public:
  // TB3 signature keeps usb_port, you can ignore it for now
  explicit TurtleBot3(const std::string & usb_port = "");
  ~TurtleBot3() override;

  // Minimal structs kept for wheels params (separation/radius)
  struct Wheels
  {
    float separation = 0.52f;
    float radius = 0.0855f;
  };

  // Kept only if still referenced somewhere else; can be removed later
  Wheels * get_wheels();

private:
  // --- init / setup ---
  void init_hardware();
  void add_wheels();
  void add_sensors();

  // --- runtime ---
  void run();
  void publish_timer(const std::chrono::milliseconds timeout);
  void cmd_vel_callback();

private:
  // Node handle trick used in TB3 to pass shared_ptr<Node> to components
  std::shared_ptr<rclcpp::Node> node_handle_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // cmd_vel subscriber wrapper
  std::unique_ptr<TwistSubscriber> cmd_vel_sub_;

  // Robot parameters
  Wheels wheels_;

  // Sensors container (TB3 style raw pointers)
  std::vector<sensors::Sensors *> sensors_;

  // --- custom hardware backends ---
  std::shared_ptr<sensors::AdcReader> adc_reader_;
  std::shared_ptr<sensors::WheelFrequencyReader> wheel_reader_;
  std::shared_ptr<sensors::WheelCommandState> wheel_cmd_state_;

  // Motor driver
  std::unique_ptr<MotorPwmDir> motor_driver_;
};

}  // namespace turtlebot3
}  // namespace robotis

#endif  // TURTLEBOT3_NODE__TURTLEBOT3_HPP_
