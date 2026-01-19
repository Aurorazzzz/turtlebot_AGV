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

#include "turtlebot3_node/turtlebot3.hpp"

#include <pigpiod_if2.h>  // <-- mode démon

#include "turtlebot3_node/sensors/battery_state.hpp"
#include "turtlebot3_node/sensors/wheel_command_state.hpp"
#include "turtlebot3_node/sensors/wheel_frequency_reader.hpp"

#include <memory>
#include <string>
#include <algorithm>
#include <cmath>

using robotis::turtlebot3::TurtleBot3;
using namespace std::chrono_literals;

TurtleBot3::TurtleBot3(const std::string & usb_port)
: Node("turtlebot3_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Init TurtleBot3 Node Main");
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  init_hardware();
  add_sensors();
  run();
}

TurtleBot3::~TurtleBot3()
{
  const int OE_GPIO = 19;

  // 1) Mise en sécurité : stop moteurs si possible
  if (motor_driver_) {
    motor_driver_->stop();
  }

  // 2) Détruire explicitement les composants qui utilisent pigpio
  //    (annule callbacks, arrête PWM, etc.)
  wheel_reader_.reset();
  motor_driver_.reset();
  adc_reader_.reset();
  wheel_cmd_state_.reset();

  // Si vous avez d'autres objets pigpio (capteurs, etc.), reset ici aussi.

  // 3) Couper OE tant que pigpio est encore actif
  if (pi_ >= 0) {
    gpio_write(pi_, OE_GPIO, 0);

    // 4) Fermer proprement la connexion au démon
    pigpio_stop(pi_);
    pi_ = -1;
  }
}


TurtleBot3::Wheels * TurtleBot3::get_wheels()
{
  return &wheels_;
}

// TurtleBot3::Motors * TurtleBot3::get_motors()
// {
//   return &motors_;
// }

void TurtleBot3::init_hardware()
{
  RCLCPP_INFO(this->get_logger(), "Init hardware (pigpiod_if2, GPIO OE, sensors)");

  // 1) Connexion au démon pigpiod (une seule fois)
  pi_ = pigpio_start(nullptr, nullptr);  // localhost:8888
  if (pi_ < 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "pigpio_start() failed (cannot connect to pigpiod), pi=%d", pi_);
    rclcpp::shutdown();
    return;
  }

  // 2) OE: GPIO0 à 1
  const int OE_GPIO = 19;
  if (set_mode(pi_, OE_GPIO, PI_OUTPUT) != 0) {
    RCLCPP_ERROR(this->get_logger(), "set_mode(OE_GPIO) failed");
    rclcpp::shutdown();
    return;
  }
  gpio_write(pi_, OE_GPIO, 1);

  // 3) Init ADC reader (injecter pi_ si enable_gpio utilisé)
  adc_reader_ = std::make_shared<AdcReader>(
    pi_,          // <-- handle pigpio
    "/dev/i2c-1",
    0x68,
    -1
  );

  if (!adc_reader_->init()) {
    RCLCPP_ERROR(this->get_logger(), "AdcReader init failed");
    rclcpp::shutdown();
    return;
  }

  // 4) Init WheelFrequencyReader (callbacks via pigpiod_if2)
  wheel_reader_ = std::make_shared<WheelFrequencyReader>(pi_, 23, 27);
  if (!wheel_reader_->init()) {
    RCLCPP_ERROR(this->get_logger(), "WheelFrequencyReader init failed");
    rclcpp::shutdown();
    return;
  }

  // 5) Moteurs : PWM12+DIR25 (gauche), PWM13+DIR24 (droite)
  motor_driver_ = std::make_unique<MotorPwmDir>(pi_, 12, 25, 13, 24, 20000);
  if (!motor_driver_->init()) {
    RCLCPP_ERROR(get_logger(), "Motor driver init failed");
    rclcpp::shutdown();
    return;
  }

  // 6) Etat partagé pour le signe
  wheel_cmd_state_ = std::make_shared<WheelCommandState>();
}

void TurtleBot3::add_wheels()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels");

  this->declare_parameter<float>("wheels.separation");
  this->declare_parameter<float>("wheels.radius");

  this->get_parameter_or<float>("wheels.separation", wheels_.separation, 0.52);
  this->get_parameter_or<float>("wheels.radius", wheels_.radius, 0.0855);
}

void TurtleBot3::add_sensors()
{
  RCLCPP_INFO(this->get_logger(), "Add Sensors (custom)");

  sensors_.push_back(
    std::make_unique<sensors::BatteryState>(
      node_handle_,
      adc_reader_,
      "battery_state"));

  sensors_.push_back(
    std::make_unique<sensors::JointState>(
      node_handle_,
      wheel_reader_,
      wheel_cmd_state_,
      "joint_states",
      "base_link"));
}

void TurtleBot3::run()
{
  RCLCPP_INFO(this->get_logger(), "Run!");

  publish_timer(std::chrono::milliseconds(50));
  cmd_vel_callback();
}

void TurtleBot3::publish_timer(const std::chrono::milliseconds timeout)
{
  publish_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      const rclcpp::Time now = this->now();
      for (const auto & sensor : sensors_) {
        sensor->publish(now);
      }
    }
  );
}

void TurtleBot3::cmd_vel_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  this->declare_parameter<double>("motors.gain_v", 200.0);
  this->declare_parameter<double>("motors.gain_w", 200.0);

  auto handle_twist = [this](double v, double w) -> void
  {
    if (!motor_driver_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "motor_driver_ is null, ignoring cmd_vel");
      return;
    }

    double gain_v = 100.0;
    double gain_w = 100.0;
    this->get_parameter("motors.gain_v", gain_v);
    this->get_parameter("motors.gain_w", gain_w);

    const double L = wheels_.separation;
    const double vL = v - 0.5 * w * L;
    const double vR = v + 0.5 * w * L;

    auto to_duty = [](double x, double k) -> int {
      const double d = x * k;
      const double clamped = std::max(-255.0, std::min(255.0, d));
      return static_cast<int>(std::lround(clamped));
    };

    const int dutyL = to_duty(vL, gain_v);
    const int dutyR = to_duty(vR, gain_v);

    motor_driver_->setSignedDuty(dutyL, dutyR);

    auto sgn = [](int d) -> int { return (d > 0) - (d < 0); };
    if (wheel_cmd_state_) {
      wheel_cmd_state_->sign_left.store(sgn(dutyL));
      wheel_cmd_state_->sign_right.store(sgn(dutyR));
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "cmd_vel v=%.3f w=%.3f -> vL=%.3f vR=%.3f dutyL=%d dutyR=%d",
                 v, w, vL, vR, dutyL, dutyR);
  };

  cmd_vel_sub_ = std::make_unique<TwistSubscriber>(
    node_handle_,
    "cmd_vel",
    qos,
    std::function<void(const geometry_msgs::msg::Twist::SharedPtr)>(
      [handle_twist](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        handle_twist(msg->linear.x, msg->angular.z);
      }
    ),
    std::function<void(const geometry_msgs::msg::TwistStamped::SharedPtr)>(
      [handle_twist](const geometry_msgs::msg::TwistStamped::SharedPtr msg) -> void
      {
        handle_twist(msg->twist.linear.x, msg->twist.angular.z);
      }
    )
  );
}
