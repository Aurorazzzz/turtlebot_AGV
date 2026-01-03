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
#include <pigpio.h>

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
  //check_device_status();
  add_sensors();

  run();
}

TurtleBot3::~TurtleBot3()
{
  const int OE_GPIO = 0;
  gpioWrite(OE_GPIO, 0);
  gpioTerminate();
}

TurtleBot3::Wheels * TurtleBot3::get_wheels()
{
  return &wheels_;
}

TurtleBot3::Motors * TurtleBot3::get_motors()
{
  return &motors_;
}

// + vos headers WheelFrequencyReader etc.

void TurtleBot3::init_hardware()
{
  RCLCPP_INFO(this->get_logger(), "Init hardware (pigpio, GPIO OE, sensors)");

  // 1) Init pigpio (une seule fois)
  if (gpioInitialise() < 0) {
    RCLCPP_ERROR(this->get_logger(), "pigpio init failed");
    rclcpp::shutdown();
    return;
  }

  // 2) OE: GPIO0 à 1
  const int OE_GPIO = 0;
  gpioSetMode(OE_GPIO, PI_OUTPUT);
  gpioWrite(OE_GPIO, 1);

  // 3) Init ADC reader
  // Exemple : enable_gpio = -1 si vous n’avez pas de GPIO enable spécifique
  adc_reader_ = std::make_shared<AdcReader>(
    "/dev/i2c-1",   // bus I2C
    0x68,           // adresse MCP3428 (à confirmer)
    -1              // ou un GPIO si vous en avez un
  );

  if (!adc_reader_->init()) {
    RCLCPP_ERROR(this->get_logger(), "AdcReader init failed");
    rclcpp::shutdown();
    return;
  }

  // 3) Init WheelFrequencyReader (sans gpioInitialise dedans)
  wheel_reader_ = std::make_shared<WheelFrequencyReader>(23, 27);
  if (!wheel_reader_->init()) {   // je vous explique juste après
    RCLCPP_ERROR(this->get_logger(), "WheelFrequencyReader init failed");
    rclcpp::shutdown();
    return;
  }

  // Moteurs : PWM12+DIR25 (gauche), PWM13+DIR24 (droite)
  motor_driver_ = std::make_unique<MotorPwmDir>(12, 25, 13, 24, 20000);
  if (!motor_driver_->init()) {
  RCLCPP_ERROR(get_logger(), "Motor driver init failed");
  rclcpp::shutdown();
  return;

  // 4) Etat partagé pour le signe (mis à jour par votre futur MotorCommand)
  wheel_cmd_state_ = std::make_shared<WheelCommandState>();
}
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

  // --- Battery ADC ---
  // adc_reader_ créé dans init_hardware() idéalement; sinon ici.
  // Assurez-vous qu’il est déjà initialisé.
  sensors_.push_back(
    new sensors::BatteryState(
      node_handle_,
      adc_reader_,          // <-- votre adc reader injecté
      "battery_state"));

  // --- Joint states (wheel frequency + external sign) ---
  sensors_.push_back(
    new sensors::JointState(
      node_handle_,
      wheel_reader_,        // <-- WheelFrequencyReader (GPIO 23/27)
      wheel_cmd_state_,     // <-- WheelCommandState (signes)
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


// void TurtleBot3::heartbeat_timer(const std::chrono::milliseconds timeout)
// {
//   heartbeat_timer_ = this->create_wall_timer(
//     timeout,
//     [this]() -> void
//     {
//       static uint8_t count = 0;
//       std::string msg;

//       dxl_sdk_wrapper_->set_data_to_device(
//         extern_control_table.heartbeat.addr,
//         extern_control_table.heartbeat.length,
//         &count,
//         &msg);

//       RCLCPP_DEBUG(this->get_logger(), "hearbeat count : %d, msg : %s", count, msg.c_str());

//       count++;
//     }
//   );
// }

// void TurtleBot3::parameter_event_callback()
// {
//   priv_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
//   while (!priv_parameters_client_->wait_for_service(std::chrono::seconds(1))) {
//     if (!rclcpp::ok()) {
//       RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
//       return;
//     }

//     RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
//   }

//   auto param_event_callback =
//     [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
//     {
//       for (const auto & changed_parameter : event->changed_parameters) {
//         RCLCPP_DEBUG(
//           this->get_logger(),
//           "changed parameter name : %s",
//           changed_parameter.name.c_str());

//         if (changed_parameter.name == "motors.profile_acceleration") {
//           std::string sdk_msg;

//           motors_.profile_acceleration =
//             rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();

//           motors_.profile_acceleration =
//             motors_.profile_acceleration / motors_.profile_acceleration_constant;

//           union Data {
//             int32_t dword[2];
//             uint8_t byte[4 * 2];
//           } data;

//           data.dword[0] = static_cast<int32_t>(motors_.profile_acceleration);
//           data.dword[1] = static_cast<int32_t>(motors_.profile_acceleration);

//           uint16_t start_addr = extern_control_table.profile_acceleration_left.addr;
//           uint16_t addr_length =
//             (extern_control_table.profile_acceleration_right.addr -
//             extern_control_table.profile_acceleration_left.addr) +
//             extern_control_table.profile_acceleration_right.length;

//           uint8_t * p_data = &data.byte[0];

//           dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

//           RCLCPP_INFO(
//             this->get_logger(),
//             "changed parameter value : %f [rev/min2] sdk_msg : %s",
//             motors_.profile_acceleration,
//             sdk_msg.c_str());
//         }
//       }
//     };

//   parameter_event_sub_ = priv_parameters_client_->on_parameter_event(param_event_callback);
// }


void TurtleBot3::cmd_vel_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Paramètres simples (sans calibration)
  // gain_v : conversion m/s -> duty(0..255)
  // gain_w : conversion rad/s -> duty(0..255) via (w * L/2)
  this->declare_parameter<double>("motors.gain_v", 200.0);
  this->declare_parameter<double>("motors.gain_w", 200.0);

  auto handle_twist = [this](double v, double w) -> void
  {
    if (!motor_driver_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "motor_driver_ is null, ignoring cmd_vel");
      return;
    }

    // Récupère les gains
    double gain_v = 200.0;
    double gain_w = 200.0;
    this->get_parameter("motors.gain_v", gain_v);
    this->get_parameter("motors.gain_w", gain_w);

    // Diff drive : vitesses linéaires roues (m/s)
    const double L = wheels_.separation; // 0.52 chez vous
    const double vL = v - 0.5 * w * L;
    const double vR = v + 0.5 * w * L;

    // Mapping "sans calibration" vers duty signé
    auto to_duty = [](double x, double k) -> int {
      const double d = x * k;
      const double clamped = std::max(-255.0, std::min(255.0, d));
      return static_cast<int>(std::lround(clamped));
    };

    // Ici vous pouvez choisir :
    // - tout passer par gain_v uniquement
    // - ou séparer v et w (mais en pratique vL/vR suffit)
    const int dutyL = to_duty(vL, gain_v);
    const int dutyR = to_duty(vR, gain_v);

    // Applique au driver (PWM+DIR)
    motor_driver_->setSignedDuty(dutyL, dutyR);

    // Met à jour le sens (pour JointState)
    auto sgn = [](int d) -> int { return (d > 0) - (d < 0); };

    if (wheel_cmd_state_) {
      wheel_cmd_state_->sign_left.store(sgn(dutyL));
      wheel_cmd_state_->sign_right.store(sgn(dutyR));
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "cmd_vel v=%.3f w=%.3f -> vL=%.3f vR=%.3f dutyL=%d dutyR=%d",
                 v, w, vL, vR, dutyL, dutyR);
  };

  // Twist
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
