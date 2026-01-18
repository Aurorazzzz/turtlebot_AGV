#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "turtlebot3_node/sensors/sensors.hpp"
#include "turtlebot3_node/sensors/wheel_frequency_reader.hpp"
#include "turtlebot3_node/sensors/wheel_command_state.hpp"

namespace robotis::turtlebot3::sensors
{

class JointState : public Sensors
{
public:
  explicit JointState(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<WheelFrequencyReader> wheel_reader,
    std::shared_ptr<WheelCommandState> wheel_cmd_state,
    const std::string & topic_name = "joint_states",
    const std::string & frame_id = "");

  void publish(const rclcpp::Time & now) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  std::shared_ptr<WheelFrequencyReader> wheel_reader_;
  std::shared_ptr<WheelCommandState> wheel_cmd_state_;

  std::string wheel_left_joint_  = "wheel_left_joint";
  std::string wheel_right_joint_ = "wheel_right_joint";

  double pos_left_{0.0};
  double pos_right_{0.0};

  rclcpp::Time last_time_;
  bool has_last_time_{false};

  // Optionnel : si vous voulez aussi une position, on peut intégrer plus tard
  // (mais ça demandera de convertir en rad/s ou d’accepter une "position brute").
};

}  // namespace robotis::turtlebot3::sensors
