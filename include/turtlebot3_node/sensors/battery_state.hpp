// battery_state.hpp

#ifndef TURTLEBOT3_NODE__SENSORS__BATTERY_STATE_HPP_
#define TURTLEBOT3_NODE__SENSORS__BATTERY_STATE_HPP_

#include <sensor_msgs/msg/battery_state.hpp>

#include <memory>
#include <string>

#include "turtlebot3_node/sensors/sensors.hpp"
#include "turtlebot3_node/sensors/AdcReader.hpp" // <-- adaptez le chemin selon où vous placez AdcReader

namespace robotis
{
namespace turtlebot3
{
namespace sensors
{

class BatteryState : public Sensors
{
public:
  explicit BatteryState(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<AdcReader> adc,
    const std::string & topic_name = "battery_state");

  // On garde la signature d'origine pour rester compatible avec l'interface Sensors,
  // mais on n'utilise plus dxl_sdk_wrapper.
  void publish(
    const rclcpp::Time & now) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_;
  std::shared_ptr<AdcReader> adc_;

  // --- Paramètres à ADAPTER à votre hardware ---
  float design_capacity_ah_ = 1.8f;  // capacité nominale (Ah) -> mettez la vôtre
  float present_voltage_threshold_v_ = 7.0f; // seuil "batterie présente"

  // Exemple: pont diviseur pour la tension batterie (si CH1 mesure Vbat_div)
  // Vbat = Vadc * (Rtop+Rbottom)/Rbottom
  double r_top_ohm_ = 100000.0;
  double r_bottom_ohm_ = 10000.0;

  // Exemple: conversion courant depuis CH4 (si CH4 mesure Vsense après ampli)
  // I = Vsense / (Rshunt * gain)
  double shunt_ohm_ = 0.01;   // 10 mΩ par exemple
  double current_gain_ = 1.0; // gain de l'ampli (ou 1 si direct)
};

}  // namespace sensors
}  // namespace turtlebot3
}  // namespace robotis

#endif  // TURTLEBOT3_NODE__SENSORS__BATTERY_STATE_HPP_
