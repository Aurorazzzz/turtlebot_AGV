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

#include "turtlebot3_node/sensors/battery_state.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::sensors::BatteryState;

BatteryState::BatteryState(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<AdcReader> adc,
  const std::string & topic_name)
: Sensors(nh),
  adc_(std::move(adc))
{
  pub_ = nh->create_publisher<sensor_msgs::msg::BatteryState>(topic_name, this->qos_);
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create battery state publisher (ADC backend)");
}

void BatteryState::publish(
  const rclcpp::Time & now)
{
  auto msg = std::make_unique<sensor_msgs::msg::BatteryState>();
  msg->header.stamp = now;

  // Capacité nominale
  msg->design_capacity = design_capacity_ah_;

  // Valeurs par défaut si la lecture échoue
  msg->present = false;
  msg->voltage = 0.0f;
  msg->current = 0.0f;
  msg->percentage = 0.0f;

  if (!adc_) {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
                         "AdcReader is null, cannot publish battery state");
    pub_->publish(std::move(msg));
    return;
  }

  // Lecture ADC (CH1 + CH4 comme dans AdcReader)
  if (!adc_->update()) {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
                         "ADC update failed, publishing invalid battery state");
    pub_->publish(std::move(msg));
    return;
  }

  const auto s = adc_->getLatest();
  if (!s.valid) {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
                         "ADC sample invalid, publishing invalid battery state");
    pub_->publish(std::move(msg));
    return;
  }

  // --- Conversion tension ---
  // Hypothèse: CH1 mesure la tension *après* pont diviseur (Vbat_div).
  const double vbat = AdcReader::applyVoltageDivider(s.ch1_V, r_top_ohm_, r_bottom_ohm_);
  msg->voltage = static_cast<float>(vbat);

  // --- Conversion courant ---
  // Hypothèse: CH4 mesure Vsense (tension shunt éventuellement amplifiée).
  // Si votre signal est déjà "courant" (ex INA donnant V = k*I), adaptez ici.
  const double current_a = AdcReader::shuntToCurrent(s.ch4_V, shunt_ohm_, current_gain_);
  msg->current = static_cast<float>(current_a);

  // --- Présence batterie ---
  msg->present = (msg->voltage > present_voltage_threshold_v_);

  // --- Pourcentage (SOC) ---
  // IMPORTANT: si vous n’avez pas de fuel gauge, un SOC basé sur tension est approximatif.
  // Ici je laisse à 0 par défaut. Vous pourrez :
  //  - soit calculer via table Li-ion (OCV->SOC) filtrée,
  //  - soit via coulomb counting si vous l’implémentez.
  // msg->percentage = ... (doit être entre 0.0 et 1.0)

  pub_->publish(std::move(msg));
}
