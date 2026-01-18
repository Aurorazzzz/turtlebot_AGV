#include "turtlebot3_node/sensors/joint_state.hpp"

#include <utility>


using robotis::turtlebot3::sensors::JointState;

JointState::JointState(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<WheelFrequencyReader> wheel_reader,
  std::shared_ptr<WheelCommandState> wheel_cmd_state,
  const std::string & topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id),
  wheel_reader_(std::move(wheel_reader)),
  wheel_cmd_state_(std::move(wheel_cmd_state)),
  pos_left_(0.0),
  pos_right_(0.0),
  has_last_time_(false)
{
  pub_ = nh->create_publisher<sensor_msgs::msg::JointState>(topic_name, this->qos_);
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create joint state publisher (freq + external sign)");
}

void JointState::publish(const rclcpp::Time & now)
{
  auto msg = std::make_unique<sensor_msgs::msg::JointState>();

  msg->header.stamp = now;
  msg->header.frame_id = frame_id_;
  msg->name = {wheel_left_joint_, wheel_right_joint_};

  // Lecture des fréquences (Hz)
  double fL = 0.0, fR = 0.0;
  if (wheel_reader_) {
    auto fr = wheel_reader_->getFrequenciesHz();
    fL = fr.first;
    fR = fr.second;
  }

  // Lecture des signes
  int sL = 0, sR = 0;
  if (wheel_cmd_state_) {
    sL = wheel_cmd_state_->sign_left.load();
    sR = wheel_cmd_state_->sign_right.load();
  }

  // Valeurs signées : on publie la fréquence signée (debug-friendly)
  const double fL_signed = fL * static_cast<double>(sL);
  const double fR_signed = fR * static_cast<double>(sR);

  constexpr double PI = 3.14159265358979323846;

  double wL = (fL_signed*1.3498 - 1.5155)*2*PI;
  double wR = (fR_signed*1.3498 - 1.5155)*2*PI;

  double dt = 0.0;
  if (!has_last_time_) {
    last_time_ = now;
    has_last_time_ = true;
  } else {
    dt = (now - last_time_).seconds();
    last_time_ = now;
  }

  // fL_signed et fR_signed sont déjà en rad/s
  if (dt > 0.0 && std::isfinite(dt)) {
    pos_left_  += wL * dt;
    pos_right_ += wR * dt;
  }


  // Position : pour l'instant on ne remplit pas (vous pourrez intégrer plus tard)
  msg->position = {pos_left_, pos_right_};  // vide

  // Velocity : ici = fréquence signée (Hz), temporaire tant que la conversion n’est pas calibrée
  msg->velocity = {wL, wR};

  pub_->publish(std::move(msg));
}
