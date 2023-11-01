#include "fwsbot/pca9685_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace fwsbot
{
hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get Params
  pca_frequency_ = std::stod(info_.hardware_parameters["pca_frequency"]);
  hw_interfaces_[0].servo.name = info_.hardware_parameters["fl_steer_joint_name"];
  hw_interfaces_[0].servo.channel = std::stoi(info_.hardware_parameters["fl_steer_joint_channel"]);
  hw_interfaces_[0].motor.name = info_.hardware_parameters["fl_drive_joint_name"];
  hw_interfaces_[0].motor.dir_channel = std::stoi(info_.hardware_parameters["fl_drive_joint_dir_channel"]);
  hw_interfaces_[0].motor.pwm_channel = std::stoi(info_.hardware_parameters["fl_drive_joint_pwm_channel"]);

  hw_interfaces_[1].servo.name = info_.hardware_parameters["fr_steer_joint_name"];
  hw_interfaces_[1].servo.channel = std::stoi(info_.hardware_parameters["fr_steer_joint_channel"]);
  hw_interfaces_[1].motor.name = info_.hardware_parameters["fr_drive_joint_name"];
  hw_interfaces_[1].motor.dir_channel = std::stoi(info_.hardware_parameters["fr_drive_joint_dir_channel"]);
  hw_interfaces_[1].motor.pwm_channel = std::stoi(info_.hardware_parameters["fr_drive_joint_pwm_channel"]);

  hw_interfaces_[2].servo.name = info_.hardware_parameters["rl_steer_joint_name"];
  hw_interfaces_[2].servo.channel = std::stoi(info_.hardware_parameters["rl_steer_joint_channel"]);
  hw_interfaces_[2].motor.name = info_.hardware_parameters["rl_drive_joint_name"];
  hw_interfaces_[2].motor.dir_channel = std::stoi(info_.hardware_parameters["rl_drive_joint_dir_channel"]);
  hw_interfaces_[2].motor.pwm_channel = std::stoi(info_.hardware_parameters["rl_drive_joint_pwm_channel"]);

  hw_interfaces_[3].servo.name = info_.hardware_parameters["rr_steer_joint_name"];
  hw_interfaces_[3].servo.channel = std::stoi(info_.hardware_parameters["rr_steer_joint_channel"]);
  hw_interfaces_[3].motor.name = info_.hardware_parameters["rr_drive_joint_name"];
  hw_interfaces_[3].motor.dir_channel = std::stoi(info_.hardware_parameters["rr_drive_joint_dir_channel"]);
  hw_interfaces_[3].motor.pwm_channel = std::stoi(info_.hardware_parameters["rr_drive_joint_pwm_channel"]);

  // Check URDF
  // for (const hardware_interface::ComponentInfo & joint : info_.joints)
  // {
  //   // Check Command Interfaces
  //   // Pca9685System has one command interface on each joint
  //   if (joint.command_interfaces.size() != 1)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("Pca9685SystemHardware"),
  //       "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
  //       joint.command_interfaces.size());
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  //   if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY || 
  //       joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("Pca9685SystemHardware"),
  //       "Joint '%s' have %s command interfaces found. '%s' or '%s' expected.", joint.name.c_str(),
  //       joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_POSITION);
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  //   // Check State Interfaces
  //   if (joint.state_interfaces.size() != 1) {
  //     RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
  //                  "Joint '%s' has %zu state interface. 1 expected.",
  //                  joint.name.c_str(), joint.state_interfaces.size());
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  //   if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY ||
  //       joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
  //     RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
  //                  "Joint '%s' have %s state interface. '%s' or '%s' expected.",
  //                  joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
  //                  hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_POSITION);
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }
  // }

  return hardware_interface::CallbackReturn::SUCCESS;
}
  
hardware_interface::CallbackReturn Pca9685SystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
              "Configuring...");

  pca.set_pwm_freq(pca_frequency_);

  for (int i=0; i< NUM_INTERFACES; i++){
    hw_interfaces_[i].servo.position = 0.0;
    hw_interfaces_[i].motor.velocity = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
              "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (int i = 0; i < NUM_INTERFACES; i++) {
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    hw_interfaces_[i].motor.name, hardware_interface::HW_IF_VELOCITY, &hw_interfaces_[i].motor.velocity));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    hw_interfaces_[i].servo.name, hardware_interface::HW_IF_POSITION, &hw_interfaces_[i].servo.position));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pca9685SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (int i = 0; i < NUM_INTERFACES; i++) {
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    hw_interfaces_[i].motor.name, hardware_interface::HW_IF_VELOCITY, &hw_interfaces_[i].motor.velocity));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    hw_interfaces_[i].servo.name, hardware_interface::HW_IF_POSITION, &hw_interfaces_[i].servo.position));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (int i = 0; i < NUM_INTERFACES; i++) {
    hw_interfaces_[i].servo.position = 0.0;
    hw_interfaces_[i].motor.velocity = 0.0;
  }
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pca9685SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  return hardware_interface::return_type::OK;
}

// double Pca9685SystemHardware::command_to_duty_cycle(double command){
//     double min_input = -1.0;
//     double max_input = 1.0;
//     double clamped_command = std::clamp(command, min_input, max_input);
//     double min_duty_cycle = 0.0;
//     double max_duty_cycle = 5.0;
//     double slope = (max_duty_cycle-min_duty_cycle)/(max_input-min_input);
//     double offset = (max_duty_cycle+min_duty_cycle)/2;
//     return slope * clamped_command + offset;
// }

void Pca9685SystemHardware::set_motor_vel(int pwm_channel, int dir_channel, double velocity){
  if (velocity < 0.0){
    pca.set_pwm(dir_channel, 0, 0xFFFF);
  } else {
    pca.set_pwm(dir_channel, 0, 0);
  }
  double min_velocity = 0.0;
  double max_velocity = 1.0;
  double clamped_velocity = std::clamp(velocity, min_velocity, max_velocity);
  double min_duty_cycle = 0.0;
  double max_duty_cycle = 4095.0;
  double slope = (max_duty_cycle-min_duty_cycle)/(max_velocity-min_velocity);
  double offset = (max_duty_cycle+min_duty_cycle)/2;
  uint16_t duty_cycle = slope * clamped_velocity + offset;
  pca.set_pwm(pwm_channel, 0, duty_cycle);
}

void Pca9685SystemHardware::set_servo_pos(int channel, double angle){
  // Angle is in radians
  double min_angle = 0.0;
  double max_angle = M_PI;
  double clamped_angle = std::clamp(angle, min_angle, max_angle);
  // Convert the angle to a corresponding pulse width between 1 ms and 2 ms
  double min_pulse_width = 1.0;
  double pulse_ms = min_pulse_width + (clamped_angle / max_angle); 
  pca.set_pwm_ms(channel, pulse_width);

}

hardware_interface::return_type Pca9685SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  for (int i = 0; i < NUM_INTERFACES; i++)
  {
    set_motor_vel(hw_interfaces_[i].motor.pwm_channel, hw_interfaces_[i].motor.dir_channel, hw_interfaces_[i].motor.velocity);
    set_servo_pos(hw_interfaces_[i].servo.channel, hw_interfaces_[i].servo.position);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace fwsbot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  fwsbot::Pca9685SystemHardware, hardware_interface::SystemInterface)
