#ifndef FWSBOT__PCA9685_SYSTEM_HPP_
#define FWSBOT__PCA9685_SYSTEM_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "fwsbot/pca9685_comm.h"
#include "fwsbot/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace fwsbot {

struct DiveJoint
{
  std::string name;
  double velocity;
};

struct SteerJoint
{
  std::string name;
  double position;
};

struct JointPair
{
  DiveJoint  motor;
  SteerJoint servo;
};

class Pca9685SystemHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Pca9685SystemHardware);

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // JointPair: pair is just (Steering Joint,Drive joint)
  // Each joint is a pair of (name, command)
  static constexpr int NUM_INTERFACES = 4;
  JointPair hw_interfaces_[NUM_INTERFACES];
  
  PiPCA9685::PCA9685 pca;
  double pca_frequency_;

  double command_to_duty_cycle(double command);
};

}  // namespace fwsbot

#endif  // FWSBOT__PCA9685_SYSTEM_HPP_
