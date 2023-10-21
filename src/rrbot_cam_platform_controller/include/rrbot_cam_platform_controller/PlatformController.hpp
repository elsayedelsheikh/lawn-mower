#ifndef RRBOT_CAM_PLATFORM_CONTROLLER_HPP_
#define RRBOT_CAM_PLATFORM_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rrbot_cam_msgs/msg/pan_tilt_command.hpp"
#include "rrbot_cam_platform_controller/PIDController.hpp"

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"


namespace rrbot_cam_platform_controller {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PlatformController : public rclcpp_lifecycle::LifecycleNode {
 public:
  PlatformController();

  CallbackReturn on_configure(const rclcpp_lifecycle::State &prev_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &prev_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &prev_state);

  void control_cycle();

  void joint_state_callback(control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg);
  void command_callback(rrbot_cam_msgs::msg::PanTiltCommand::UniquePtr msg);

 private:

    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
        joint_state_sub_;

    rclcpp::Subscription<rrbot_cam_msgs::msg::PanTiltCommand>::SharedPtr
        command_sub_;

    rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
        trajectory_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    control_msgs::msg::JointTrajectoryControllerState::UniquePtr last_state_;
    rrbot_cam_msgs::msg::PanTiltCommand::UniquePtr last_command_;
    rclcpp::Time last_command_ts_;

    PIDController pan_pid_, tilt_pid_;
};

}  // namespace rrbot_cam_platform_controller

#endif  // RRBOT_CAM_PLATFORM_CONTROLLER_HPP_