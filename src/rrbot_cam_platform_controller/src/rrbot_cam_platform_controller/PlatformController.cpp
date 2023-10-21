#include "rrbot_cam_platform_controller/PlatformController.hpp"

namespace rrbot_cam_platform_controller {

PlatformController::PlatformController():
LifecycleNode("platform_controller"),
pan_pid_(0.41, 0.06, 0.53, 0.0, 1.0, 0.0, 0.3), // Kp, Ki, Kd, RefMin, RefMax, OutputMin, OutputMax
tilt_pid_(0.41, 0.06, 0.53, 0.0, 1.0, 0.0, 0.3)
{
  command_sub_ = create_subscription<rrbot_cam_msgs::msg::PanTiltCommand>(
      "command", 10, std::bind(&PlatformController::command_callback, this, std::placeholders::_1));
  
  joint_state_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "joint_state", 5, std::bind(&PlatformController::joint_state_callback, this, std::placeholders::_1));
  
  trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10);
}

CallbackReturn
PlatformController::on_configure(const rclcpp_lifecycle::State &prev_state)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  pan_pid_.set_gains(0.4, 0.05, 0.55);
  tilt_pid_.set_gains(0.4, 0.05, 0.55);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
PlatformController::on_activate(const rclcpp_lifecycle::State &prev_state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  trajectory_pub_->on_activate();
  control_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PlatformController::control_cycle, this));
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn
PlatformController::on_deactivate(const rclcpp_lifecycle::State &prev_state)
{
  RCLCPP_INFO(get_logger(), "HeadController deactivated");

  trajectory_msgs::msg::JointTrajectory command_msg;
  command_msg.header.stamp = now();
  command_msg.joint_names = last_state_->joint_names;
  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);
  command_msg.points[0].positions[0] = 0.0;
  command_msg.points[0].positions[1] = 0.0;
  command_msg.points[0].velocities[0] = 0.1;
  command_msg.points[0].velocities[1] = 0.1;
  command_msg.points[0].accelerations[0] = 0.1;
  command_msg.points[0].accelerations[1] = 0.1;
  command_msg.points[0].time_from_start = rclcpp::Duration(std::chrono::milliseconds(1000));

  trajectory_pub_->publish(command_msg);

  trajectory_pub_->on_deactivate();
  control_timer_ = nullptr;

  return CallbackReturn::SUCCESS;
}

void
PlatformController::joint_state_callback(
  control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg)
{
  last_state_ = std::move(msg);
}

void 
PlatformController::command_callback(
  rrbot_cam_msgs::msg::PanTiltCommand::UniquePtr msg)
{
  last_command_ = std::move(msg);
  last_command_ts_ = now();
}

void
PlatformController::control_cycle()
{
  if (last_state_ == nullptr)
    return;
  
  trajectory_msgs::msg::JointTrajectory command_msg;
  command_msg.header.stamp = now();
  command_msg.joint_names = last_state_->joint_names;
  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);
  command_msg.points[0].time_from_start = rclcpp::Duration(std::chrono::milliseconds(200));

  if (last_command_ == nullptr || (now() - last_command_ts_) > std::chrono::milliseconds(100)) {
    command_msg.points[0].positions[0] = 0.0;
    command_msg.points[0].positions[1] = 0.0;
    // command_msg.points[0].velocities[0] = 0.1;
    // command_msg.points[0].velocities[1] = 0.1;
    // command_msg.points[0].accelerations[0] = 0.1;
    // command_msg.points[0].accelerations[1] = 0.1;
    command_msg.points[0].time_from_start = rclcpp::Duration(std::chrono::milliseconds(1000));
  } else {
    double control_pan = pan_pid_.compute(last_command_->pan);
    double control_tilt = tilt_pid_.compute(last_command_->tilt);

    command_msg.points[0].positions[0] = last_state_->actual.positions[0] - control_pan;
    command_msg.points[0].positions[1] = last_state_->actual.positions[1] - control_tilt;

    // command_msg.points[0].velocities[0] = 0.5;
    // command_msg.points[0].velocities[1] = 0.5;
    // command_msg.points[0].accelerations[0] = 0.5;
    // command_msg.points[0].accelerations[1] = 0.5;
  }

  trajectory_pub_->publish(command_msg);

}


}  // namespace rrbot_cam_platform_controller