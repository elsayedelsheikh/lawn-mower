#ifndef RRBOT_CAM_OBJECT_FOLLOWER_HPP_
#define RRBOT_CAM_OBJECT_FOLLOWER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rrbot_cam_msgs/msg/pan_tilt_command.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace rrbot_cam_object_follower {

class ObjectFollower : public rclcpp::Node {
 public:
  ObjectFollower();

 private:
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
      detected_object_sub_;
  rclcpp::Publisher<rrbot_cam_msgs::msg::PanTiltCommand>::SharedPtr
      command_pub_;
  void detected_object_cb(
      const vision_msgs::msg::Detection2DArray::ConstSharedPtr &msg);
  int IMG_WIDTH;
  int IMG_HEIGHT;
};

}  // namespace rrbot_cam_object_follower

#endif  // RRBOT_CAM_OBJECT_FOLLOWER_HPP_