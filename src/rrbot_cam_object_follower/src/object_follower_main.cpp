#include "rclcpp/rclcpp.hpp"
#include "rrbot_cam_object_follower/ObjectFollower.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rrbot_cam_object_follower::ObjectFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
