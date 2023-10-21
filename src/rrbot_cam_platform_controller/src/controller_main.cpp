#include "rclcpp/rclcpp.hpp"
#include "rrbot_cam_platform_controller/PlatformController.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rrbot_cam_platform_controller::PlatformController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
