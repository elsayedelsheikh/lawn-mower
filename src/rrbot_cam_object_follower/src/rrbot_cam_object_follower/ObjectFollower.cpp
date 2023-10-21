#include "rrbot_cam_object_follower/ObjectFollower.hpp"

namespace rrbot_cam_object_follower {

ObjectFollower::ObjectFollower() : Node("object_follower") {
  declare_parameter("image_width", IMG_WIDTH);
  declare_parameter("image_height", IMG_HEIGHT);

  get_parameter("image_width", IMG_WIDTH);
  get_parameter("image_height", IMG_HEIGHT);

  RCLCPP_INFO(this->get_logger(), "Image width: %d", IMG_WIDTH);
  RCLCPP_INFO(this->get_logger(), "Image height: %d", IMG_HEIGHT);

  detected_object_sub_ =
      create_subscription<vision_msgs::msg::Detection2DArray>(
          "detection", rclcpp::SensorDataQoS(),
          std::bind(&ObjectFollower::detected_object_cb, this,
                    std::placeholders::_1));

  command_pub_ =
      create_publisher<rrbot_cam_msgs::msg::PanTiltCommand>("command", 10);
}

void ObjectFollower::detected_object_cb(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr &msg) {
  if (msg->detections.empty()) return;

  rrbot_cam_msgs::msg::PanTiltCommand cmd_msg;
  for (auto detect: msg->detections) {

    // RCLCPP_INFO(this->get_logger(),detect.results[0].hypothesis.class_id.c_str());
    // if (detect.results[0].hypothesis.class_id == "x01") {
    //   cmd_msg.pan = (detect.bbox.center.position.x / IMG_WIDTH) * 2 - 1;
    //   cmd_msg.tilt = (detect.bbox.center.position.y / IMG_HEIGHT) * 2 - 1;
    //   command_pub_->publish(cmd_msg);
    //   return;
    // }

    // *2 to get range from 0 to 2, -1 to get range from -1 to 1
    cmd_msg.pan = (detect.bbox.center.position.x / IMG_WIDTH) * 2 - 1;
    cmd_msg.tilt = (detect.bbox.center.position.y / IMG_HEIGHT) * 2 - 1;
    RCLCPP_INFO(this->get_logger(), "Pan: %f, Tilt: %f", cmd_msg.pan, cmd_msg.tilt);
    command_pub_->publish(cmd_msg);
    return;
  }
}

}  // namespace rrbot_cam_object_follower