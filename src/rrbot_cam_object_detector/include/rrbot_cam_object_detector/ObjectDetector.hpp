/*
Input: Image from camera (sensor_msgs/Image)
    Raw image

Output: Detected object (vision_msgs/Detection2D)
    Bounding box with:
        center (vision_msgs/Point2D) [x, y]
        size [size_x, size_y]
*/

#ifndef RRBOT_CAM_OBJECT_DETECTOR_HPP_
#define RRBOT_CAM_OBJECT_DETECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "image_transport/image_transport.hpp"

namespace rrbot_cam_object_detector {

class ObjectDetector : public rclcpp::Node {
 public:
  ObjectDetector();

 private:
  image_transport::Publisher debug_img_pub_it_;
//   image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr
      detected_object_pub_;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

  // HSV color range for object detection
  std::vector<double> hsv_ranges_{0, 180, 0, 255, 0, 255};
  bool debug_{false};
};

}  // namespace rrbot_cam_object_detector

#endif  // RRBOT_CAM_OBJECT_DETECTOR_HPP_