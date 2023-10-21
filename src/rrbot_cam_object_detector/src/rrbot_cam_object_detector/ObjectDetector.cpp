#include "rrbot_cam_object_detector/ObjectDetector.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

namespace rrbot_cam_object_detector {

ObjectDetector::ObjectDetector() : Node("object_detector") {
  declare_parameter("hsv_ranges", hsv_ranges_);
  declare_parameter("debug", debug_);

  get_parameter("hsv_ranges", hsv_ranges_);
  get_parameter("debug", debug_);

  if (debug_) {
    RCLCPP_INFO(get_logger(), "Debug mode is on");
    RCLCPP_INFO(get_logger(), "HSV ranges to be detected: ");
    RCLCPP_INFO(get_logger(), "h: %f - %f", hsv_ranges_[0], hsv_ranges_[1]);
    RCLCPP_INFO(get_logger(), "s: %f - %f", hsv_ranges_[2], hsv_ranges_[3]);
    RCLCPP_INFO(get_logger(), "v: %f - %f", hsv_ranges_[4], hsv_ranges_[5]);
  }

  debug_img_pub_it_ = image_transport::create_publisher(this, "debug_image");

  // image_sub_ = image_transport::create_subscription(
  //   this, "input_image", std::bind(&ObjectDetector::image_callback, this, std::placeholders::_1),
  //   "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "input_image", 5, 
      std::bind(&ObjectDetector::image_callback, this, std::placeholders::_1));

  detected_object_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
      "detection", 10);
}

void ObjectDetector::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &img) {
  // Check if there's any subscribers on output_detection topic
  // if (detected_object_pub_->get_subscription_count() == 0) return;

  const double &h = hsv_ranges_[0];
  const double &H = hsv_ranges_[1];
  const double &s = hsv_ranges_[2];
  const double &S = hsv_ranges_[3];
  const double &v = hsv_ranges_[4];
  const double &V = hsv_ranges_[5];

  // Convert image msg to openCV image
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_INFO(get_logger(), "cv_bridge error: %s", e.what());
    return;
  }

  // Convert to HSV
  cv::Mat img_hsv;
  cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_BGR2HSV);

  // Check if it's within range
  cv::Mat1b filtered;
  cv::inRange(img_hsv, cv::Scalar(h, s, v), cv::Scalar(H, S, V), filtered);

  // Get center
  cv::Rect bbox = cv::boundingRect(filtered);
  auto m = cv::moments(filtered, true);

  if (m.m00 < 0.000001) return;

  int cx = m.m10 / m.m00;
  int cy = m.m01 / m.m00;
  
  if (debug_)
    RCLCPP_INFO(get_logger(), "Detected object at (%d, %d)", cx, cy);

  // Publish the message
  vision_msgs::msg::Detection2DArray detection_array_msg;
  vision_msgs::msg::Detection2D detection_msg;
  detection_msg.header = img->header;
  detection_msg.bbox.size_x = bbox.width;
  detection_msg.bbox.size_y = bbox.height;
  detection_msg.bbox.center.position.x = cx;
  detection_msg.bbox.center.position.y = cy;

  detection_array_msg.header = img->header;
  detection_array_msg.detections.push_back(detection_msg);

  detected_object_pub_->publish(detection_array_msg);

  if (debug_){
    cv::rectangle(cv_ptr->image, bbox, cv::Scalar(0, 0, 255), 3);
    cv::circle(cv_ptr->image, cv::Point(cx, cy), 3, cv::Scalar(255, 0, 0), 3);
    debug_img_pub_it_.publish(cv_ptr->toImageMsg());
  }
}

}  // namespace rrbot_cam_object_detector