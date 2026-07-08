#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class CameraDriverNode : public rclcpp::Node
{
public:
  CameraDriverNode() : Node("trainros_camera_driver")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");
    pipeline_ = declare_parameter<std::string>("pipeline", "");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    publisher_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", qos);
    timer_ = create_wall_timer(33ms, std::bind(&CameraDriverNode::publish_placeholder, this));

    RCLCPP_INFO(get_logger(), "Camera driver 框架已启动，等待接入 GStreamer pipeline");
  }

private:
  void publish_placeholder()
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.height = 0;
    msg.width = 0;
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = 0;
    publisher_->publish(msg);
  }

  std::string frame_id_;
  std::string pipeline_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDriverNode>());
  rclcpp::shutdown();
  return 0;
}
