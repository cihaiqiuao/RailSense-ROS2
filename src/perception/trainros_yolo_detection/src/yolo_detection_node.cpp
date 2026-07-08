#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "trainros_interfaces/msg/coupler.hpp"

class YoloDetectionNode : public rclcpp::Node
{
public:
  YoloDetectionNode() : Node("trainros_yolo_detection")
  {
    model_path_ = declare_parameter<std::string>("model_path", "");

    publisher_ = create_publisher<trainros_interfaces::msg::Coupler>(
      "/coupler_detection", rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
      std::bind(&YoloDetectionNode::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "YOLO detection 框架已启动，等待接入模型：%s", model_path_.c_str());
  }

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr image)
  {
    trainros_interfaces::msg::Coupler msg;
    msg.header = image->header;
    msg.bbox = {0.0F, 0.0F, 0.0F, 0.0F};
    msg.score = 0.0F;
    msg.status = "unknown";
    publisher_->publish(msg);
  }

  std::string model_path_;
  rclcpp::Publisher<trainros_interfaces::msg::Coupler>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
