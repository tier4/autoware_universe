// Stub when ONNX Runtime is not available: subscribes and publishes empty DetectedObjectsWithFeature.
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

namespace autoware::traffic_light
{

class WholeImageDetectorNode : public rclcpp::Node
{
public:
  explicit WholeImageDetectorNode(const rclcpp::NodeOptions & options)
  : Node("traffic_light_whole_image_detector_node", options)
  {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "~/input/image", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        tier4_perception_msgs::msg::DetectedObjectsWithFeature out;
        out.header = msg->header;
        objects_pub_->publish(out);
      });
    objects_pub_ =
      create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>("~/output/rois", 1);
    RCLCPP_WARN(
      get_logger(), "Whole image detector built without TensorRT; publishing empty objects.");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    objects_pub_;
};

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::WholeImageDetectorNode)
