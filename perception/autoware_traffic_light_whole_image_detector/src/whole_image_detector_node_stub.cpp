// Stub when ONNX Runtime is not available: subscribes and publishes empty rois.
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

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
        tier4_perception_msgs::msg::TrafficLightRoiArray out;
        out.header = msg->header;
        roi_pub_->publish(out);
      });
    roi_pub_ = create_publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>("~/output/rois", 1);
    RCLCPP_WARN(get_logger(), "Whole image detector built without TensorRT; publishing empty ROIs.");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr roi_pub_;
};

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::WholeImageDetectorNode)
