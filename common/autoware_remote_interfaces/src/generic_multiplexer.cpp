/**
 * @file generic_multiplexer.cpp
 * @brief Multiplexer node to choose an input topics to pass to an output topic.
 * @copyright 2025 TUM-FTM
 * @ingroup autoware_remote_interfaces
 */

#include "autoware_remote_interfaces/generic_multiplexer.hpp"

namespace autoware_remote_interfaces {

GenericMultiplexer::GenericMultiplexer() : Node("generic_multiplexer"),
    selected_input_("AUTO")
{
    // QoS - Reliable, volatile, keep last for all considered topics in autoware.
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.durability_volatile();

    // Parameters
    declare_parameter("input_topic_names", std::vector<std::string>());
    declare_parameter("input_topic_keys", std::vector<std::string>());
    declare_parameter("output_topic", "/mux/output");
    declare_parameter("topic_type", "std_msgs/msg/Bool");
    declare_parameter("select_input_service", "select_input");

    std::vector<std::string> input_topic_names;
    std::string output_topic;
    std::string topic_type;
    std::string select_input_service;

    input_topic_names = get_parameter("input_topic_names").as_string_array();
    input_topic_keys_ = get_parameter("input_topic_keys").as_string_array();
    output_topic = get_parameter("output_topic").as_string();
    topic_type = get_parameter("topic_type").as_string();
    select_input_service = get_parameter("select_input_service").as_string();

    // Subscribers
    for (size_t idx = 0; idx < input_topic_names.size(); ++idx) {
      const auto & input_topic_name = input_topic_names.at(idx);
      const auto & key = input_topic_keys_.at(idx);
      subs_.push_back(create_generic_subscription(
        input_topic_name,
        topic_type,
        qos,
        [this, key](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            handle_input(msg, key);
        })
      );      
      RCLCPP_INFO_STREAM(get_logger(), "Added subscriber for: " << input_topic_name);
    }
    
    // Publisher
    pub_output_ = create_generic_publisher(output_topic,
                                           topic_type,
                                           qos);

    // Service
    srv_select_input_ = create_service<autoware_adapi_v1_msgs::srv::MuxSelectInput>(
        select_input_service,
                [this](const std::shared_ptr<autoware_adapi_v1_msgs::srv::MuxSelectInput::Request> request, 
                       std::shared_ptr<autoware_adapi_v1_msgs::srv::MuxSelectInput::Response> response) {
                    handle_input_select(request, response);
                }
    );
};

// Callbacks
void GenericMultiplexer::handle_input(std::shared_ptr<rclcpp::SerializedMessage> msg, std::string key) 
{
    if (key == selected_input_) {
        pub_output_->publish(*msg);
    }
};

void GenericMultiplexer::handle_input_select(const std::shared_ptr<autoware_adapi_v1_msgs::srv::MuxSelectInput::Request> request, 
                                             std::shared_ptr<autoware_adapi_v1_msgs::srv::MuxSelectInput::Response> response)
{
    if (std::count(input_topic_keys_.begin(), input_topic_keys_.end(), request->select_input)) {
        selected_input_ = request->select_input;
        response->set__success(true);
        RCLCPP_INFO_STREAM(this->get_logger(), "Selected Input: " << selected_input_);
    } else {
        response->set__success(false);
        std::ostringstream oss;
        for (size_t i = 0; i < input_topic_keys_.size(); ++i) { 
          if (i) {oss << ", ";} oss << input_topic_keys_[i]; 
        }
        RCLCPP_ERROR_STREAM(get_logger(), "Service call not successful! Your service called for key: " << request->select_input << ". Valid options are: " << oss.str());
    }
};


} // namespace autoware_remote_interfaces