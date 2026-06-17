/**
 * @file generic_multiplexer.hpp
 * @brief Multiplexer node to choose an input topics to pass to an output topic.
 * @copyright 2025 TUM-FTM
 * @ingroup autoware_remote_interfaces
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "autoware_adapi_v1_msgs/srv/mux_select_input.hpp"

namespace autoware_remote_interfaces {
/**
 * @defgroup autoware_remote_interfaces
 * @ingroup autoware_remote_interfaces
 * @brief Interfaces to integrate TUM Teleoperation capabilities into Autoware.
 */

/**
 * @brief Multiplexer class to choose among input topics of same message type to pass to an output. 
 */
class GenericMultiplexer : public rclcpp::Node
{
    public:
        GenericMultiplexer();
        ~GenericMultiplexer() = default;
    
    private:
        // Subscriptions
        std::vector<rclcpp::GenericSubscription::SharedPtr> subs_;

        // Publisher     
        rclcpp::GenericPublisher::SharedPtr pub_output_;

        // Service 
        rclcpp::Service<autoware_adapi_v1_msgs::srv::MuxSelectInput>::SharedPtr srv_select_input_;

        // Callbacks
        void handle_input(std::shared_ptr<rclcpp::SerializedMessage> msg, std::string key);
        void handle_input_select(const std::shared_ptr<autoware_adapi_v1_msgs::srv::MuxSelectInput::Request> request, 
                                       std::shared_ptr<autoware_adapi_v1_msgs::srv::MuxSelectInput::Response> response);

        // Variables
        std::string selected_input_;
        std::vector<std::string> input_topic_keys_;
};

} // namespace autoware_remote_interfaces