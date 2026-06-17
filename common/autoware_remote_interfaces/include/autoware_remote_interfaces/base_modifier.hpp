/**
 * @file base_gate.hpp
 * @brief Base class for gates to modify data flows within the vehicle software or an automation stack.
 * @copyright 2025 TUM-FTM
 * @ingroup autoware_remote_interfaces
 */

 #pragma once

 #include <unordered_map>
 #include <memory>
 
 #include "boost/sml.hpp"
 
 #include "rclcpp/rclcpp.hpp"
 
 namespace sml = boost::sml;
 
 namespace autoware_remote_interfaces {
 
 template  <typename ModifierStateMachineImpl>
 class BaseModifier : public rclcpp::Node 
 {
    public:
       // Constructor
       explicit BaseModifier (const std::string &node_name, void *context);
       // Destructor
       virtual ~BaseModifier (){};
    
    protected:
       // Input topic subscriber
       template <typename MessageType>
       void add_input_subscriber(const std::string &topic_name, std::function<void(const  std::shared_ptr<MessageType>)> callback) 
       {
           auto sub = create_subscription<MessageType>(
               topic_name, 
               1,
               [this, callback](const std::shared_ptr<MessageType> msg) {
                   callback(msg);
               });
           input_subscribers_[topic_name] = sub;
       }
       // Modifier control service
       template <typename ServiceType>
       void set_control_service(const std::string &service_name, std::function<void(const std::shared_ptr< typename ServiceType::Request>, std::shared_ptr< typename ServiceType::Response>)> callback) 
       {
            control_service_ = create_service<ServiceType>(service_name, callback, rmw_qos_profile_services_default);
       }
       // Output publisher
       template <typename MessageType>
       void set_output_publisher(const std::string &topic_name) 
       {
           output_publisher_ = create_publisher<MessageType>(topic_name, 1);
       }
       template <typename MessageType>
       void publish_to_output(const std::shared_ptr<MessageType> msg) {
           auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<MessageType>>(output_publisher_);
           if (pub) {
               pub->publish(*msg);
           } else {
               RCLCPP_ERROR(get_logger(), "Incorrect message type or output publisher not set.");
           }
       }
       // State machine 
       struct ModifierStateMachine : public sml::sm<ModifierStateMachineImpl> 
       {
           explicit ModifierStateMachine(void *context) 
               : sml::sm<ModifierStateMachineImpl>(static_cast<typename ModifierStateMachineImpl::context_type *>(context)) {}
       };
       std::shared_ptr<ModifierStateMachine> state_machine_;

    private:
       std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> input_subscribers_;
       
       rclcpp::PublisherBase::SharedPtr output_publisher_;
       rclcpp::ServiceBase::SharedPtr control_service_;
 };
 
// Constructor template
template <typename ModifierStateMachineImpl>
BaseModifier<ModifierStateMachineImpl>::BaseModifier(const std::string &node_name, void *context) 
    : Node(node_name), 
      state_machine_(std::make_shared<ModifierStateMachine>(context)) { };

} // namespace autoware_remote_interfaces