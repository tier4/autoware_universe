#include "autoware/agnocast_wrapper/node.hpp"

namespace autoware::agnocast_wrapper
{

Node::Node(const std::string & node_name, const rclcpp::NodeOptions & options)
{
  initialize(node_name, "", options);
}

Node::Node(
  const std::string & node_name, const std::string & namespace_,
  const rclcpp::NodeOptions & options)
{
  initialize(node_name, namespace_, options);
}

void Node::initialize(
  const std::string & node_name, const std::string & namespace_,
  const rclcpp::NodeOptions & options)
{
  use_agnocast_ = use_agnocast();

  if (use_agnocast_) {
    if (namespace_.empty()) {
      agnocast_node_ = std::make_shared<agnocast::Node>(node_name, options);
    } else {
      agnocast_node_ = std::make_shared<agnocast::Node>(node_name, namespace_, options);
    }
  } else {
    if (namespace_.empty()) {
      rclcpp_node_ = std::make_shared<rclcpp::Node>(node_name, options);
    } else {
      rclcpp_node_ = std::make_shared<rclcpp::Node>(node_name, namespace_, options);
    }
  }
}

std::string Node::get_name() const
{
  if (use_agnocast_) {
    return agnocast_node_->get_name();
  }
  return rclcpp_node_->get_name();
}

std::string Node::get_namespace() const
{
  if (use_agnocast_) {
    return agnocast_node_->get_namespace();
  }
  return rclcpp_node_->get_namespace();
}

std::string Node::get_fully_qualified_name() const
{
  if (use_agnocast_) {
    return agnocast_node_->get_fully_qualified_name();
  }
  return rclcpp_node_->get_fully_qualified_name();
}

rclcpp::Logger Node::get_logger() const
{
  if (use_agnocast_) {
    return agnocast_node_->get_logger();
  }
  return rclcpp_node_->get_logger();
}

rclcpp::Clock::SharedPtr Node::get_clock()
{
  if (use_agnocast_) {
    return agnocast_node_->get_clock();
  }
  return rclcpp_node_->get_clock();
}

rclcpp::Time Node::now() const
{
  if (use_agnocast_) {
    return agnocast_node_->now();
  }
  return rclcpp_node_->now();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr Node::get_node_base_interface()
{
  if (use_agnocast_) {
    return agnocast_node_->get_node_base_interface();
  }
  return rclcpp_node_->get_node_base_interface();
}

rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr Node::get_node_topics_interface()
{
  if (use_agnocast_) {
    return agnocast_node_->get_node_topics_interface();
  }
  return rclcpp_node_->get_node_topics_interface();
}

rclcpp::node_interfaces::NodeParametersInterface::SharedPtr Node::get_node_parameters_interface()
{
  if (use_agnocast_) {
    return agnocast_node_->get_node_parameters_interface();
  }
  return rclcpp_node_->get_node_parameters_interface();
}

rclcpp::CallbackGroup::SharedPtr Node::create_callback_group(
  rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node)
{
  if (use_agnocast_) {
    return agnocast_node_->create_callback_group(group_type, automatically_add_to_executor_with_node);
  }
  return rclcpp_node_->create_callback_group(group_type, automatically_add_to_executor_with_node);
}

const rclcpp::ParameterValue & Node::declare_parameter(
  const std::string & name, const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor, bool ignore_override)
{
  if (use_agnocast_) {
    return agnocast_node_->declare_parameter(name, default_value, descriptor, ignore_override);
  }
  return rclcpp_node_->declare_parameter(name, default_value, descriptor, ignore_override);
}

const rclcpp::ParameterValue & Node::declare_parameter(
  const std::string & name, rclcpp::ParameterType type,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor, bool ignore_override)
{
  if (use_agnocast_) {
    return agnocast_node_->declare_parameter(name, type, descriptor, ignore_override);
  }
  return rclcpp_node_->declare_parameter(name, type, descriptor, ignore_override);
}

bool Node::has_parameter(const std::string & name) const
{
  if (use_agnocast_) {
    return agnocast_node_->has_parameter(name);
  }
  return rclcpp_node_->has_parameter(name);
}

void Node::undeclare_parameter(const std::string & name)
{
  if (use_agnocast_) {
    agnocast_node_->undeclare_parameter(name);
  } else {
    rclcpp_node_->undeclare_parameter(name);
  }
}

rclcpp::Parameter Node::get_parameter(const std::string & name) const
{
  if (use_agnocast_) {
    return agnocast_node_->get_parameter(name);
  }
  return rclcpp_node_->get_parameter(name);
}

bool Node::get_parameter(const std::string & name, rclcpp::Parameter & parameter) const
{
  if (use_agnocast_) {
    return agnocast_node_->get_parameter(name, parameter);
  }
  return rclcpp_node_->get_parameter(name, parameter);
}

std::vector<rclcpp::Parameter> Node::get_parameters(const std::vector<std::string> & names) const
{
  if (use_agnocast_) {
    return agnocast_node_->get_parameters(names);
  }
  return rclcpp_node_->get_parameters(names);
}

rcl_interfaces::msg::SetParametersResult Node::set_parameter(const rclcpp::Parameter & parameter)
{
  if (use_agnocast_) {
    return agnocast_node_->set_parameter(parameter);
  }
  return rclcpp_node_->set_parameter(parameter);
}

std::vector<rcl_interfaces::msg::SetParametersResult> Node::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  if (use_agnocast_) {
    return agnocast_node_->set_parameters(parameters);
  }
  return rclcpp_node_->set_parameters(parameters);
}

rcl_interfaces::msg::SetParametersResult Node::set_parameters_atomically(
  const std::vector<rclcpp::Parameter> & parameters)
{
  if (use_agnocast_) {
    return agnocast_node_->set_parameters_atomically(parameters);
  }
  return rclcpp_node_->set_parameters_atomically(parameters);
}

rcl_interfaces::msg::ParameterDescriptor Node::describe_parameter(const std::string & name) const
{
  if (use_agnocast_) {
    auto result = agnocast_node_->get_node_parameters_interface()->describe_parameters({name});
    if (result.empty()) {
      throw rclcpp::exceptions::ParameterNotDeclaredException(name);
    }
    return result.front();
  }
  return rclcpp_node_->describe_parameter(name);
}

std::vector<rcl_interfaces::msg::ParameterDescriptor> Node::describe_parameters(
  const std::vector<std::string> & names) const
{
  if (use_agnocast_) {
    return agnocast_node_->get_node_parameters_interface()->describe_parameters(names);
  }
  return rclcpp_node_->describe_parameters(names);
}

std::vector<uint8_t> Node::get_parameter_types(const std::vector<std::string> & names) const
{
  if (use_agnocast_) {
    return agnocast_node_->get_node_parameters_interface()->get_parameter_types(names);
  }
  return rclcpp_node_->get_parameter_types(names);
}

rcl_interfaces::msg::ListParametersResult Node::list_parameters(
  const std::vector<std::string> & prefixes, uint64_t depth) const
{
  if (use_agnocast_) {
    return agnocast_node_->get_node_parameters_interface()->list_parameters(prefixes, depth);
  }
  return rclcpp_node_->list_parameters(prefixes, depth);
}

rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
Node::add_on_set_parameters_callback(
  rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType callback)
{
  if (use_agnocast_) {
    return agnocast_node_->add_on_set_parameters_callback(callback);
  }
  return rclcpp_node_->add_on_set_parameters_callback(callback);
}

void Node::remove_on_set_parameters_callback(
  const rclcpp::node_interfaces::OnSetParametersCallbackHandle * const handler)
{
  if (use_agnocast_) {
    agnocast_node_->remove_on_set_parameters_callback(handler);
  } else {
    rclcpp_node_->remove_on_set_parameters_callback(handler);
  }
}

}  // namespace autoware::agnocast_wrapper
