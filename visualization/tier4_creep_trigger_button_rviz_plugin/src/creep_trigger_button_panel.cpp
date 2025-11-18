#include "creep_trigger_button_panel.hpp"

#include <QVBoxLayout>

namespace tier4_creep_trigger_button_rviz_plugin
{

CreepTriggerButtonPanel::CreepTriggerButtonPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // create node
  node_ = rclcpp::Node::make_shared("creep_trigger_button_panel");

  // create publisher
  publisher_ = node_->create_publisher<tier4_mutual_yielding_msgs::msg::CreepTrigger>(
    "/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/input/"
    "mot_slowdown_go",
    rclcpp::QoS(10));

  // create button
  button_ = new QPushButton("Creep Trigger", this);
  connect(button_, &QPushButton::clicked, this, &CreepTriggerButtonPanel::onButtonClicked);

  // create layout
  auto * layout = new QVBoxLayout(this);
  layout->addWidget(button_);
  setLayout(layout);
}

void CreepTriggerButtonPanel::onButtonClicked()
{
  // publish message
  tier4_mutual_yielding_msgs::msg::CreepTrigger msg;
  msg.stamp = node_->now();
  publisher_->publish(msg);
}

}  // namespace tier4_creep_trigger_button_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_creep_trigger_button_rviz_plugin::CreepTriggerButtonPanel, rviz_common::Panel)
