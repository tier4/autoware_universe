#ifndef SLOWDOWN_GO_BUTTON_RVIZ_PLUGIN__SLOWDOWN_GO_BUTTON_PANEL_HPP_
#define SLOWDOWN_GO_BUTTON_RVIZ_PLUGIN__SLOWDOWN_GO_BUTTON_PANEL_HPP_

#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <tier4_mutual_yielding_msgs/msg/creep_trigger.hpp>

namespace tier4_creep_trigger_button_rviz_plugin
{

class CreepTriggerButtonPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CreepTriggerButtonPanel(QWidget * parent = nullptr);
  ~CreepTriggerButtonPanel() override = default;

protected Q_SLOTS:
  void onButtonClicked();

private:
  QPushButton * button_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<tier4_mutual_yielding_msgs::msg::CreepTrigger>::SharedPtr publisher_;
};

}  // namespace tier4_creep_trigger_button_rviz_plugin

#endif  // SLOWDOWN_GO_BUTTON_RVIZ_PLUGIN__SLOWDOWN_GO_BUTTON_PANEL_HPP_
