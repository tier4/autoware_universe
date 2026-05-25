// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DENOISING_STEP_PANEL_HPP_
#define DENOISING_STEP_PANEL_HPP_

#include <QLabel>
#include <QWidget>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

#include <cstddef>
#include <vector>

class QGridLayout;
class QScrollArea;

namespace autoware::denoising_step_rviz_plugin
{

struct DenoisingStepData
{
  std::vector<float> data;
  size_t steps{0};
  size_t batches{0};
  size_t points{0};
  size_t dims{0};
};

class DenoisingStepChartWidget : public QWidget
{
  Q_OBJECT

public:
  explicit DenoisingStepChartWidget(QWidget * parent = nullptr);

  void setData(const DenoisingStepData & data);

private:
  struct StepChart
  {
    QtCharts::QChartView * view{nullptr};
    QtCharts::QValueAxis * axis_x{nullptr};
    QtCharts::QValueAxis * axis_y{nullptr};
    std::vector<QtCharts::QLineSeries *> series;
  };

  void clearCharts();
  void ensureCharts(size_t steps, size_t batches, int cols);

  QGridLayout * grid_layout_;
  std::vector<StepChart> charts_;
  size_t chart_batches_{0};
};

class DenoisingStepPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DenoisingStepPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private:
  using Float32MultiArray = std_msgs::msg::Float32MultiArray;

  void onDenoisingSteps(const Float32MultiArray::ConstSharedPtr msg);

  QLabel * status_label_;
  QScrollArea * scroll_area_;
  DenoisingStepChartWidget * chart_widget_;

  rclcpp::Subscription<Float32MultiArray>::SharedPtr sub_denoising_steps_;
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};

}  // namespace autoware::denoising_step_rviz_plugin

#endif  // DENOISING_STEP_PANEL_HPP_
