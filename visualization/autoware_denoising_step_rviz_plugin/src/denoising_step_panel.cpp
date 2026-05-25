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

#include "denoising_step_panel.hpp"

#include <QBrush>
#include <QGridLayout>
#include <QMargins>
#include <QMetaObject>
#include <QPainter>
#include <QPen>
#include <QScrollArea>
#include <QString>
#include <QVBoxLayout>
#include <QtCharts/QChart>
#include <QtCharts/QLegend>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace autoware::denoising_step_rviz_plugin
{

namespace
{
constexpr auto k_default_topic =
  "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/debug/"
  "denoising_steps";
constexpr size_t k_first_visualized_point = 1;

size_t get_dim_size(const std_msgs::msg::MultiArrayLayout & layout, const char * label)
{
  const auto it = std::find_if(
    layout.dim.begin(), layout.dim.end(),
    [label](const std_msgs::msg::MultiArrayDimension & dim) { return dim.label == label; });
  return it == layout.dim.end() ? 0 : static_cast<size_t>(it->size);
}
}  // namespace

DenoisingStepChartWidget::DenoisingStepChartWidget(QWidget * parent) : QWidget(parent)
{
  grid_layout_ = new QGridLayout(this);
  grid_layout_->setContentsMargins(4, 4, 4, 4);
  grid_layout_->setSpacing(6);
  setLayout(grid_layout_);
}

void DenoisingStepChartWidget::setData(const DenoisingStepData & data)
{
  if (
    data.steps == 0 || data.batches == 0 || data.points <= k_first_visualized_point ||
    data.dims < 2) {
    clearCharts();
    return;
  }

  const int n = static_cast<int>(data.steps);
  const int rows = std::max(1, static_cast<int>(std::floor(std::sqrt(n))));
  const int cols = std::max(1, static_cast<int>(std::ceil(static_cast<double>(n) / rows)));
  const int margin = 8;

  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  for (size_t step = 0; step < data.steps; ++step) {
    for (size_t batch = 0; batch < data.batches; ++batch) {
      for (size_t point = k_first_visualized_point; point < data.points; ++point) {
        const auto index = (((step * data.batches + batch) * data.points + point) * data.dims);
        const auto x = data.data.at(index);
        const auto y = data.data.at(index + 1);
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
      }
    }
  }

  const auto x_range = std::max(1.0f, max_x - min_x);
  const auto y_range = std::max(1.0f, max_y - min_y);
  const auto center_x = 0.5f * (min_x + max_x);
  const auto center_y = 0.5f * (min_y + max_y);
  const auto half_range = 0.55f * std::max(x_range, y_range);
  const auto x_min = center_x - half_range;
  const auto x_max = center_x + half_range;
  const auto y_min = center_y - half_range;
  const auto y_max = center_y + half_range;

  ensureCharts(data.steps, data.batches, cols);

  for (size_t step = 0; step < data.steps; ++step) {
    auto & chart = charts_.at(step);
    chart.axis_x->setRange(x_min, x_max);
    chart.axis_y->setRange(y_min, y_max);
    for (size_t batch = 0; batch < data.batches; ++batch) {
      QVector<QPointF> points;
      points.reserve(static_cast<int>(data.points - k_first_visualized_point));
      for (size_t point = k_first_visualized_point; point < data.points; ++point) {
        const auto index = (((step * data.batches + batch) * data.points + point) * data.dims);
        points.append(QPointF(data.data.at(index), data.data.at(index + 1)));
      }
      chart.series.at(batch)->replace(points);
    }
  }

  setMinimumSize(cols * (240 + margin), rows * (220 + margin));
}

void DenoisingStepChartWidget::clearCharts()
{
  for (auto & chart : charts_) {
    grid_layout_->removeWidget(chart.view);
    chart.view->deleteLater();
  }
  charts_.clear();
  chart_batches_ = 0;
}

void DenoisingStepChartWidget::ensureCharts(
  const size_t steps, const size_t batches, const int cols)
{
  if (charts_.size() == steps && chart_batches_ == batches) {
    return;
  }

  clearCharts();
  charts_.reserve(steps);
  chart_batches_ = batches;

  for (size_t step = 0; step < steps; ++step) {
    const int row = static_cast<int>(step) / cols;
    const int col = static_cast<int>(step) % cols;
    auto * chart = new QtCharts::QChart();
    chart->setTitle(QString("step %1").arg(step));
    chart->legend()->hide();
    chart->setMargins(QMargins(4, 4, 4, 4));
    chart->setBackgroundBrush(QBrush(QColor(250, 250, 250)));

    auto * axis_x = new QtCharts::QValueAxis();
    axis_x->setTickCount(5);
    axis_x->setLabelFormat("%.1f");

    auto * axis_y = new QtCharts::QValueAxis();
    axis_y->setTickCount(5);
    axis_y->setLabelFormat("%.1f");

    StepChart step_chart;
    step_chart.axis_x = axis_x;
    step_chart.axis_y = axis_y;
    step_chart.series.reserve(batches);

    for (size_t batch = 0; batch < batches; ++batch) {
      auto * series = new QtCharts::QLineSeries();
      series->setName(QString("batch %1").arg(batch));
      QPen pen(batch == 0 ? QColor("#1f77b4") : QColor("#ff7f0e"));
      pen.setWidthF(1.8);
      series->setPen(pen);
      chart->addSeries(series);
      step_chart.series.push_back(series);
    }

    chart->addAxis(axis_x, Qt::AlignBottom);
    chart->addAxis(axis_y, Qt::AlignLeft);
    for (auto * series : step_chart.series) {
      series->attachAxis(axis_x);
      series->attachAxis(axis_y);
    }

    auto * chart_view = new QtCharts::QChartView(chart, this);
    chart_view->setRenderHint(QPainter::Antialiasing);
    chart_view->setMinimumSize(240, 220);
    grid_layout_->addWidget(chart_view, row, col);
    step_chart.view = chart_view;
    charts_.push_back(step_chart);
  }
}

DenoisingStepPanel::DenoisingStepPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  chart_widget_ = new DenoisingStepChartWidget(this);
  scroll_area_ = new QScrollArea(this);
  scroll_area_->setWidgetResizable(true);
  scroll_area_->setWidget(chart_widget_);
  layout->addWidget(scroll_area_);

  status_label_ = new QLabel("Waiting for denoising steps");
  layout->addWidget(status_label_);
  setLayout(layout);
}

void DenoisingStepPanel::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
  if (rviz_ros_node_.expired()) {
    status_label_->setText("RViz ROS node is not ready");
    return;
  }

  auto node = rviz_ros_node_.lock()->get_raw_node();
  sub_denoising_steps_ = node->create_subscription<Float32MultiArray>(
    k_default_topic, rclcpp::QoS{1},
    [this](const Float32MultiArray::ConstSharedPtr msg) { onDenoisingSteps(msg); });
}

void DenoisingStepPanel::onDenoisingSteps(const Float32MultiArray::ConstSharedPtr msg)
{
  DenoisingStepData data;
  data.steps = get_dim_size(msg->layout, "step");
  data.batches = get_dim_size(msg->layout, "batch");
  data.points = get_dim_size(msg->layout, "time");
  data.dims = get_dim_size(msg->layout, "dim");
  data.data = msg->data;

  const auto expected_size = data.steps * data.batches * data.points * data.dims;
  if (
    data.steps == 0 || data.batches == 0 || data.points == 0 || data.dims < 2 ||
    data.data.size() != expected_size) {
    QMetaObject::invokeMethod(
      this, [this]() { status_label_->setText("Invalid denoising step message"); },
      Qt::QueuedConnection);
    return;
  }

  const auto steps = data.steps;
  const auto batches = data.batches;
  const auto points = data.points;
  QMetaObject::invokeMethod(
    this,
    [this, steps, batches, points]() {
      status_label_->setText(
        QString("steps: %1, batches: %2, points: %3").arg(steps).arg(batches).arg(points));
    },
    Qt::QueuedConnection);
  QMetaObject::invokeMethod(
    chart_widget_, [this, data = std::move(data)]() { chart_widget_->setData(data); },
    Qt::QueuedConnection);
}

void DenoisingStepPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void DenoisingStepPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

}  // namespace autoware::denoising_step_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware::denoising_step_rviz_plugin::DenoisingStepPanel, rviz_common::Panel)
