// Copyright 2024 The Autoware Contributors
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

#include "traffic_display.hpp"

#include <QFontDatabase>
#include <QPainter>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_rendering/render_system.hpp>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <string>

namespace autoware_overlay_rviz_plugin
{

TrafficDisplay::TrafficDisplay()
: tl_red_(QString("#cc3d3d")),
  tl_yellow_(QString("#ccb43d")),
  tl_green_(QString("#3dcc55")),
  tl_gray_(QString("#4f4f4f"))
{
  // Load the traffic light image
  std::string package_path =
    ament_index_cpp::get_package_share_directory("autoware_overlay_rviz_plugin");

  std::string image_path = package_path + "/assets/images/traffic.png";
  traffic_light_image_.load(image_path.c_str());

  std::string arrow_image_path = package_path + "/assets/images/arrow_signal.png";
  traffic_light_arrow_image_.load(arrow_image_path.c_str());
}

void TrafficDisplay::updateTrafficLightData(
  const autoware_perception_msgs::msg::TrafficLightGroup::ConstSharedPtr & msg)
{
  current_traffic_ = *msg;
}

QColor TrafficDisplay::getColorById(uint8_t color_id) const
{
  using autoware_perception_msgs::msg::TrafficLightElement;
  switch (color_id) {
    case TrafficLightElement::RED:
      return tl_red_;
    case TrafficLightElement::AMBER:
      return tl_yellow_;
    case TrafficLightElement::GREEN:
      return tl_green_;
    case TrafficLightElement::WHITE:
      return Qt::white;
    default:
      return tl_gray_;
  }
}

void TrafficDisplay::drawTrafficLightIndicator(QPainter & painter, const QRectF & backgroundRect)
{
  using autoware_perception_msgs::msg::TrafficLightElement;

  // Enable Antialiasing for smoother drawing
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

  QRectF circleRect = QRectF(50, 50, 50, 50);
  circleRect.moveTopRight(QPointF(
    backgroundRect.right() - circleRect.width() - 75,
    backgroundRect.height() / 2 - circleRect.height() / 2));

  painter.setPen(Qt::NoPen);

  QColor main_color = tl_gray_;
  QColor arrow_color = tl_gray_;
  bool has_circle = false;
  bool has_arrow = false;
  qreal arrow_angle = 0.0;

  if (!current_traffic_.elements.empty()) {
    for (const auto & element : current_traffic_.elements) {
      QColor color = getColorById(element.color);

      if (element.shape == TrafficLightElement::CIRCLE) {
        main_color = color;
        has_circle = true;
      } else if (element.shape != TrafficLightElement::UNKNOWN) {
        arrow_color = color;
        has_arrow = true;

        switch (element.shape) {
          case TrafficLightElement::LEFT_ARROW:
            arrow_angle = 180.0;
            break;
          case TrafficLightElement::RIGHT_ARROW:
            arrow_angle = 0.0;
            break;
          case TrafficLightElement::UP_ARROW:
            arrow_angle = -90.0;
            break;
          case TrafficLightElement::DOWN_ARROW:
            arrow_angle = 90.0;
            break;
          case TrafficLightElement::DOWN_LEFT_ARROW:
            arrow_angle = 135.0;
            break;
          case TrafficLightElement::DOWN_RIGHT_ARROW:
            arrow_angle = 45.0;
            break;
          default:
            break;
        }
      }
    }

    if (!has_circle && has_arrow) {
      main_color = arrow_color;
    }
  }

  painter.setBrush(QBrush(main_color));
  painter.drawEllipse(circleRect);

  if (has_circle && has_arrow) {
    double scale = 0.5;
    double w = circleRect.width() * scale;
    double h = circleRect.height() * scale;

    QRectF innerRect(0, 0, w, h);
    innerRect.moveCenter(circleRect.center());

    painter.setBrush(QBrush(arrow_color));
    painter.drawEllipse(innerRect);
  }

  const QImage & imageToDraw = (has_arrow) ? traffic_light_arrow_image_ : traffic_light_image_;

  float scaleFactor = 0.75;
  QSize scaledSize = imageToDraw.size() * scaleFactor;

  QImage scaledTrafficImage =
    imageToDraw.scaled(scaledSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);

  painter.save();

  painter.translate(circleRect.center());

  if (has_arrow) {
    painter.rotate(arrow_angle);
  }

  QRectF targetRect(
    -scaledSize.width() / 2.0, -scaledSize.height() / 2.0, scaledSize.width(), scaledSize.height());

  painter.drawImage(targetRect, scaledTrafficImage);

  painter.restore();
}

QImage TrafficDisplay::coloredImage(const QImage & source, const QColor & color)
{
  QImage result = source;
  QPainter p(&result);
  p.setCompositionMode(QPainter::CompositionMode_SourceAtop);
  p.fillRect(result.rect(), color);
  p.end();
  return result;
}

}  // namespace autoware_overlay_rviz_plugin
