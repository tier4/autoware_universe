// Copyright 2025 TIER IV, Inc.
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

#include "rclcpp/rclcpp.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include "autoware_map_msgs/msg/lanelet_map_bin.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
// Lanelet2 includes
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_io/Io.h>

using std::size_t;

struct Pt2
{
  double x, y;
};

struct KDItem
{
  Pt2 p;
  lanelet::Id lanelet_id;
  size_t idx_in_centerline;
};

class KDTree2D
{
public:
  struct Node
  {
    KDItem item;
    std::unique_ptr<Node> left;
    std::unique_ptr<Node> right;
    int axis = 0;
  };

  KDTree2D() = default;

  void build(std::vector<KDItem> items) { root_ = buildRecursive(std::move(items), 0); }

  const KDItem * nearest(const Pt2 & query) const
  {
    if (!root_) return nullptr;
    const KDItem * best = nullptr;
    double best_dist2 = std::numeric_limits<double>::infinity();
    searchRecursive(root_.get(), query, best, best_dist2);
    return best;
  }

private:
  std::unique_ptr<Node> root_;

  static std::unique_ptr<Node> buildRecursive(std::vector<KDItem> items, int depth)
  {
    if (items.empty()) return nullptr;
    int axis = depth % 2;
    auto cmp = [axis](const KDItem & a, const KDItem & b) {
      return (axis == 0) ? (a.p.x < b.p.x) : (a.p.y < b.p.y);
    };
    size_t mid = items.size() / 2;
    std::nth_element(items.begin(), items.begin() + mid, items.end(), cmp);
    std::unique_ptr<Node> node = std::make_unique<Node>();
    node->axis = axis;
    node->item = items[mid];
    std::vector<KDItem> left_items(items.begin(), items.begin() + mid);
    std::vector<KDItem> right_items(items.begin() + mid + 1, items.end());
    node->left = buildRecursive(std::move(left_items), depth + 1);
    node->right = buildRecursive(std::move(right_items), depth + 1);
    return node;
  }

  static double dist2(const Pt2 & a, const Pt2 & b)
  {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return dx * dx + dy * dy;
  }

  void searchRecursive(
    const Node * node, const Pt2 & query, const KDItem *& best, double & best_dist2) const
  {
    if (!node) return;
    double d2 = dist2(node->item.p, query);
    if (d2 < best_dist2) {
      best_dist2 = d2;
      best = &node->item;
    }
    int axis = node->axis;
    double diff = (axis == 0) ? (query.x - node->item.p.x) : (query.y - node->item.p.y);
    const Node * near = (diff <= 0) ? node->left.get() : node->right.get();
    const Node * far = (diff <= 0) ? node->right.get() : node->left.get();
    if (near) searchRecursive(near, query, best, best_dist2);
    if (far && diff * diff < best_dist2) searchRecursive(far, query, best, best_dist2);
  }
};

// Utility functions
static inline double euclidDist(const Pt2 & a, const Pt2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

// Calculate perpendicular distance from point to line defined by two points
// Also returns the signed cross product to determine which side the point is on
static inline double calculatePerpendicularDistance(
  const Pt2 & p, const Pt2 & line_start, const Pt2 & line_end, double & out_cross_product)
{
  // Road direction vector
  double vx = line_end.x - line_start.x;
  double vy = line_end.y - line_start.y;
  double v_len = std::hypot(vx, vy);

  if (v_len <= 1e-9) {
    out_cross_product = 0.0;
    return std::hypot(p.x - line_start.x, p.y - line_start.y);
  }

  // Normalize direction vector
  vx /= v_len;
  vy /= v_len;

  // Vector from line_start to point
  double wx = p.x - line_start.x;
  double wy = p.y - line_start.y;

  // Calculate cross product (determines left/right side)
  // Positive = left side, Negative = right side (in standard coordinates)
  out_cross_product = vx * wy - vy * wx;

  // Calculate perpendicular distance using cross product magnitude
  double perp_dist = std::abs(out_cross_product);

  return perp_dist;
}

class DrivableAreaVisualFilter : public rclcpp::Node
{
public:
  DrivableAreaVisualFilter() : Node("drivable_area_visual_filter")
  {
    this->declare_parameter<std::string>("input_objects_topic", "/input/objects");
    this->declare_parameter<std::string>("output_objects_topic", "/output/objects");
    this->declare_parameter<std::string>("map_topic", "/input/map");

    this->get_parameter("input_objects_topic", input_objects_topic_);
    this->get_parameter("output_objects_topic", output_objects_topic_);
    this->get_parameter("map_topic", map_topic_);

    RCLCPP_INFO(
      this->get_logger(), "Publishing filtered poses to: '%s'", output_objects_topic_.c_str());

    pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(output_objects_topic_, 10);

    // Lanelet map subscription
    map_sub_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
      map_topic_, rclcpp::QoS{1}.transient_local(),
      std::bind(&DrivableAreaVisualFilter::mapCallback, this, std::placeholders::_1));

    // Predicted objects subscription
    obj_sub_ = this->create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
      input_objects_topic_, 10,
      std::bind(&DrivableAreaVisualFilter::objectsCallback, this, std::placeholders::_1));
  }

private:
  std::string input_objects_topic_;
  std::string output_objects_topic_;
  std::string map_topic_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr obj_sub_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::vector<KDItem> centerline_points_;
  KDTree2D kdtree_;
  std::unordered_map<lanelet::Id, lanelet::Lanelet> lanelet_by_id_;

  void mapCallback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr map_msg)
  {
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);

    centerline_points_.clear();
    lanelet_by_id_.clear();

    for (const auto & ll : lanelet_map_ptr_->laneletLayer) {
      lanelet_by_id_.emplace(ll.id(), ll);
      const auto & center = ll.centerline();
      for (size_t i = 0; i < center.size(); ++i) {
        KDItem it{{center[i].x(), center[i].y()}, ll.id(), i};
        centerline_points_.push_back(it);
      }
    }

    if (!centerline_points_.empty()) {
      kdtree_.build(centerline_points_);
      RCLCPP_INFO(
        this->get_logger(), "Received lanelet map. Centerline points: %zu",
        centerline_points_.size());
    } else {
      RCLCPP_WARN(this->get_logger(), "Received lanelet map but no centerline points found");
    }
  }

  void objectsCallback(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
  {
    if (centerline_points_.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000 /*ms*/,
        "Lanelet map not ready, skipping object filtering");
      return;  // map not ready
    }
    geometry_msgs::msg::PoseArray out;
    out.header = msg->header;
    out.poses.reserve(msg->objects.size());

    int filtered_count = 0;
    int total_count = 0;

    for (const auto & obj : msg->objects) {
      total_count++;
      const auto & pose = obj.kinematics.initial_pose_with_covariance.pose;
      Pt2 pcar{pose.position.x, pose.position.y};

      const KDItem * nearest = kdtree_.nearest(pcar);
      if (!nearest) {
        out.poses.push_back(pose);
        continue;
      }

      auto itll = lanelet_by_id_.find(nearest->lanelet_id);
      if (itll == lanelet_by_id_.end()) {
        out.poses.push_back(pose);
        continue;
      }

      const lanelet::Lanelet & ll = itll->second;
      const auto center = ll.centerline();
      size_t idx = nearest->idx_in_centerline;

      // Get nearest centerline point
      Pt2 c_nearest{nearest->p.x, nearest->p.y};

      // Get neighbor point to define road direction
      Pt2 c_neigh;
      if (idx + 1 < center.size()) {
        c_neigh = Pt2{center[idx + 1].x(), center[idx + 1].y()};
      } else if (idx > 0) {
        c_neigh = Pt2{center[idx - 1].x(), center[idx - 1].y()};
      } else {
        // Single point centerline - accept the vehicle
        out.poses.push_back(pose);
        continue;
      }

      // Calculate perpendicular distance and determine which side vehicle is on
      double cross_product = 0.0;
      double d_perp = calculatePerpendicularDistance(pcar, c_nearest, c_neigh, cross_product);

      // Determine which side of the centerline the vehicle is on
      bool is_on_left = cross_product > 0.0;

      // Calculate distance to left and right boundaries
      double d_left = estimateDistanceToLineStringBounds(c_nearest, ll.leftBound());
      double d_right = estimateDistanceToLineStringBounds(c_nearest, ll.rightBound());

      // Apply correct filter: vehicle must be within the boundary on its side
      bool is_valid = false;
      if (is_on_left) {
        is_valid = (d_perp <= d_left + 1e-6);
      } else {
        is_valid = (d_perp <= d_right + 1e-6);
      }

      if (is_valid) {
        out.poses.push_back(pose);
      } else {
        filtered_count++;
      }
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /*ms*/, "Filtered %d/%d objects (%.1f%% kept)",
      filtered_count, total_count,
      total_count > 0 ? 100.0 * (total_count - filtered_count) / total_count : 0.0);

    pub_->publish(out);
  }

  double estimateDistanceToLineStringBounds(
    const Pt2 & center_pt, const lanelet::ConstLineString3d & b)
  {
    if (b.empty()) return 0.0;
    double min_d = std::numeric_limits<double>::infinity();

    // Find minimum distance from center_pt to any point on the boundary linestring
    for (const auto & p : b) {
      double d = euclidDist(center_pt, Pt2{p.x(), p.y()});
      if (d < min_d) min_d = d;
    }

    return (min_d == std::numeric_limits<double>::infinity()) ? 0.0 : min_d;
  }

  lanelet::ConstLanelets road_lanelets_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DrivableAreaVisualFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
