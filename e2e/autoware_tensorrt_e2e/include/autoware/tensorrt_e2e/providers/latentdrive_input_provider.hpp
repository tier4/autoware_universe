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

#ifndef AUTOWARE__TENSORRT_E2E__PROVIDERS__LATENTDRIVE_INPUT_PROVIDER_HPP_
#define AUTOWARE__TENSORRT_E2E__PROVIDERS__LATENTDRIVE_INPUT_PROVIDER_HPP_

#include "autoware/tensorrt_e2e/input_provider.hpp"

#include <Eigen/Core>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @brief Pure functions of the LatentDrive input contract, kept free of ROS so they can be
 * tested against the training-side reference.
 */
namespace latentdrive
{

/**
 * @brief Preprocess one camera frame the way LatentDrive was trained.
 *
 * BGR -> RGB, centre-crop the height to width / 2 (the full width is kept), bilinear resize to
 * `width` x `height`, scale to [0, 1], subtract `mean` and multiply by `inverse_std` per channel,
 * and pack HWC -> CHW. `out_chw` must hold 3 * height * width floats.
 * @return false when the frame is empty or not 3-channel.
 */
bool preprocess_frame(
  const cv::Mat & bgr, int64_t width, int64_t height, const std::array<float, 3> & mean,
  const std::array<float, 3> & inverse_std, float * out_chw);

/**
 * @brief Pick one stored frame per video slot.
 *
 * Slots are `interval` seconds apart and end at the newest stamp, so slot k targets
 * `newest - (num_frames - 1 - k) * interval`. Each slot takes the stamp nearest its target.
 * @param stamps Stamps in seconds, ascending.
 * @return Indices into `stamps`, oldest slot first; std::nullopt when any slot has no stamp
 *         within `tolerance` seconds of its target.
 */
std::optional<std::vector<size_t>> select_frame_slots(
  const std::vector<double> & stamps, int64_t num_frames, double interval, double tolerance);

/**
 * @brief The point `ahead_m` of arc length beyond the reference point nearest to `position`.
 *
 * Follows the LatentDrive replay node: nearest point by Euclidean distance, then walk the
 * polyline forward. When the reference ends first, its last point is returned.
 * @param reference At least one point, in the frame the result is wanted in.
 */
Eigen::Vector2d subgoal_along(
  const std::vector<Eigen::Vector2d> & reference, const Eigen::Vector2d & position, double ahead_m);

}  // namespace latentdrive

/**
 * @class LatentDriveInputProvider
 * @brief Produces the two inputs of the LatentDrive planner (a V-JEPA2 encoder over a short
 * front-camera clip, feeding a planner transformer).
 *
 * Claimable tensors (names configurable):
 * - `video` `[1, 3, T, H, W]`: T front-camera frames `frame_interval_seconds` apart, oldest
 *   first, each preprocessed once on arrival (see latentdrive::preprocess_frame) and stacked
 *   channel-major. T, H and W come from the engine. Frames are chosen by stamp, so a dropped
 *   camera frame shifts the pick to its neighbour rather than compressing the clip.
 * - `status` `[1, 6]`: `(subgoal_x / d, subgoal_y / d, v_x, v_y, a_x, a_y)` in the ego frame,
 *   with `d = status_subgoal_divisor`. The subgoal is the point `subgoal_ahead_m` of arc length
 *   ahead on the reference trajectory; velocity comes from the odometry, acceleration from the
 *   acceleration topic.
 *
 * Subscribes to `~/input/camera0/image` (via image_transport) and `~/input/reference_trajectory`
 * (`autoware_planning_msgs/Trajectory`). The reference is what the model was trained to follow,
 * a route sampled 50 m ahead; in an open-loop replay the recorded planner trajectory plays that
 * role. The subgoal actually fed to the model is published on `~/debug/latentdrive/subgoal`, the
 * ego pose the plan is anchored on (odometry at the tick) on `~/debug/latentdrive/ego_pose`,
 * and every frame the provider receives is echoed on `~/debug/latentdrive/frame_age_ms`
 * (stamped with the frame's sensor stamp, value = age at arrival), so frames lost between
 * the camera and this node show up as gaps in that topic.
 */
class LatentDriveInputProvider : public InputProviderInterface
{
public:
  explicit LatentDriveInputProvider(rclcpp::Node & node);

  std::string name() const override { return "latentdrive"; }
  std::vector<std::string> claim_inputs(const std::vector<TensorSpec> & engine_inputs) override;
  bool collect(
    const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs,
    std::string & error) override;

private:
  using Trajectory = autoware_planning_msgs::msg::Trajectory;

  struct Frame
  {
    rclcpp::Time stamp;
    std::shared_ptr<const std::vector<float>> chw;  //!< 3 * H * W, preprocessed.
  };

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  bool build_video_tensor(const rclcpp::Time & now, TensorMap & inputs, std::string & error);
  bool build_status_tensor(const EgoFrame & ego, TensorMap & inputs, std::string & error);

  rclcpp::Node & node_;

  // Deployment parameters
  std::string video_tensor_name_;
  std::string status_tensor_name_;
  std::string transport_;
  double frame_interval_s_{0.5};
  double frame_tolerance_s_{0.15};
  double max_delay_ms_{200.0};
  std::array<float, 3> mean_{};
  std::array<float, 3> inverse_std_{};
  double subgoal_ahead_m_{50.0};
  double subgoal_divisor_{10.0};
  //! One INFO line per tick, "[LatentDrive-debug] ...": whether the tick ran, the age of each
  //! selected frame relative to the tick, and the status fed to the model; or why it was skipped.
  bool tick_log_{false};

  // Engine-derived configuration
  std::vector<int64_t> video_shape_;
  std::vector<int64_t> status_shape_;
  int64_t num_frames_{0};
  int64_t height_{0};
  int64_t width_{0};

  // ROS interfaces
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_subgoal_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_ego_pose_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr pub_frame_age_;

  // State
  std::deque<Frame> frames_;  //!< Ascending by stamp; pruned to the clip span on push.
  Trajectory::ConstSharedPtr latest_trajectory_;
  mutable std::mutex mutex_;
  std::vector<float> video_buffer_;
  bool warned_no_acceleration_{false};
  std::vector<double> last_frame_ages_ms_;  //!< Ages of the selected frames at the last tick.
  std::array<float, 6> last_status_{};      //!< Status vector of the last tick, encoded.
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__PROVIDERS__LATENTDRIVE_INPUT_PROVIDER_HPP_
