// Copyright 2020 Tier IV, Inc.
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
//
//

#include "multi_object_tracker/tracker/model/tracker_base.hpp"

#include "multi_object_tracker/utils/utils.hpp"

#include <algorithm>
#include <random>

namespace
{
float updateProbability(
  const float & prior, const float & true_positive, const float & false_positive)
{
  constexpr float max_updated_probability = 0.999;
  constexpr float min_updated_probability = 0.100;
  const float probability =
    (prior * true_positive) / (prior * true_positive + (1 - prior) * false_positive);
  return std::max(std::min(probability, max_updated_probability), min_updated_probability);
}
float decayProbability(const float & prior, const float & delta_time)
{
  constexpr float minimum_probability = 0.001;
  const float decay_rate = log(0.5f) / 0.3f;  // half-life (50% decay) of 0.3s
  return std::max(prior * std::exp(decay_rate * delta_time), minimum_probability);
}
}  // namespace

Tracker::Tracker(
  const rclcpp::Time & time,
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification,
  const size_t & channel_size)
: classification_(classification),
  no_measurement_count_(0),
  total_no_measurement_count_(0),
  total_measurement_count_(1),
  last_update_with_measurement_time_(time)
{
  // Generate random number
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid_.uuid.begin(), uuid_.uuid.end(), bit_eng);

  // Initialize existence probabilities
  existence_probabilities_.resize(channel_size, 0.001);
  total_existence_probability_ = 0.001;
}

void Tracker::initializeExistenceProbabilities(
  const uint & channel_index, const float & existence_probability)
{
  // The initial existence probability is modeled
  // since the incoming object's existence probability is not reliable
  // existence probability on each channel
  constexpr float initial_existence_probability = 0.1;
  existence_probabilities_[channel_index] = initial_existence_probability;

  // total existence probability
  constexpr float max_probability = 0.999;
  constexpr float min_probability = 0.100;
  total_existence_probability_ =
    std::max(std::min(existence_probability, max_probability), min_probability);
}

bool Tracker::updateWithMeasurement(
  const autoware_perception_msgs::msg::DetectedObject & object,
  const rclcpp::Time & measurement_time, const geometry_msgs::msg::Transform & self_transform,
  const uint & channel_index)
{
  // Update existence probability
  {
    no_measurement_count_ = 0;
    ++total_measurement_count_;

    // existence probability on each channel
    const float delta_time =
      std::abs((measurement_time - last_update_with_measurement_time_).seconds());
    constexpr float probability_true_detection = 0.9;
    constexpr float probability_false_detection = 0.2;

    // update measured channel probability without decay
    existence_probabilities_[channel_index] = updateProbability(
      existence_probabilities_[channel_index], probability_true_detection,
      probability_false_detection);

    // decay other channel probabilities
    for (size_t i = 0; i < existence_probabilities_.size(); ++i) {
      if (i != channel_index) {
        existence_probabilities_[i] = decayProbability(existence_probabilities_[i], delta_time);
      }
    }

    // update total existence probability
    const double existence_probability =
      channel_info.trust_existence_probability ? object.existence_probability : 0.9;
    total_existence_probability_ = updateProbability(
      total_existence_probability_, existence_probability * probability_true_detection,
      probability_false_detection);
  }

  last_update_with_measurement_time_ = measurement_time;

  // Update object
  measure(object, measurement_time, self_transform);

  // Update object status
  getTrackedObject(measurement_time, object_);

  return true;
}

bool Tracker::updateWithoutMeasurement(const rclcpp::Time & timestamp)
{
  // Update existence probability
  ++no_measurement_count_;
  ++total_no_measurement_count_;
  {
    // decay existence probability
    float const delta_time = (timestamp - last_update_with_measurement_time_).seconds();
    for (float & existence_probability : existence_probabilities_) {
      existence_probability = decayProbability(existence_probability, delta_time);
    }
    total_existence_probability_ = decayProbability(total_existence_probability_, delta_time);
  }

  // Update object status
  getTrackedObject(timestamp, object_);

  return true;
}

void Tracker::updateClassification(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification)
{
  // classification algorithm:
  // 0. Normalize the input classification
  // 1-1. Update the matched classification probability with a gain (ratio of 0.05)
  // 1-2. If the label is not found, add it to the classification list
  // 2. Remove the class with probability < remove_threshold (0.001)
  // 3. Normalize tracking classification

  // Parameters
  // if the remove_threshold is too high (compare to the gain), the classification will be removed
  // immediately
  const float gain = 0.05;
  constexpr float remove_threshold = 0.001;

  // Normalization function
  auto normalizeProbabilities =
    [](std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification) {
      float sum = 0.0;
      for (const auto & a_class : classification) {
        sum += a_class.probability;
      }
      for (auto & a_class : classification) {
        a_class.probability /= sum;
      }
    };

  // Normalize the input
  auto classification_input = classification;
  normalizeProbabilities(classification_input);

  // Update the matched classification probability with a gain
  for (const auto & new_class : classification_input) {
    bool found = false;
    for (auto & old_class : classification_) {
      if (new_class.label == old_class.label) {
        old_class.probability += new_class.probability * gain;
        found = true;
        break;
      }
    }
    // If the label is not found, add it to the classification list
    if (!found) {
      auto adding_class = new_class;
      adding_class.probability *= gain;
      classification_.push_back(adding_class);
    }
  }

  // If the probability is less than the threshold, remove the class
  classification_.erase(
    std::remove_if(
      classification_.begin(), classification_.end(),
      [remove_threshold](const auto & a_class) { return a_class.probability < remove_threshold; }),
    classification_.end());

  // Normalize tracking classification
  normalizeProbabilities(classification_);
}

geometry_msgs::msg::PoseWithCovariance Tracker::getPoseWithCovariance(
  const rclcpp::Time & time) const
{
  autoware_perception_msgs::msg::TrackedObject object;
  getTrackedObject(time, object);
  return object.kinematics.pose_with_covariance;
}

void Tracker::getPositionCovarianceEigenSq(
  const rclcpp::Time & time, double & major_axis_sq, double & minor_axis_sq) const
{
  // estimate the covariance of the position at the given time
  types::DynamicObject object = object_;
  if (object.time.seconds() + 1e-6 < time.seconds()) {  // 1usec is allowed error
    getTrackedObject(time, object);
  }
  using autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  auto & pose_cov = object.pose_covariance;

  // principal component of the position covariance matrix
  Eigen::Matrix2d covariance;
  covariance << pose_cov[XYZRPY_COV_IDX::X_X], pose_cov[XYZRPY_COV_IDX::X_Y],
    pose_cov[XYZRPY_COV_IDX::Y_X], pose_cov[XYZRPY_COV_IDX::Y_Y];
  Eigen::EigenSolver<Eigen::Matrix2d> es(covariance);
  major_axis_sq = es.eigenvalues().real().maxCoeff();
  minor_axis_sq = es.eigenvalues().real().minCoeff();
}

bool Tracker::isConfident(const rclcpp::Time & time) const
{
  // check the number of measurements. if the measurement is too small, definitely not confident
  const int count = getTotalMeasurementCount();
  if (count < 2) {
    return false;
  }

  double major_axis_sq = 0.0;
  double minor_axis_sq = 0.0;
  getPositionCovarianceEigenSq(time, major_axis_sq, minor_axis_sq);

  // if the covariance is very small, the tracker is confident
  constexpr double STRONG_COV_SQ_THRESHOLD = 0.28;
  if (major_axis_sq < STRONG_COV_SQ_THRESHOLD) {
    // debug message
    {
      std::cout << "Tracker is strongly confident " << getUuidString().substr(0, 6) << " "
                << getTotalExistenceProbability() << ", axis_sq " << major_axis_sq << " x "
                << minor_axis_sq << std::endl;
    }
    return true;
  }

  // if the existence probability is high and the covariance is small enough, the tracker is
  // confident
  constexpr double WEAK_COV_SQ_THRESHOLD = 1.6;
  if (getTotalExistenceProbability() > 0.60 && major_axis_sq < WEAK_COV_SQ_THRESHOLD) {
    // debug message
    {
      std::cout << "Tracker is weakly confident " << getUuidString().substr(0, 6) << " "
                << getTotalExistenceProbability() << ", axis_sq " << major_axis_sq << " x "
                << minor_axis_sq << std::endl;
    }
    return true;
  }

  // debug message
  {
    std::cout << "Tracker is not confident " << getUuidString().substr(0, 6) << " "
              << getTotalExistenceProbability() << " , axis_sq " << major_axis_sq << " x "
              << minor_axis_sq << std::endl;
  }
  return false;
}

bool Tracker::isExpired(const rclcpp::Time & now) const
{
  // check the number of no measurements
  const double elapsed_time = getElapsedTimeFromLastUpdate(now);

  // if the last measurement is too old, the tracker is expired
  if (elapsed_time > 1.0) {
    // debug message
    double major_axis_sq = 0.0;
    double minor_axis_sq = 0.0;
    getPositionCovarianceEigenSq(now, major_axis_sq, minor_axis_sq);
    std::cout << "Tracker is expired " << getUuidString().substr(0, 6) << " " << elapsed_time
              << " , axis_sq " << major_axis_sq << " x " << minor_axis_sq << std::endl;
    return true;
  }

  // if the tracker is not confident, the tracker is expired
  const float existence_probability = getTotalExistenceProbability();
  if (existence_probability < 0.015) {
    // debug message
    double major_axis_sq = 0.0;
    double minor_axis_sq = 0.0;
    getPositionCovarianceEigenSq(now, major_axis_sq, minor_axis_sq);
    std::cout << "Tracker is expired " << getUuidString().substr(0, 6) << " " << elapsed_time
              << ", existence_probability " << existence_probability << " , axis_sq "
              << major_axis_sq << " x " << minor_axis_sq << std::endl;
    return true;
  }

  // if the tracker is a bit old and the existence probability is low, check the covariance size
  if (elapsed_time > 0.18 && existence_probability < 0.4) {
    // if the tracker covariance is too large, the tracker is expired
    double major_axis_sq = 0.0;
    double minor_axis_sq = 0.0;
    getPositionCovarianceEigenSq(now, major_axis_sq, minor_axis_sq);
    constexpr double MAJOR_COV_SQ_THRESHOLD = 1.8;
    constexpr double MINOR_COV_SQ_THRESHOLD = 1.2;
    if (major_axis_sq > MAJOR_COV_SQ_THRESHOLD || minor_axis_sq > MINOR_COV_SQ_THRESHOLD) {
      // debug message
      std::cout << "Tracker is expired " << getUuidString().substr(0, 6) << " " << elapsed_time
                << ", existence_probability " << existence_probability << " , axis_sq "
                << major_axis_sq << " x " << minor_axis_sq << std::endl;
      return true;
    }
  }

  return false;
}

float Tracker::getKnownObjectProbability() const
{
  // find unknown probability
  float unknown_probability = 0.0;
  for (const auto & a_class : object_.classification) {
    if (a_class.label == autoware_perception_msgs::msg::ObjectClassification::UNKNOWN) {
      unknown_probability = a_class.probability;
      break;
    }
  }
  // known object probability is reverse of unknown probability
  return 1.0 - unknown_probability;
}

double Tracker::getPositionCovarianceSizeSq() const
{
  using autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  auto & pose_cov = object_.pose_covariance;
  const double determinant = pose_cov[XYZRPY_COV_IDX::X_X] * pose_cov[XYZRPY_COV_IDX::Y_Y] -
                             pose_cov[XYZRPY_COV_IDX::X_Y] * pose_cov[XYZRPY_COV_IDX::Y_X];

  // position covariance size is square root of the determinant
  return determinant;
}