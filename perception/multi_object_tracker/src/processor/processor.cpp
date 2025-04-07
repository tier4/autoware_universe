// Copyright 2024 TIER IV, Inc.
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

#include "multi_object_tracker/processor/processor.hpp"

#include "multi_object_tracker/tracker/tracker.hpp"
#include "object_recognition_utils/object_recognition_utils.hpp"

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <iterator>

using Label = autoware_perception_msgs::msg::ObjectClassification;

TrackerProcessor::TrackerProcessor(
  const std::map<std::uint8_t, std::string> & tracker_map, const size_t & channel_size)
: tracker_map_(tracker_map), channel_size_(channel_size)
{
  // Set tracker lifetime parameters
  max_elapsed_time_ = 1.0;  // [s]

  // Set tracker overlap remover parameters
  min_iou_ = 0.1;                       // [ratio]
  min_iou_for_unknown_object_ = 0.001;  // [ratio]
  distance_threshold_ = 5.0;            // [m]

  // Set tracker confidence threshold
  confident_count_threshold_ = 3;  // [count]
}

void TrackerProcessor::predict(const rclcpp::Time & time)
{
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    (*itr)->predict(time);
  }
}

void TrackerProcessor::update(
  const autoware_perception_msgs::msg::DetectedObjects & detected_objects,
  const geometry_msgs::msg::Transform & self_transform,
  const std::unordered_map<int, int> & direct_assignment, const uint & channel_index)
{
  int tracker_idx = 0;
  const auto & time = detected_objects.header.stamp;
  for (auto tracker_itr = list_tracker_.begin(); tracker_itr != list_tracker_.end();
       ++tracker_itr, ++tracker_idx) {
    if (direct_assignment.find(tracker_idx) != direct_assignment.end()) {  // found
      const auto & associated_object =
        detected_objects.objects.at(direct_assignment.find(tracker_idx)->second);
      (*(tracker_itr))
        ->updateWithMeasurement(associated_object, time, self_transform, channel_index);
    } else {  // not found
      (*(tracker_itr))->updateWithoutMeasurement(time);
    }
  }
}

void TrackerProcessor::spawn(
  const autoware_perception_msgs::msg::DetectedObjects & detected_objects,
  const geometry_msgs::msg::Transform & self_transform,
  const std::unordered_map<int, int> & reverse_assignment, const uint & channel_index)
{
  const auto & time = detected_objects.header.stamp;
  for (size_t i = 0; i < detected_objects.objects.size(); ++i) {
    if (reverse_assignment.find(i) != reverse_assignment.end()) {  // found
      continue;
    }
    const auto & new_object = detected_objects.objects.at(i);
    std::shared_ptr<Tracker> tracker =
      createNewTracker(new_object, time, self_transform, channel_index);
    if (tracker) list_tracker_.push_back(tracker);
  }
}

std::shared_ptr<Tracker> TrackerProcessor::createNewTracker(
  const autoware_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & self_transform, const uint & channel_index) const
{
  const std::uint8_t label = object_recognition_utils::getHighestProbLabel(object.classification);
  if (tracker_map_.count(label) != 0) {
    const auto tracker = tracker_map_.at(label);
    if (tracker == "bicycle_tracker")
      return std::make_shared<BicycleTracker>(
        time, object, self_transform, channel_size_, channel_index);
    if (tracker == "big_vehicle_tracker")
      return std::make_shared<BigVehicleTracker>(
        time, object, self_transform, channel_size_, channel_index);
    if (tracker == "multi_vehicle_tracker")
      return std::make_shared<MultipleVehicleTracker>(
        time, object, self_transform, channel_size_, channel_index);
    if (tracker == "normal_vehicle_tracker")
      return std::make_shared<NormalVehicleTracker>(
        time, object, self_transform, channel_size_, channel_index);
    if (tracker == "pass_through_tracker")
      return std::make_shared<PassThroughTracker>(
        time, object, self_transform, channel_size_, channel_index);
    if (tracker == "pedestrian_and_bicycle_tracker")
      return std::make_shared<PedestrianAndBicycleTracker>(
        time, object, self_transform, channel_size_, channel_index);
    if (tracker == "pedestrian_tracker")
      return std::make_shared<PedestrianTracker>(
        time, object, self_transform, channel_size_, channel_index);
  }
  return std::make_shared<UnknownTracker>(
    time, object, self_transform, channel_size_, channel_index);
}

void TrackerProcessor::prune(const rclcpp::Time & time)
{
  // Check tracker lifetime: if the tracker is old, delete it
  removeOldTracker(time);
  // Check tracker overlap: if the tracker is overlapped, delete the one with lower IOU
  removeOverlappedTracker(time);
}

void TrackerProcessor::removeOldTracker(const rclcpp::Time & time)
{
  // Check elapsed time from last update
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    const bool is_old = max_elapsed_time_ < (*itr)->getElapsedTimeFromLastUpdate(time);
    // If the tracker is old, delete it
    if (is_old) {
      auto erase_itr = itr;
      --itr;
      list_tracker_.erase(erase_itr);
    }
  }
}

// This function removes overlapped trackers based on distance and IoU criteria
void TrackerProcessor::removeOverlappedTracker(const rclcpp::Time & time)
{
  // Create sorted list with non-UNKNOWN objects first, then by measurement count
  std::vector<std::shared_ptr<Tracker>> sorted_list_tracker(
    list_tracker_.begin(), list_tracker_.end());
  std::sort(
    sorted_list_tracker.begin(), sorted_list_tracker.end(),
    [&time](const std::shared_ptr<Tracker> & a, const std::shared_ptr<Tracker> & b) {
      bool a_unknown = (a->getHighestProbLabel() == Label::UNKNOWN);
      bool b_unknown = (b->getHighestProbLabel() == Label::UNKNOWN);
      if (a_unknown != b_unknown) {
        return b_unknown;  // Put non-UNKNOWN objects first
      }
      if (a->getTotalMeasurementCount() != b->getTotalMeasurementCount()) {
        return a->getTotalMeasurementCount() >
               b->getTotalMeasurementCount();  // Then sort by measurement count
      }
      return a->getElapsedTimeFromLastUpdate(time) <
             b->getElapsedTimeFromLastUpdate(time);  // Finally sort by elapsed time (smaller first)
    });

  /* Iterate through the list of trackers */
  for (size_t i = 0; i < sorted_list_tracker.size(); ++i) {
    autoware_perception_msgs::msg::TrackedObject object1;
    if (!sorted_list_tracker[i]->getTrackedObject(time, object1)) continue;
    // Compare the current tracker with the remaining trackers
    for (size_t j = i + 1; j < sorted_list_tracker.size(); ++j) {
      autoware_perception_msgs::msg::TrackedObject object2;
      if (!sorted_list_tracker[j]->getTrackedObject(time, object2)) continue;
      // Calculate the distance between the two objects
      const double distance = std::hypot(
        object1.kinematics.pose_with_covariance.pose.position.x -
          object2.kinematics.pose_with_covariance.pose.position.x,
        object1.kinematics.pose_with_covariance.pose.position.y -
          object2.kinematics.pose_with_covariance.pose.position.y);

      // If the distance is too large, skip
      if (distance > distance_threshold_) {
        continue;
      }

      // Check the Intersection over Union (IoU) between the two objects
      const double min_union_iou_area = 1e-2;
      const auto iou = object_recognition_utils::get2dIoU(object1, object2, min_union_iou_area);
      const auto & label1 = sorted_list_tracker[i]->getHighestProbLabel();
      const auto & label2 = sorted_list_tracker[j]->getHighestProbLabel();
      bool delete_candidate_tracker = false;

      // If both trackers are UNKNOWN, delete the younger tracker
      // If one side of the tracker is UNKNOWN, delete UNKNOWN objects
      if (label1 == Label::UNKNOWN || label2 == Label::UNKNOWN) {
        if (min_iou_for_unknown_object_ < iou) {
          if (label2 == Label::UNKNOWN) {
            delete_candidate_tracker = true;
          }
        }
      } else {  // If neither is UNKNOWN, delete the one with lower IOU.
        if (min_iou_ < iou) {
          /* erase only when prioritized one has a measurement */
          delete_candidate_tracker = true;
        }
      }

      if (delete_candidate_tracker) {
        /* erase only when prioritized one has later(or equal time) meas than the other's */
        if (
          sorted_list_tracker[i]->getElapsedTimeFromLastUpdate(time) <=
          sorted_list_tracker[j]->getElapsedTimeFromLastUpdate(time)) {
          // Remove from original list_tracker
          list_tracker_.remove(sorted_list_tracker[j]);
          // Remove from sorted list
          sorted_list_tracker.erase(sorted_list_tracker.begin() + j);
          --j;
        }
      }
    }
  }
}

bool TrackerProcessor::isConfidentTracker(const std::shared_ptr<Tracker> & tracker) const
{
  // Confidence is determined by counting the number of measurements.
  // If the number of measurements is equal to or greater than the threshold, the tracker is
  // considered confident.
  return tracker->getTotalMeasurementCount() >= confident_count_threshold_;
}

void TrackerProcessor::getTrackedObjects(
  const rclcpp::Time & time, autoware_perception_msgs::msg::TrackedObjects & tracked_objects) const
{
  tracked_objects.header.stamp = time;
  for (const auto & tracker : list_tracker_) {
    // Skip if the tracker is not confident
    if (!isConfidentTracker(tracker)) continue;
    // Get the tracked object, extrapolated to the given time
    autoware_perception_msgs::msg::TrackedObject tracked_object;
    if (tracker->getTrackedObject(time, tracked_object)) {
      tracked_objects.objects.push_back(tracked_object);
    }
  }
}

void TrackerProcessor::getTentativeObjects(
  const rclcpp::Time & time,
  autoware_perception_msgs::msg::TrackedObjects & tentative_objects) const
{
  tentative_objects.header.stamp = time;
  for (const auto & tracker : list_tracker_) {
    if (!isConfidentTracker(tracker)) {
      autoware_perception_msgs::msg::TrackedObject tracked_object;
      if (tracker->getTrackedObject(time, tracked_object)) {
        tentative_objects.objects.push_back(tracked_object);
      }
    }
  }
}
