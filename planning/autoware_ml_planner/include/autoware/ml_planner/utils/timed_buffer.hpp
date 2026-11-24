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

#ifndef AUTOWARE__ML_PLANNER__UTILS__TIMED_BUFFER_HPP_
#define AUTOWARE__ML_PLANNER__UTILS__TIMED_BUFFER_HPP_

#include <rclcpp/time.hpp>

#include <algorithm>
#include <deque>
#include <functional>
#include <utility>

namespace autoware::ml_planner::utils
{

/**
 * @brief Chronological history within a fixed interval behind the newest stamp.
 *
 * A late message can fill a gap in the retained history. The first message
 * received for each measurement stamp is kept.
 */
template <typename MsgT>
class TimedBuffer
{
public:
  using StampOf = std::function<rclcpp::Time(const MsgT &)>;

  TimedBuffer(const double window_s, StampOf stamp_of)
  : window_s_(window_s), stamp_of_(std::move(stamp_of))
  {
  }

  void push_back(const MsgT & msg)
  {
    const auto stamp = stamp_of_(msg);
    const auto newest = msgs_.empty() ? stamp : std::max(stamp, stamp_of_(msgs_.back()));
    if ((newest - stamp).seconds() > window_s_) {
      return;
    }
    const auto pos = std::lower_bound(
      msgs_.begin(), msgs_.end(), stamp,
      [this](const MsgT & entry, const rclcpp::Time & time) { return stamp_of_(entry) < time; });
    if (pos != msgs_.end() && stamp_of_(*pos) == stamp) {
      return;
    }
    msgs_.insert(pos, msg);
    while ((newest - stamp_of_(msgs_.front())).seconds() > window_s_) {
      msgs_.pop_front();
    }
  }

  void clear() { msgs_.clear(); }
  [[nodiscard]] bool empty() const { return msgs_.empty(); }
  [[nodiscard]] const MsgT & back() const { return msgs_.back(); }

  /// Chronological messages within the time window.
  [[nodiscard]] const std::deque<MsgT> & msgs() const { return msgs_; }

private:
  double window_s_;
  StampOf stamp_of_;
  std::deque<MsgT> msgs_;
};

}  // namespace autoware::ml_planner::utils
#endif  // AUTOWARE__ML_PLANNER__UTILS__TIMED_BUFFER_HPP_
