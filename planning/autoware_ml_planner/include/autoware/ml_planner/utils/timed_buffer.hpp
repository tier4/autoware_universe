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

#include <deque>
#include <functional>
#include <utility>

namespace autoware::ml_planner::utils
{

/**
 * @brief FIFO buffer of stamped messages bounded by a fixed time window.
 *
 * Messages are kept in chronological order. push_back() ignores messages that
 * are not strictly newer than the newest entry (deduplication by stamp) and
 * automatically drops entries older than the newest stamp minus the window,
 * so the buffer always holds at most window_s seconds of history.
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

  /**
   * @brief Append a message and trim the window.
   *
   * The message is ignored unless its stamp is strictly newer than the newest
   * entry, so pushing the same (not yet updated) message every cycle is safe.
   */
  void push_back(const MsgT & msg)
  {
    const double stamp_sec = stamp_of_(msg).seconds();
    if (!msgs_.empty() && stamp_sec <= stamp_of_(msgs_.back()).seconds()) {
      return;
    }
    msgs_.push_back(msg);
    while (stamp_of_(msgs_.front()).seconds() < stamp_sec - window_s_) {
      msgs_.pop_front();
    }
  }

  /**
   * @brief Append a message, or replace the newest entry when the stamps are equal.
   *
   * For a buffer whose newest entry is recomputed within the same frame (e.g. a frame pose that is
   * first seeded from a measurement and then replaced by a pose derived from it).
   */
  void push_or_replace_back(const MsgT & msg)
  {
    if (!msgs_.empty() && stamp_of_(msg).seconds() == stamp_of_(msgs_.back()).seconds()) {
      msgs_.back() = msg;
      return;
    }
    push_back(msg);
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
