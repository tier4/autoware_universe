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

#ifndef AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__FIXED_TIME_GRID_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__FIXED_TIME_GRID_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{

/**
 * @brief A bounded, timestamp-ordered buffer for zero-order-hold sampling.
 *
 * The training converter selects the latest message whose bag timestamp is at or before each
 * fixed-rate tick. This class contains that selection rule without depending on ROS types, so it
 * can be tested independently of a ROS executor. The online caller right-aligns the grid to its
 * current planner clock: the converter's absolute origin is bag-specific and is not observable by
 * a live node, while the interval and zero-order-hold rules are preserved exactly.
 *
 * Messages are ordered by selection_time_ns, not by their message header.  The caller is
 * responsible for supplying the clock that corresponds to the training converter's bag/arrival
 * clock.  The message header remains payload metadata and is not used for selection.
 */
template <typename MessageT>
class FixedTimeGridBuffer
{
public:
  using MessagePtr = std::shared_ptr<const MessageT>;

  struct TimedMessage
  {
    int64_t selection_time_ns{0};
    MessagePtr message;
  };

  struct GridSample
  {
    // Exactly time_length entries. A null entry means that no message existed at or before that
    // tick (startup padding); the caller must treat it as an empty frame rather than skipping the
    // tick.
    std::vector<MessagePtr> messages;
    // The selected source time for each entry, or nullopt for a startup-padding entry.
    std::vector<std::optional<int64_t>> selected_time_ns;
  };

  explicit FixedTimeGridBuffer(const int64_t retention_ns) : retention_ns_(retention_ns)
  {
    if (retention_ns_ < 0) {
      throw std::invalid_argument("FixedTimeGridBuffer retention must be non-negative");
    }
  }

  void clear() noexcept { messages_.clear(); }

  /**
   * @brief Insert a message in source-time order.
   *
   * Equal timestamps are kept in insertion order. Therefore, the last message at an equal
   * timestamp is selected, matching the converter's forward cursor over bag order.
   */
  void add(const int64_t selection_time_ns, const MessagePtr & message)
  {
    if (!message) {
      return;
    }

    const TimedMessage timed_message{selection_time_ns, message};
    const auto insert_position = std::upper_bound(
      messages_.begin(), messages_.end(), selection_time_ns,
      [](const int64_t time_ns, const TimedMessage & entry) {
        return time_ns < entry.selection_time_ns;
      });
    messages_.insert(insert_position, timed_message);

    // Keep one message before the retention boundary so a target tick exactly at the beginning of
    // the retained window can still perform a zero-order hold.
    const int64_t newest_time_ns = messages_.back().selection_time_ns;
    const int64_t cutoff_ns = newest_time_ns > retention_ns_ ? newest_time_ns - retention_ns_
                                                             : std::numeric_limits<int64_t>::min();
    while (messages_.size() > 1 && messages_[1].selection_time_ns < cutoff_ns) {
      messages_.pop_front();
    }
  }

  [[nodiscard]] MessagePtr latest_at_or_before(const int64_t reference_time_ns) const
  {
    const auto selected = latest_entry_at_or_before(reference_time_ns);
    return selected ? selected->message : nullptr;
  }

  [[nodiscard]] std::optional<int64_t> latest_time_ns_at_or_before(
    const int64_t reference_time_ns) const
  {
    const auto selected = latest_entry_at_or_before(reference_time_ns);
    return selected ? std::make_optional(selected->selection_time_ns) : std::nullopt;
  }

  /**
   * @brief Sample a fixed-rate, right-aligned time grid using zero-order hold.
   *
   * For a 31-step, 10 Hz grid and reference_time_ns T, the targets are
   * T - 3.0 s, T - 2.9 s, ..., T. No future message is ever selected. The caller should not round
   * T to a global epoch grid unless it also buffers the other model inputs, because that would
   * make the neighbor current state older than the ego current state by up to one period.
   */
  [[nodiscard]] GridSample sample(
    const int64_t reference_time_ns, const size_t time_length, const int64_t period_ns) const
  {
    if (time_length == 0) {
      return {};
    }
    if (period_ns <= 0) {
      throw std::invalid_argument("FixedTimeGridBuffer period must be positive");
    }

    GridSample result;
    result.messages.resize(time_length);
    result.selected_time_ns.resize(time_length);

    size_t cursor = 0;
    std::optional<size_t> selected_index;
    for (size_t i = 0; i < time_length; ++i) {
      const int64_t steps_from_reference = static_cast<int64_t>(time_length - 1 - i);
      const int64_t target_time_ns = reference_time_ns - steps_from_reference * period_ns;

      while (cursor < messages_.size() && messages_[cursor].selection_time_ns <= target_time_ns) {
        selected_index = cursor;
        ++cursor;
      }

      if (selected_index) {
        result.messages[i] = messages_[*selected_index].message;
        result.selected_time_ns[i] = messages_[*selected_index].selection_time_ns;
      }
    }
    return result;
  }

  [[nodiscard]] size_t size() const noexcept { return messages_.size(); }

private:
  [[nodiscard]] std::optional<TimedMessage> latest_entry_at_or_before(
    const int64_t reference_time_ns) const
  {
    const auto it = std::upper_bound(
      messages_.begin(), messages_.end(), reference_time_ns,
      [](const int64_t time_ns, const TimedMessage & entry) {
        return time_ns < entry.selection_time_ns;
      });
    if (it == messages_.begin()) {
      return std::nullopt;
    }
    return *(it - 1);
  }

  int64_t retention_ns_{0};
  std::deque<TimedMessage> messages_;
};

}  // namespace autoware::diffusion_planner::preprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__FIXED_TIME_GRID_HPP_
