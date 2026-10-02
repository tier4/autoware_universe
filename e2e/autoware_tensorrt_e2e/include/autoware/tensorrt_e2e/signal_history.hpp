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

#ifndef AUTOWARE__TENSORRT_E2E__SIGNAL_HISTORY_HPP_
#define AUTOWARE__TENSORRT_E2E__SIGNAL_HISTORY_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::tensorrt_e2e
{

inline constexpr const char * LANE_SIGNALS_TENSOR = "lane_signals";
inline constexpr const char * ROUTE_SIGNALS_TENSOR = "route_signals";
//! green, yellow, red, unknown, no traffic light: the rows' one-hot channels.
inline constexpr int SIGNAL_STATES = 5;

/**
 * Every lane (or route) row's traffic-light one-hot over the last seconds, `[rows, steps, 5]`
 * oldest first, each row followed by its lanelet (its segment index) rather than its slot,
 * as OnePlanner's loader reads it (`oneplanner.data.t4direct.signal_history` over the
 * devkit's `T4WindowBuilder._read_signals`). The package states `steps` and `step_s`
 * (`context.signal_history.*`, the contract's `signal_history` block):
 *
 *   every tick: record (stamp, {segment: the one-hot its row carries}) for the selected rows
 *   step k of a row (0 oldest) = its segment's one-hot in the record nearest
 *     stamp - (steps - 1 - k) step_s, within step_s / 2; all zeros when there is no such
 *     record or the segment was not selected in it; the last step is this tick's own row
 */
class SignalHistory
{
public:
  using State = std::array<float, SIGNAL_STATES>;

  SignalHistory(const int steps, const double step_s)
  : steps_(steps), step_ns_(static_cast<int64_t>(step_s * 1e9 + 0.5))
  {
    if (steps < 1 || !(step_s > 0.0)) {
      throw std::runtime_error(
        "context.signal_history needs steps >= 1 and step_s > 0, got " + std::to_string(steps) +
        " and " + std::to_string(step_s));
    }
  }

  int steps() const { return steps_; }

  //! Forget every record: the segment indices name other lanelets once the map is rebuilt.
  void clear() { records_.clear(); }

  /**
   * @brief Record this tick's rows and return their timeline.
   * @param segments the segment index behind each selected row, in row order; rows past its
   *   end are padding and read all zeros.
   * @param rows_data the rows' tensor data, `[rows, points, channels]` row-major, whose
   *   channels `light_offset .. light_offset + 5` hold the one-hot (copied across points).
   * @return `[rows, steps, 5]` row-major.
   */
  std::vector<float> update(
    const int64_t stamp_ns, const std::vector<int64_t> & segments,
    const std::vector<float> & rows_data, const int64_t rows, const int64_t points,
    const int64_t channels, const int64_t light_offset)
  {
    const auto selected = static_cast<int64_t>(std::min<size_t>(segments.size(), rows));
    if (static_cast<int64_t>(rows_data.size()) < rows * points * channels) {
      throw std::runtime_error("signal history: the rows' data is shorter than their shape");
    }
    Record record{stamp_ns, {}};
    for (int64_t r = 0; r < selected; ++r) {
      State state{};
      const auto first = rows_data.begin() + r * points * channels + light_offset;
      std::copy(first, first + SIGNAL_STATES, state.begin());
      record.states[segments[r]] = state;
    }
    if (!records_.empty() && stamp_ns < records_.back().stamp_ns) {
      records_.clear();  // time went backwards (a replay restarted): nothing before is ours
    }
    if (!records_.empty() && stamp_ns == records_.back().stamp_ns) {
      records_.back() = std::move(record);  // the same cloud collected again
    } else {
      records_.push_back(std::move(record));
    }
    const int64_t reach = (steps_ - 1) * step_ns_ + step_ns_ / 2;
    while (records_.front().stamp_ns < stamp_ns - reach) {
      records_.pop_front();
    }

    std::vector<float> out(static_cast<size_t>(rows * steps_ * SIGNAL_STATES), 0.0f);
    for (int k = 0; k < steps_; ++k) {
      const int64_t target = stamp_ns - (steps_ - 1 - k) * step_ns_;
      const Record * nearest = nullptr;
      int64_t best = step_ns_ / 2;
      for (const auto & candidate : records_) {
        const int64_t gap = std::llabs(candidate.stamp_ns - target);
        if (gap <= best) {
          best = gap;
          nearest = &candidate;
        }
      }
      if (!nearest) {
        continue;
      }
      for (int64_t r = 0; r < selected; ++r) {
        const auto found = nearest->states.find(segments[r]);
        if (found != nearest->states.end()) {
          std::copy(
            found->second.begin(), found->second.end(),
            out.begin() + (r * steps_ + k) * SIGNAL_STATES);
        }
      }
    }
    return out;
  }

private:
  struct Record
  {
    int64_t stamp_ns;
    std::unordered_map<int64_t, State> states;
  };

  int steps_;
  int64_t step_ns_;
  std::deque<Record> records_;
};

/**
 * @brief Refuse a package whose graph and `context.signal_history.enabled` disagree.
 * @throws std::runtime_error when the graph reads one of `lane_signals` / `route_signals`
 * but not the other, reads them with the history off, or the history is on for a graph
 * that reads neither.
 */
inline void check_signal_history_inputs(
  const bool enabled, const bool graph_reads_lanes, const bool graph_reads_route)
{
  if (graph_reads_lanes != graph_reads_route) {
    throw std::runtime_error(
      std::string("The graph reads '") +
      (graph_reads_lanes ? LANE_SIGNALS_TENSOR : ROUTE_SIGNALS_TENSOR) + "' but not '" +
      (graph_reads_lanes ? ROUTE_SIGNALS_TENSOR : LANE_SIGNALS_TENSOR) +
      "': the exporter declares both or neither");
  }
  if (graph_reads_lanes && !enabled) {
    throw std::runtime_error(
      "The graph reads 'lane_signals' / 'route_signals' but context.signal_history.enabled is "
      "false: regenerate its ml_package file, which states the history");
  }
  if (!graph_reads_lanes && enabled) {
    throw std::runtime_error(
      "context.signal_history.enabled is true but the graph reads no 'lane_signals': the "
      "package and the graph come from different exports");
  }
}

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__SIGNAL_HISTORY_HPP_
