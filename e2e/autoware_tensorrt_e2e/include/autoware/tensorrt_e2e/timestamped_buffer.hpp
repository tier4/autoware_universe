// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE__TENSORRT_E2E__TIMESTAMPED_BUFFER_HPP_
#define AUTOWARE__TENSORRT_E2E__TIMESTAMPED_BUFFER_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <optional>
#include <utility>

namespace autoware::tensorrt_e2e {
// Sensor time, not callback count. Retain one predecessor at the time-window
// boundary so interpolation remains possible at the oldest requested timestamp.
template <class T> class TimestampedBuffer {
public:
  struct Sample {
    int64_t stamp_ns;
    T value;
  };
  std::deque<Sample> samples;

  bool insert(int64_t stamp_ns, const T &value, int64_t keep_ns) {
    const bool rewound = !samples.empty() && stamp_ns < samples.back().stamp_ns;
    if (rewound)
      samples.clear();
    if (!samples.empty() && stamp_ns == samples.back().stamp_ns) {
      samples.back().value = value;
    } else {
      samples.push_back({stamp_ns, value});
    }
    while (samples.size() > 2 && samples[1].stamp_ns <= stamp_ns - keep_ns) {
      samples.pop_front();
    }
    return rewound;
  }

  std::optional<std::pair<size_t, size_t>> bracket(int64_t target_ns) const {
    if (samples.empty() || target_ns < samples.front().stamp_ns ||
        target_ns > samples.back().stamp_ns)
      return std::nullopt;
    const auto hi = std::lower_bound(samples.begin(), samples.end(), target_ns,
                                     [](const Sample &sample, int64_t stamp) {
                                       return sample.stamp_ns < stamp;
                                     });
    const size_t index = static_cast<size_t>(hi - samples.begin());
    return std::make_pair(hi->stamp_ns == target_ns ? index : index - 1, index);
  }

  const T *at_or_before(int64_t target_ns, int64_t max_age_ns) const {
    const auto after =
        std::upper_bound(samples.begin(), samples.end(), target_ns,
                         [](int64_t stamp, const Sample &sample) {
                           return stamp < sample.stamp_ns;
                         });
    if (after == samples.begin())
      return nullptr;
    const auto &sample = *std::prev(after);
    return target_ns - sample.stamp_ns <= max_age_ns ? &sample.value : nullptr;
  }
};
} // namespace autoware::tensorrt_e2e
#endif
