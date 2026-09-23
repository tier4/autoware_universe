// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>
namespace autoware::tensorrt_e2e {
class RollingLatency {
public:
  void add(double ms) {
    if (!std::isfinite(ms) || ms < 0)
      return;
    values_.push_back(ms);
    if (values_.size() > 512)
      values_.pop_front();
  }
  double percentile(double p) const {
    if (values_.empty())
      return 0.0;
    std::vector<double> sorted(values_.begin(), values_.end());
    std::sort(sorted.begin(), sorted.end());
    return sorted[static_cast<size_t>(std::ceil(p * sorted.size())) - 1];
  }
  void clear() { values_.clear(); }

private:
  std::deque<double> values_;
};
} // namespace autoware::tensorrt_e2e
