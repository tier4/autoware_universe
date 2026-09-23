#include "autoware/tensorrt_e2e/timestamped_buffer.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  using autoware::tensorrt_e2e::TimestampedBuffer;
  TimestampedBuffer<double> buffer;
  for (int i = 0; i <= 100; ++i)
    buffer.insert(i * 10000000LL, i * 0.1, 300000000LL);
  assert(buffer.samples.front().stamp_ns == 700000000LL);
  auto b = buffer.bracket(955000000LL);
  assert(b);
  const auto &lo = buffer.samples[b->first];
  const auto &hi = buffer.samples[b->second];
  const double alpha =
      double(955000000LL - lo.stamp_ns) / (hi.stamp_ns - lo.stamp_ns);
  assert(std::abs((lo.value + alpha * (hi.value - lo.value)) - 9.55) < 1e-9);
  assert(!buffer.bracket(1001000000LL));
  assert(!buffer.bracket(699000000LL));
  auto n = buffer.samples.size();
  buffer.insert(1000000000LL, 42, 300000000LL);
  assert(buffer.samples.size() == n && buffer.samples.back().value == 42);
  assert(*buffer.at_or_before(1000000001LL, 100) == 42);
  assert(!buffer.at_or_before(1000000200LL, 100));
  assert(!buffer.at_or_before(0, 100));
  assert(buffer.insert(0, 0, 300000000LL));
  assert(buffer.samples.size() == 1);
  assert(buffer.bracket(0)->first == 0);
  // Irregular 50 Hz odometry, including missing samples, must still support
  // the training's 31-pose grid anchored to a non-grid LiDAR timestamp.
  TimestampedBuffer<double> history;
  for (int i = 0; i <= 200; ++i) {
    if (i == 53 || i == 109) continue;
    const int64_t stamp = i * 20000000LL + (i % 3) * 1000000LL;
    history.insert(stamp, stamp * 1e-9 * 10.0, 4000000000LL);
  }
  const int64_t lidar_stamp = 3955000000LL;
  for (int step = 0; step < 31; ++step) {
    const int64_t target = lidar_stamp - (30 - step) * 100000000LL;
    const auto bracket = history.bracket(target);
    assert(bracket);
    const auto & a = history.samples[bracket->first];
    const auto & b = history.samples[bracket->second];
    const double ratio = a.stamp_ns == b.stamp_ns ? 0.0 :
      double(target - a.stamp_ns) / (b.stamp_ns - a.stamp_ns);
    const double position = a.value + ratio * (b.value - a.value);
    assert(std::abs(position - target * 1e-9 * 10.0) < 1e-9);
  }
  std::cout << "31-pose 10 Hz grid from irregular odometry: PASS\n";
  std::cout << "Timestamp buffer: interpolation, duration retention, "
               "duplicate, bounds, staleness and reset PASS\n";
}
