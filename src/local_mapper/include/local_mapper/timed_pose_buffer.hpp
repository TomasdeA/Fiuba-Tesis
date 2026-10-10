#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>

namespace local_mapper {
// Timestamped planar poses. Never extrapolate across missing odometry.
class TimedPoseBuffer {
 public:
  struct Pose { int64_t stamp; double x, z, yaw; };
  enum class Status { Ready, Pending, TooOld, Gap };
  explicit TimedPoseBuffer(std::size_t capacity = 200) : capacity_(capacity) {
    if (capacity_ < 2) throw std::invalid_argument("pose capacity must be >= 2");
  }
  void insert(Pose pose) {
    auto it = std::lower_bound(poses_.begin(), poses_.end(), pose.stamp,
        [](const Pose& p, int64_t t) { return p.stamp < t; });
    if (it != poses_.end() && it->stamp == pose.stamp) *it = pose;
    else poses_.insert(it, pose);
    while (poses_.size() > capacity_) poses_.pop_front();
  }
  Status lookup(int64_t stamp, int64_t max_gap, Pose& out) const {
    auto hi = std::lower_bound(poses_.begin(), poses_.end(), stamp,
        [](const Pose& p, int64_t t) { return p.stamp < t; });
    if (hi == poses_.end()) return Status::Pending;
    if (hi->stamp == stamp) { out = *hi; return Status::Ready; }
    if (hi == poses_.begin()) return Status::TooOld;
    const auto& lo = *std::prev(hi);
    const auto gap = hi->stamp - lo.stamp;
    if (gap > max_gap) return Status::Gap;
    const double alpha = static_cast<double>(stamp - lo.stamp) / gap;
    const double dyaw = std::atan2(std::sin(hi->yaw - lo.yaw), std::cos(hi->yaw - lo.yaw));
    out = {stamp, lo.x + alpha*(hi->x-lo.x), lo.z + alpha*(hi->z-lo.z),
        lo.yaw + alpha*dyaw};
    return Status::Ready;
  }
 private:
  std::size_t capacity_;
  std::deque<Pose> poses_;
};
}  // namespace local_mapper
