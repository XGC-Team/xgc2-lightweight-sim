#pragma once
#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace xsim {
// Integer nanoseconds from one steady clock. Models receive a shared duration.
// The scheduler owns wall/simulation anchors and the adaptive period.
class PhysicsClock {
public:
  static constexpr int64_t minimum_step = 1000; // accumulate sub-microsecond tails
  PhysicsClock(int64_t nominal, int64_t maximum)
      : nominal_(nominal), maximum_(maximum), period_(nominal) {
    if (nominal < minimum_step || maximum < nominal || maximum > 20000000)
      throw std::invalid_argument("physics periods require 1 us <= nominal <= maximum <= 20 ms");
  }
  void rebase(int64_t wall, int64_t simulation) {
    wall_anchor_ = wall;
    sim_anchor_ = simulation;
    next_wake_ = wall > INT64_MAX - period_ ? INT64_MAX : wall + period_;
    window_wall_ = wall;
    cost_ = integrated_ = 0;
    overload_windows_ = recovery_windows_ = 0;
  }
  int64_t target(int64_t wall) const {
    const auto elapsed = std::max<int64_t>(0, wall - wall_anchor_);
    return sim_anchor_ > INT64_MAX - elapsed ? INT64_MAX : sim_anchor_ + elapsed;
  }
  int64_t step(int64_t target, int64_t simulation) const {
    const auto debt = std::max<int64_t>(0, target - simulation);
    if (debt < minimum_step) return 0;
    // Allow ordinary wake-up jitter in one step. Long stalls remain bounded.
    return std::min(debt, std::min(maximum_, period_ + period_ / 4));
  }
  void observe(int64_t duration, int64_t cost) {
    integrated_ += duration;
    cost_ += std::max<int64_t>(0, cost);
  }
  void schedule(int64_t wall) {
    if (wall - window_wall_ >= 100000000) {
      const double load = integrated_ ? double(cost_) / double(integrated_) : 0;
      overload_windows_ = load > .8 ? overload_windows_ + 1 : 0;
      recovery_windows_ = load < .5 ? recovery_windows_ + 1 : 0;
      if (overload_windows_ >= 2 && period_ < maximum_) {
        period_ = std::min(maximum_, period_ + std::max<int64_t>(1, period_ / 4));
        ++degradations_;
        recovery_windows_ = 0;
      } else if (recovery_windows_ >= 5 && period_ > nominal_) {
        period_ = std::max(nominal_, period_ - std::max<int64_t>(1, period_ / 20));
      }
      window_wall_ = wall;
      cost_ = integrated_ = 0;
    }
    // Absolute wall deadlines; computation consumes the period's budget.
    if (next_wake_ <= wall) {
      const auto missed = (wall - next_wake_) / period_ + 1;
      next_wake_ = missed > (INT64_MAX - next_wake_) / period_
                       ? INT64_MAX : next_wake_ + missed * period_;
    }
  }
  int64_t next_wake() const { return next_wake_; }
  int64_t period() const { return period_; }
  uint64_t degradations() const { return degradations_; }
private:
  const int64_t nominal_, maximum_;
  int64_t period_, wall_anchor_ = 0, sim_anchor_ = 0, next_wake_ = 0,
          window_wall_ = 0, cost_ = 0, integrated_ = 0;
  unsigned overload_windows_ = 0, recovery_windows_ = 0;
  uint64_t degradations_ = 0;
};
} // namespace xsim
