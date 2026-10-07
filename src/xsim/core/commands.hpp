#pragma once
#include "entity.hpp"
#include "systems/robots.hpp"
#include <condition_variable>
#include <mutex>

namespace xsim {
struct Prepared {
  std::shared_ptr<Entity> entity;
  Model model;
};
enum class Op {
  Add,
  Remove,
  Pause,
  Resume,
  Step,
  Reset,
  Provider,
  Arm,
  Mode,
  Pva,
  Attitude,
  Velocity
};
struct Result {
  bool applied = false, success = false, enabled = false;
  uint32_t reason = 0;
  Key key;
  uint64_t step = 0;
  int64_t stamp = 0;
};
struct Command {
  Op op = Op::Pause;
  Key key;
  int action = 0;
  bool arm = false;
  std::string mode;
  uint64_t steps = 1;
  int64_t at = 0;
  int64_t arrival_ns = 0; // realtime arrival guard; not a coalescing/event key
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()}; // forward,left,yaw rate
  xgc_position_target_v1 pva{};
  FlightAttitudeSetpoint attitude;
  std::unique_ptr<Prepared> prepared;
  std::shared_ptr<Entity> retired;
  std::vector<std::pair<Key, Model>> resets;
  Clock::time_point deadline = Clock::now() + std::chrono::milliseconds(1500);
  // 0 queued; 1 claimed; 2 applied; 3 cancelled. Cancellation wins before
  // claim, or caller waits for the already executing bounded boundary
  // operation.
  std::atomic<int> phase{0};
  Result result;
  std::mutex mutex;
  std::condition_variable done;
};
using Ticket = std::shared_ptr<Command>;
} // namespace xsim
