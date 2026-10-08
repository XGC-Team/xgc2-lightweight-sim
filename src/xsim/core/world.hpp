#pragma once
#include "commands.hpp"
#include "physics_clock.hpp"
#include <deque>
#include <exception>
#include <functional>
#include <list>
#include <thread>
#include <unordered_map>

namespace xsim {
class World {
public:
  World(int64_t epoch, int64_t dt = 2000000, int64_t output = 8000000,
        unsigned catchup = 8, int64_t maximum_step = 10000000);
  ~World();
  World(const World &) = delete;
  void submit(const Ticket &); // nonworld threads; continuous inputs coalesce
                               // by entity/type/time
  static bool wait(const Ticket &);
  void boundary(); // world owner only, also used by deterministic fixtures
  void advance();
  void advance(int64_t elapsed_ns); // explicit duration, world owner only
  Frame capture() const;    // world owner only (fixtures)
  static constexpr size_t frame_pool_size = 16;
  bool take_frame(std::shared_ptr<const Frame> &); // immutable shared output
  bool take_frame(Frame &); // fixture-only copy, outside the output lock
  void start(); // returns after the first native boundary and output snapshot
  void stop();
  Metrics metrics;
  const int64_t epoch, dt, output_period;

private:
  struct Slot {
    std::shared_ptr<Entity> entity;
    size_t dense = 0;
  };
  struct Geometry {
    Key key;
    bool enabled = false;
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  };
  std::unordered_map<uint64_t, Slot> slots_;
  std::vector<uint64_t> flight_ids_, scout_ids_, mecanum_ids_;
  BodyColumns bodies_;
  PlanarColumns scout_poses_, mecanum_poses_;
  std::vector<Flight> flights_;
  std::vector<uint8_t> flight_enabled_; // hot control gate, no entity/hash lookup
  std::vector<Scout> scouts_;
  std::vector<Mecanum> mecanums_;
  uint64_t next_id_ = 1, steps_ = 0, stepping_ = 0, revision_ = 0,
           emitted_revision_ = UINT64_MAX;
  int64_t emitted_stamp_ = -1;
  std::deque<Ticket> step_waiters_;
  int64_t time_, next_output_;
  unsigned catchup_;
  PhysicsClock physics_clock_;
  std::atomic<uint64_t> wake_revision_{0};
  std::atomic<int64_t> wall_offset_ns_{0};
  std::mutex input_mutex_, output_mutex_;
  std::list<Ticket> inbox_, commands_;
  // All owners/control blocks are created once. Consumers may retain const
  // frames; only a slot held exclusively by this pool can be rewritten.
  std::array<std::shared_ptr<Frame>, frame_pool_size> frame_pool_;
  std::shared_ptr<const Frame> ready_;
  size_t frame_cursor_ = 0;
  std::vector<Geometry> geometry_;
  uint64_t geometry_revision_ = 0;
  bool geometry_dirty_ = true;
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::condition_variable wake_;
  std::condition_variable started_wake_;
  std::mutex wake_mutex_;
  bool started_ = false;
  std::exception_ptr startup_error_;
  State state(uint64_t, const Slot &) const;
  Result apply(Command &);
  void reset(Slot &, Model &&);
  void reserve_frames(); // topology changes; geometrical capacity growth
  void observe_geometry(Frame &);
  void emit();
  void finish(const Ticket &, Result);
  void cancel_pending_controls(Key);
  void run();
};
} // namespace xsim
