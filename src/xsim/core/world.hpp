#pragma once
#include "commands.hpp"
#include <deque>
#include <functional>
#include <list>
#include <thread>
#include <unordered_map>

namespace xsim {
class World {
public:
  World(int64_t epoch, int64_t dt = 1000000, int64_t output = 10000000,
        unsigned catchup = 8);
  ~World();
  World(const World &) = delete;
  void submit(const Ticket &); // nonworld threads; continuous inputs coalesce
                               // by entity/type/time
  static bool wait(const Ticket &);
  void boundary(); // world owner only, also used by deterministic fixtures
  void advance();
  Frame capture() const;    // world owner only (fixtures)
  bool take_frame(Frame &); // output owner; never holds lock while publishing
  void start();
  void stop();
  Metrics metrics;
  const int64_t epoch, dt, output_period;
  std::function<void(const State &)> sample_sensor;

private:
  struct Slot {
    std::shared_ptr<Entity> entity;
    size_t dense = 0;
  };
  std::unordered_map<uint64_t, Slot> slots_;
  std::vector<uint64_t> flight_ids_, scout_ids_, mecanum_ids_;
  BodyColumns bodies_;
  PlanarColumns scout_poses_, mecanum_poses_;
  std::vector<Flight> flights_;
  std::vector<Scout> scouts_;
  std::vector<Mecanum> mecanums_;
  uint64_t next_id_ = 1, steps_ = 0, stepping_ = 0, revision_ = 0,
           emitted_revision_ = UINT64_MAX;
  int64_t emitted_stamp_ = -1;
  std::deque<Ticket> step_waiters_;
  int64_t time_, next_output_;
  unsigned catchup_;
  std::mutex input_mutex_, output_mutex_;
  std::list<Ticket> inbox_, commands_;
  Frame ready_, scratch_;
  bool ready_available_ = false;
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::condition_variable wake_;
  std::mutex wake_mutex_;
  State state(uint64_t, const Slot &) const;
  Result apply(Command &);
  void reset(Slot &, Model &&);
  void emit();
  void finish(const Ticket &, Result);
  void run();
};
} // namespace xsim
