#include "xsim/sensors.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
using namespace xsim;
Ticket request(World &w, Op op, Key k = {}) {
  auto t = std::make_shared<Command>();
  t->op = op;
  t->key = k;
  return t;
}
void execute(World &w, const Ticket &t) {
  w.submit(t);
  w.boundary();
  assert(t->phase == 2);
}
std::shared_ptr<Entity> add(World &w, const char *kind,
                            const std::string &name) {
  auto e = std::make_shared<Entity>(parse_entity({{"name", name},
                                                  {"kind", kind},
                                                  {"position", {2, -1, 0}},
                                                  {"yaw", .35}}));
  auto t = request(w, Op::Add);
  t->prepared =
      std::make_unique<Prepared>(Prepared{e, prepare_model(e->config)});
  execute(w, t);
  assert(t->result.success);
  return e;
}
void start(World &w, const std::shared_ptr<Entity> &e) {
  auto t = request(w, Op::Provider, {e->id, e->generation});
  t->action = 1;
  t->prepared =
      std::make_unique<Prepared>(Prepared{e, prepare_model(e->config)});
  execute(w, t);
  assert(t->result.success && e->enabled);
}
int main() {
  // Output deadlines stay on the epoch grid even when dt does not divide the
  // output period. This is the original plant's 3 ms / 10 ms schedule.
  World rate_grid(1700000000000000000LL,3000000,10000000);
  Frame output;std::vector<int64_t> stamps;
  for(int k=0;k<20;++k){rate_grid.advance();if(rate_grid.take_frame(output))stamps.push_back(output.stamp-rate_grid.epoch);}
  assert((stamps==std::vector<int64_t>{12000000,21000000,30000000,42000000,51000000,60000000}));
  World paced(1700000000000000000LL, 10000000, 10000000);
  const auto wall_start = Clock::now();
  paced.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  assert(paced.metrics.steps == 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  const auto completed = paced.metrics.steps.load();
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           Clock::now() - wall_start)
                           .count();
  assert(completed > 0 && completed * 10000000 <= uint64_t(elapsed));
  paced.stop();
  World w(1700000000000000000LL);
  std::vector<std::shared_ptr<Entity>> entities;
  for (const char *kind : {"fs150", "scout", "mecanum"}) {
    auto e = add(w, kind, kind);
    entities.push_back(e);
    start(w, e);
    Key old{e->id, e->generation};
    auto stale =
        request(w, e->config.kind == Kind::FS150 ? Op::Arm : Op::Velocity, old);
    stale->arm = true;
    stale->velocity = {1, 1, 1};
    stale->at = w.metrics.sim_ns + 100000000;
    // Pending old-generation command must not affect reset entity.
    if (stale->op == Op::Arm) {
      stale->op = Op::Pva;
      stale->pva.coordinate_frame = 1;
      stale->pva.type_mask = 3527;
      stale->pva.velocity[0] = 1;
    }
    w.submit(stale);
    auto reset = request(w, Op::Reset, old);
    reset->prepared =
        std::make_unique<Prepared>(Prepared{e, prepare_model(e->config)});
    execute(w, reset);
    assert(e->generation == old.generation + 1);
    for (int k = 0; k < 110; ++k) {
      w.boundary();
      w.advance();
    }
    assert(stale->phase == 2 && !stale->result.success &&
           stale->result.reason == 1);
    auto expired = request(w, Op::Arm, {e->id, e->generation});
    expired->arm = true;
    expired->deadline = Clock::now() - std::chrono::milliseconds(1);
    w.submit(expired);
    assert(!World::wait(expired));
    w.boundary();
    assert(expired->phase == 3);
    auto stop = request(w, Op::Provider, {e->id, e->generation});
    stop->action = 2;
    execute(w, stop);
    assert(!e->enabled);
    auto oldstop = request(w, Op::Provider, old);
    oldstop->action = 2;
    execute(w, oldstop);
    assert(!oldstop->result.success);
    start(w, e);
  }
  // Dense swap after removing first of the same kind, then command survivor.
  auto a = add(w, "mecanum", "a"), b = add(w, "mecanum", "b");
  start(w, a);
  start(w, b);
  auto remove = request(w, Op::Remove, {a->id, a->generation});
  execute(w, remove);
  assert(!a->alive);
  auto replacement = add(w, "mecanum", "a");
  assert(replacement->id != a->id);
  start(w, replacement);
  auto old = request(w, Op::Velocity, {a->id, a->generation});
  old->velocity = {1, 0, 0};
  execute(w, old);
  assert(!old->result.success);
  auto move = request(w, Op::Velocity, {b->id, b->generation});
  move->velocity = {1, 0, 0};
  execute(w, move);
  w.advance();
  for (auto &s : w.capture().states) {
    if (s.key.id == b->id)
      assert(s.velocity.norm() > .9);
    if (s.key.id == replacement->id)
      assert(s.velocity.norm() == 0);
  }
  // Caller can be slow/hold its synchronization lock; world completion cannot
  // wait.
  auto held = request(w, Op::Arm, {entities[0]->id, entities[0]->generation});
  held->arm = true;
  std::unique_lock<std::mutex> held_lock(held->mutex);
  execute(w, held);
  assert(held->result.success);
  held_lock.unlock();
  auto pause = request(w, Op::Pause);
  execute(w, pause);
  const auto before = w.metrics.steps.load();
  w.start();
  auto paused_add = request(w, Op::Add);
  auto pe = std::make_shared<Entity>(
      parse_entity({{"name", "paused"}, {"kind", "scout"}}));
  paused_add->prepared =
      std::make_unique<Prepared>(Prepared{pe, prepare_model(pe->config)});
  w.submit(paused_add);
  assert(World::wait(paused_add));
  assert(w.metrics.steps == before);
  auto step = request(w, Op::Step);
  step->steps = 7;
  w.submit(step);
  assert(World::wait(step));
  assert(w.metrics.steps == before + 7);
  w.stop();
  // Real immutable-scene CPU scans, mount transform, generation and bounded
  // pending.
  Sensors sensors({{"obstacles", Json::array({{{"type", "box"},
                                               {"position", {4, 0, 0}},
                                               {"size", {1, 2, 2}}}})}},
                  2);
  auto se =
      std::make_shared<Entity>(parse_entity({{"name", "sensor"},
                                             {"kind", "scout"},
                                             {"sensor",
                                              {{"backend", "cpu"},
                                               {"h_fov_deg", 90},
                                               {"v_fov_deg", 30},
                                               {"h_res", 1000},
                                               {"v_res", 200},
                                               {"rate_hz", 1000},
                                               {"translation", {1, 0, 0}}}}}));
  se->id = 9;
  se->alive = true;
  se->enabled = true;
  se->generation = 1;
  se->sensor = sensors.prepare(se);
  State s;
  s.entity = se;
  s.key = {9, 1};
  s.enabled = true;
  for (int k = 0; k < 100; ++k) {
    s.stamp = 1700000000000000000LL + k * 1000000;
    sensors.submit(s);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  Sample sampled;
  std::vector<uint8_t> cloud;
  assert(Sensors::take(se->sensor, sampled, cloud));
  assert(sampled.position.x() == 1);
  assert(sampled.scene_version == 1);
  assert(!cloud.empty());
  assert(se->sensor->misses > 0);
  s.stamp += 1000000;
  sensors.submit(s);
  ++se->generation;
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  assert(!Sensors::take(se->sensor, sampled, cloud));
  auto sensor_status = sensors.status();
  sensors.stop();
  std::cout << Json({{"result", "PASS"},
                     {"checks",
                      {"three_kinds", "reset_old_generation", "timeout_cancel",
                       "swap_remove", "old_id", "slow_waiter",
                       "paused_management", "step_exact", "cpu_scan",
                       "mount_pose", "pending_replacement", "stale_scan"}},
                     {"sensors", sensor_status}})
                   .dump()
            << '\n';
}
