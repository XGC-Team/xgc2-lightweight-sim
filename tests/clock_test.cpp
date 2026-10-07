#include "core/physics_clock.hpp"
#include "core/world.hpp"
#include <cassert>
#include <iostream>
using namespace xsim;
int main() {
  PhysicsClock clock(2000000, 10000000);
  clock.rebase(0, 0);
  assert(clock.step(clock.target(2350000), 0) == 2350000);
  assert(clock.step(clock.target(999), 0) == 0);
  assert(clock.step(clock.target(1001), 0) == 1001); // tail retained
  int64_t simulated = 0;
  const auto stalled_target = clock.target(50000000);
  unsigned count = 0;
  while (const auto h = clock.step(stalled_target, simulated)) {
    assert(h <= 2500000);
    simulated += h;
    ++count;
    clock.observe(h, 50000);
  }
  assert(simulated == 50000000 && count == 20);
  clock.schedule(150000000);
  assert(clock.period() == 2000000 && clock.degradations() == 0);
  // Sustained compute overload degrades smoothly and remains bounded.
  int64_t wall = 150000000;
  for (int k = 0; k < 20; ++k) {
    clock.observe(100000000, 120000000);
    wall += 100000000;
    const auto previous = clock.period();
    clock.schedule(wall);
    assert(clock.period() >= previous && clock.period() <= previous * 5 / 4);
  }
  assert(clock.period() == 10000000 && clock.degradations() > 0);
  for (int k = 0; k < 200; ++k) {
    clock.observe(100000000, 10000000);
    wall += 100000000;
    const auto previous = clock.period();
    clock.schedule(wall);
    assert(clock.period() <= previous && clock.period() >= previous * 95 / 100);
  }
  assert(clock.period() == 2000000);
  clock.rebase(wall + 10000000000LL, simulated);
  assert(clock.target(wall + 10000000000LL) == simulated); // pause has no debt
  clock.rebase(INT64_MAX - 500, INT64_MAX - 500);
  assert(clock.next_wake() == INT64_MAX);
  clock.schedule(INT64_MAX);
  assert(clock.next_wake() == INT64_MAX && clock.target(INT64_MAX) == INT64_MAX);

  // Timestamp and state advance by the actual shared dt, independent of count.
  World world(1700000000000000000LL);
  Config config; config.kind = Kind::Mecanum; config.name = "clock_body";
  auto entity = std::make_shared<Entity>(config);
  auto execute = [&](const Ticket &t) { world.submit(t); world.boundary(); assert(t->phase == 2 && t->result.success); };
  auto add = std::make_shared<Command>(); add->op = Op::Add;
  add->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(config)});
  execute(add);
  auto start = std::make_shared<Command>(); start->op = Op::Provider;
  start->key = {entity->id, entity->generation}; start->action = 1;
  start->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(config)});
  execute(start);
  auto move = std::make_shared<Command>(); move->op = Op::Velocity;
  move->key = {entity->id, entity->generation}; move->velocity = {1, 0, 0};
  execute(move);
  int64_t total = 0;
  for (const auto h : {2350000, 6000000, 10000000, 1700000, 1001}) {
    world.advance(h); total += h;
  }
  const auto frame = world.capture();
  assert(frame.steps == 5 && frame.stamp == world.epoch + total);
  assert(std::abs(frame.states.front().position.x() - double(total) * 1e-9) < 1e-12);
  const auto before = world.metrics.steps.load();
  auto pause = std::make_shared<Command>(); pause->op = Op::Pause; execute(pause);
  world.start();
  auto steps = std::make_shared<Command>(); steps->op = Op::Step; steps->steps = 7;
  world.submit(steps); assert(World::wait(steps));
  world.stop();
  assert(world.metrics.steps == before + 7);
  assert(world.metrics.sim_ns == frame.stamp + 7 * world.dt);

  // A real paced fleet shares one elapsed interval and keeps constant-twist
  // geometry aligned with the committed clock, without per-entity clocks.
  World fleet(world.epoch);
  for (int i = 0; i < 1000; ++i) {
    Config c; c.kind = Kind::Mecanum; c.name = "fleet_" + std::to_string(i);
    auto e = std::make_shared<Entity>(c);
    auto a = std::make_shared<Command>(); a->op = Op::Add;
    a->prepared = std::make_unique<Prepared>(Prepared{e, prepare_model(c)});
    fleet.submit(a); fleet.boundary(); assert(a->result.success);
    auto enable = std::make_shared<Command>(); enable->op = Op::Provider;
    enable->key = {e->id, e->generation}; enable->action = 1;
    enable->prepared = std::make_unique<Prepared>(Prepared{e, prepare_model(c)});
    fleet.submit(enable); fleet.boundary(); assert(enable->result.success);
    auto velocity = std::make_shared<Command>(); velocity->op = Op::Velocity;
    velocity->key = {e->id, e->generation}; velocity->velocity = {1, .2, .1};
    fleet.submit(velocity); fleet.boundary(); assert(velocity->result.success);
  }
  const auto wall_begin = Clock::now();
  fleet.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  fleet.stop();
  const auto wall_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - wall_begin).count();
  const auto fleet_frame = fleet.capture();
  const auto duration = fleet_frame.stamp - fleet.epoch;
  assert(duration > 0 && duration <= wall_duration && fleet_frame.states.size() == 1000);
  const auto expected = xgc2_math::stepBodyVelocity(xgc2_math::Pose2{}, {1, .2}, .1, double(duration) * 1e-9);
  for (const auto &s : fleet_frame.states) {
    assert((s.position.head<2>() - expected.position).norm() < 1e-10);
  }
  std::cout << "1000 Mecanum: simulation_ns=" << duration << " wall_ns=" << wall_duration
            << " rtf=" << double(duration) / double(wall_duration)
            << " period_ns=" << fleet.metrics.scheduling_period_ns
            << " last_dt_ns=" << fleet.metrics.last_dt_ns << '\n';
  std::cout << "PASS: actual dt, stall catch-up, bounded adaptive load, recovery, pause and manual step\n";
}
