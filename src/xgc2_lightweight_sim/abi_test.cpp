#include <xgc_rt.h>
#include <xgc-robotics-interfaces/robotics_interfaces_v1.h>
#include <xgc-lightweight-sim/simulation_records_v1.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

extern "C" const xgc_plugin_descriptor *xgc_rt_plugin_v1();

struct Sample {
  int64_t received;
  std::vector<uint8_t> data;
};
struct Boundary {
  const xgc_plugin_descriptor *plugin{xgc_rt_plugin_v1()};
  std::array<std::deque<Sample>, XGC_RT_MAX_PORTS> inputs;
  std::map<uint32_t, std::vector<uint8_t>> outputs;
  // Every publication in order: (port, bytes).
  std::vector<std::pair<uint32_t, std::vector<uint8_t>>> published;
  std::vector<uint8_t> popped;
  std::vector<std::string> errors, warnings;
  xgc_host_api api{};
  void *instance;
  static constexpr int64_t epoch = 1000000000;

  explicit Boundary(const char *model = "fs150", const std::string &extra = "",
                    bool start_provider = true) {
    api.abi_version = XGC_RT_ABI_VERSION;
    api.abi_minor = XGC_RT_ABI_MINOR;
    api.host = this;
    api.next = [](void *p, uint32_t port, xgc_sample_view *out) {
      auto &self = *static_cast<Boundary *>(p);
      auto &queue = self.inputs.at(port);
      if (queue.empty())
        return XGC_ERR_AGAIN;
      const auto received = queue.front().received;
      self.popped = std::move(queue.front().data);
      queue.pop_front();
      *out = {};
      out->data = self.popped.data();
      out->len = self.popped.size();
      out->t_rx = received;
      return XGC_OK;
    };
    api.publish = [](void *p, uint32_t port, uint64_t, const uint8_t *data,
                     uint32_t size) {
      auto &self = *static_cast<Boundary *>(p);
      self.outputs[port] = {data, data + size};
      self.published.emplace_back(port, std::vector<uint8_t>(data, data + size));
      return XGC_OK;
    };
    api.log = [](void *p, xgc_log_level level, const char *text) {
      auto &self = *static_cast<Boundary *>(p);
      (level >= XGC_LOG_ERROR ? self.errors : self.warnings).emplace_back(text);
    };
    instance = plugin->vtbl->create(&api);
    assert(instance);
    const std::string config = std::string("model = \"") + model +
                               "\"\nepoch_ns = 1000000000\noutput_ms = 1\n" +
                               extra;
    assert(plugin->vtbl->configure(instance, config.c_str()) == XGC_OK);
    assert(plugin->vtbl->activate(instance) == XGC_OK);
    if (std::string(model) == "fs150" && start_provider) {
      tick(epoch);
      xgc_fcu_extended_state_v1 fleet{};
      std::memcpy(&fleet, outputs.at(61).data(), sizeof fleet);
      for (uint32_t robot = 0; robot != fleet.count; ++robot) {
        xgc_sim_provider_request_v1 request{};
        request.stamp = double(epoch) * 1e-9;
        request.request_id = robot + 1;
        request.robot_index = robot;
        request.action = 1;
        send(62, request, epoch);
      }
      tick(epoch);
      for (const auto &sample : published)
        if (sample.first == 63) {
          xgc_sim_provider_result_v1 result{};
          std::memcpy(&result, sample.second.data(), sizeof result);
          assert(result.accepted && result.enabled && result.generation == 1);
        }
    }
  }
  ~Boundary() {
    plugin->vtbl->deactivate(instance);
    plugin->vtbl->destroy(instance);
  }
  template <class T>
  void send(uint32_t port, const T &value, int64_t received) {
    auto *bytes = reinterpret_cast<const uint8_t *>(&value);
    inputs.at(port).push_back({received, {bytes, bytes + sizeof value}});
  }
  void tick(int64_t now, int64_t round_start = -1) {
    xgc_step_ctx context{};
    context.now = now;
    context.round_start = round_start < 0 ? now : round_start;
    context.round_advanced = 1;
    context.round = (now - epoch) / 1000000;
    assert(plugin->vtbl->step(instance, &context) == XGC_OK);
    assert(errors.empty());
  }
  void arm(bool value, int64_t received = epoch) {
    xgc_fcu_request_v2 request{};
    request.stamp = 1;
    request.kind = 1;
    request.arm = value;
    send(2, request, received);
  }
  void mode(const char *name, int64_t received = epoch) {
    xgc_fcu_request_v2 request{};
    request.stamp = 1;
    request.kind = 2;
    std::strcpy(request.mode, name);
    send(2, request, received);
  }
  void enable_flight() {
    arm(true);
    mode("OFFBOARD");
  }
  xgc_fcu_state_v1 fcu_state() const {
    xgc_fcu_state_v1 value;
    const auto &bytes = outputs.at(6);
    assert(bytes.size() == sizeof value);
    std::memcpy(&value, bytes.data(), sizeof value);
    return value;
  }
  xgc_pose_v1 pose() const {
    xgc_pose_v1 value;
    const auto &bytes = outputs.at(3);
    assert(bytes.size() == sizeof value);
    std::memcpy(&value, bytes.data(), sizeof value);
    return value;
  }
  xgc_twist_v1 velocity() const {
    xgc_twist_v1 value;
    const auto &bytes = outputs.at(4);
    assert(bytes.size() == sizeof value);
    std::memcpy(&value, bytes.data(), sizeof value);
    return value;
  }
};

xgc_position_target_v1 acceleration(double at, double value) {
  xgc_position_target_v1 result{};
  result.stamp = at;
  result.coordinate_frame = 1;
  result.type_mask = 3135;
  result.acceleration[0] = value;
  return result;
}

// The controller's Takeoff/Hover position-only mask.
xgc_position_target_v1 position(double at, double z) {
  xgc_position_target_v1 result{};
  result.stamp = at;
  result.coordinate_frame = 1;
  result.type_mask = 0b111111111000;
  result.position[2] = z;
  return result;
}

bool has_warning(const Boundary &boundary, const char *text) {
  for (const auto &warning : boundary.warnings)
    if (warning.find(text) != std::string::npos)
      return true;
  return false;
}

// Stream a position setpoint every 10 ms (received at its stamp) while
// ticking every millisecond from `from` (exclusive) to `to` (inclusive).
void hover(Boundary &boundary, int64_t from, int64_t to, double z) {
  for (int64_t now = from + 1000000; now <= to; now += 1000000) {
    if ((now - Boundary::epoch) % 10000000 == 0)
      boundary.send(0, position(double(now) * 1e-9, z), now);
    boundary.tick(now);
  }
}

void fcu_semantics() {
  constexpr int64_t ms = 1000000, epoch = Boundary::epoch;
  // Only modelled modes are accepted; the others leave the mode unchanged.
  Boundary modes;
  modes.tick(epoch);
  assert(std::string(modes.fcu_state().mode) == "POSCTL");
  modes.mode("ALTCTL", epoch + ms);
  modes.tick(epoch + 2 * ms);
  assert(std::string(modes.fcu_state().mode) == "ALTCTL");
  for (const char *unsupported : {"STABILIZED", "AUTO.RTL", "AUTO.MISSION",
                                  "MANUAL", "AUTO.TAKEOFF", ""})
    modes.mode(unsupported, epoch + 3 * ms);
  modes.tick(epoch + 4 * ms);
  assert(std::string(modes.fcu_state().mode) == "ALTCTL");
  assert(has_warning(modes, "STABILIZED is not modelled; ALTCTL kept"));
  // OFFBOARD without a setpoint stream is refused.
  modes.mode("OFFBOARD", epoch + 5 * ms);
  modes.tick(epoch + 6 * ms);
  assert(std::string(modes.fcu_state().mode) == "ALTCTL");
  assert(has_warning(modes, "OFFBOARD refused without a fresh setpoint"));

  // Takeoff to 2 m in OFFBOARD; an in-air disarm is refused; AUTO.LAND
  // descends and disarms on touchdown (the old plant hovered, armed).
  Boundary land;
  land.send(0, position(1.0, 2.0), epoch);
  land.enable_flight();
  land.tick(epoch);
  hover(land, epoch, epoch + 10000 * ms, 2.0);
  assert(std::abs(land.pose().position[2] - 2.0) < 0.02);
  assert(land.fcu_state().armed && std::string(land.fcu_state().mode) == "OFFBOARD");
  land.arm(false, epoch + 10000 * ms);
  land.tick(epoch + 10001 * ms);
  assert(land.fcu_state().armed);
  assert(has_warning(land, "disarm refused while airborne"));
  land.mode("AUTO.LAND", epoch + 10001 * ms);
  int64_t now = epoch + 10001 * ms;
  while (land.fcu_state().armed && now < epoch + 20000 * ms) {
    now += ms;
    land.tick(now);
  }
  assert(!land.fcu_state().armed);
  assert(std::string(land.fcu_state().mode) == "AUTO.LAND");
  assert(std::abs(land.pose().position[2]) < 1e-10 && std::abs(land.velocity().linear[2]) < 1e-10);
  // About 2 m at 0.7 m/s after the 0.3 s velocity response.
  assert(now - epoch - 10001 * ms > 2000 * ms && now - epoch - 10001 * ms < 6000 * ms);

  // A stopped stream uses the fixed COM_OF_LOSS_T default (1 s), then the
  // plant brakes instead of integrating the last acceleration forever.
  Boundary silent;
  silent.send(0, acceleration(1.0, 0.5), epoch);
  silent.enable_flight();
  for (int i = 0; i <= 999; ++i)
    silent.tick(epoch + i * ms);
  assert(std::string(silent.fcu_state().mode) == "OFFBOARD");
  silent.tick(epoch + 1002 * ms);
  assert(std::string(silent.fcu_state().mode) == "AUTO.LOITER");
  assert(has_warning(silent, "holding in AUTO.LOITER"));
  silent.tick(epoch + 60000 * ms);
  assert(std::abs(silent.velocity().linear[0]) < 0.02);
  assert(silent.pose().position[0] < 1.0);
  // A configured timeout is honoured.
  Boundary patient("fs150", "[fcu_parameters]\nCOM_OF_LOSS_T = 2\n");
  patient.send(0, acceleration(1.0, 0.5), epoch);
  patient.enable_flight();
  patient.tick(epoch + 1999 * ms);
  assert(std::string(patient.fcu_state().mode) == "OFFBOARD");
  patient.tick(epoch + 2002 * ms);
  assert(std::string(patient.fcu_state().mode) == "AUTO.LOITER");
}

void bounded_future_controls() {
  constexpr int64_t ms = 1000000, epoch = Boundary::epoch;
  // A stamp further ahead of its receipt than max_future_ms is a clock-domain
  // error: it is dropped and reported, never queued for later.
  Boundary future("fs150", "max_future_ms = 100\n");
  future.send(0, acceleration(1.0, 0.0), epoch);
  future.enable_flight();
  future.send(0, acceleration(1.1005, 3.0), epoch + ms); // 99.5 ms ahead
  future.send(0, acceleration(3.0, 5.0), epoch + ms);    // 2 s ahead
  future.tick(epoch + 150 * ms);
  assert(has_warning(future, "dropped 1 control(s) stamped beyond max_future_ms"));
  // The in-window command applies at the first grid point >=100.5 ms.
  // Compare with a command actually received on that boundary; this remains
  // a causality test when the actuator is a finite-bandwidth rigid body.
  Boundary on_time;
  on_time.send(0, acceleration(1.0, 0.0), epoch);
  on_time.enable_flight();
  on_time.send(0, acceleration(1.101, 3.0), epoch + 101 * ms);
  on_time.tick(epoch + 150 * ms);
  assert(future.outputs == on_time.outputs);
  future.tick(epoch + 400 * ms);
  on_time.tick(epoch + 400 * ms);
  assert(future.outputs == on_time.outputs);

  // At most max_pending future controls wait. Additional commands cannot
  // affect the path: the accepted four must match the same bounded stream.
  Boundary full("fs150", "max_pending = 4\n"), accepted;
  for (auto *boundary : {&full, &accepted}) {
    boundary->send(0, acceleration(1.0, 0.0), epoch);
    boundary->enable_flight();
    boundary->tick(epoch);
    boundary->tick(epoch + ms);
  }
  for (int i = 0; i != 10; ++i) {
    const auto command = acceleration(1.0 + 0.01 * (i + 1), double(i));
    full.send(0, command, epoch + ms);
    if (i < 4) accepted.send(0, command, epoch + ms);
  }
  full.tick(epoch + 2 * ms);
  accepted.tick(epoch + 2 * ms);
  assert(has_warning(full, "dropped 1 control(s): max_pending"));
  assert(has_warning(full, "dropped 4 control(s): max_pending"));
  assert(!has_warning(full, "dropped 3 control(s): max_pending"));
  full.tick(epoch + 60 * ms);
  accepted.tick(epoch + 60 * ms);
  assert(full.outputs == accepted.outputs);

}

constexpr uint32_t kBlock = 10; // ports per robot; six blocks fit the ABI

// Robot r's publications in order, as (port within its block, bytes).
std::vector<std::pair<uint32_t, std::vector<uint8_t>>>
robot_outputs(const Boundary &boundary, uint32_t robot) {
  std::vector<std::pair<uint32_t, std::vector<uint8_t>>> result;
  for (const auto &item : boundary.published)
    if (item.first / kBlock == robot)
      result.emplace_back(item.first % kBlock, item.second);
  return result;
}

double stamp_of(const std::vector<uint8_t> &bytes) {
  double stamp;
  std::memcpy(&stamp, bytes.data(), sizeof stamp);
  return stamp;
}

bool configures(const std::string &config) {
  const auto *plugin = xgc_rt_plugin_v1();
  xgc_host_api api{};
  api.abi_version = XGC_RT_ABI_VERSION;
  api.abi_minor = XGC_RT_ABI_MINOR;
  api.log = [](void *, xgc_log_level, const char *) {};
  void *instance = plugin->vtbl->create(&api);
  assert(instance);
  const bool ok = plugin->vtbl->configure(instance, config.c_str()) == XGC_OK;
  plugin->vtbl->destroy(instance);
  return ok;
}

void rate_commands_and_target_feedback() {
  constexpr int64_t ms = 1000000, epoch = Boundary::epoch;
  Boundary model;
  model.send(0, position(1.0, 1.0), epoch);
  model.enable_flight();
  for (int step = 0; step <= 4000; ++step) {
    const auto now = epoch + step * ms;
    if (step % 10 == 0) model.send(0, position(double(now)*1e-9, 1.0), now);
    model.tick(now);
  }
  assert(model.pose().position[2] > 0.9);
  xgc_attitude_target_v2 command{};
  command.type_mask = 128; // q is ignored; body rates and thrust are active
  for (auto &q : command.q_wxyz) q = std::numeric_limits<double>::quiet_NaN();
  command.body_rate[2] = 0.5;
  command.thrust = 0.27726948;
  for (int step = 4001; step <= 5000; ++step) {
    const auto now = epoch + step * ms;
    if (step == 4001 || step % 10 == 0) {
      command.stamp = double(now)*1e-9;
      model.send(8, command, now);
    }
    model.tick(now);
  }
  const auto pose = model.pose();
  assert(2*std::atan2(pose.q_wxyz[3], pose.q_wxyz[0]) > 0.25);
  xgc_attitude_target_v2 target{};
  std::memcpy(&target, model.outputs.at(9).data(), sizeof target);
  assert(target.stamp == pose.stamp && target.type_mask == 128);
  assert(std::isnan(target.q_wxyz[0]));
  assert(std::abs(target.thrust-0.27726948) < 1e-12);
  assert(target.body_rate[2] == 0.5);
  // An unsupported mask does not extend OFFBOARD command freshness.
  command.stamp = 6.1;
  command.type_mask = 192;
  model.send(8, command, epoch + 5100*ms);
  model.tick(epoch + 5101*ms); // command affects the interval after its boundary
  assert(has_warning(model, "refused invalid or unsupported attitude mask"));
  model.tick(epoch + 6002*ms);
  assert(std::string(model.fcu_state().mode) == "AUTO.LOITER");
  std::memcpy(&target, model.outputs.at(9).data(), sizeof target);
  assert(target.type_mask == 7 && std::isfinite(target.body_rate[2]));
  // A new PVA stream replaces the old rate command rather than combining it.
  model.send(0, position(7.003, 1.0), epoch + 6003*ms);
  model.mode("OFFBOARD", epoch + 6003*ms);
  for (int step = 6003; step <= 12000; ++step) {
    const auto now = epoch + step*ms;
    if (step % 10 == 0) model.send(0, position(double(now)*1e-9, 1.0), now);
    model.tick(now);
  }
  std::memcpy(&target, model.outputs.at(9).data(), sizeof target);
  assert(target.type_mask == 7 && std::isfinite(target.body_rate[2]));
  assert(std::abs(model.velocity().angular[2]) < 0.01);
  model.mode("AUTO.LAND", epoch + 12001*ms);
  int step = 12001;
  do { model.tick(epoch + step++*ms); }
  while (model.fcu_state().armed && step < 25000);
  assert(!model.fcu_state().armed && model.pose().position[2] < 1e-8);
}

void centralized_round_target_is_shared() {
  Boundary early("fs150", "advance_on_round = true\n");
  Boundary late("fs150", "advance_on_round = true\n");
  for (auto *model : {&early, &late}) {
    model->send(0, position(1.0, 1.0), Boundary::epoch);
    model->enable_flight();
  }
  for (int k = 0; k < 40; ++k) {
    const auto round_start = Boundary::epoch + k*10000000LL;
    early.tick(round_start+2000000, round_start);
    late.tick(round_start+7000000, round_start);
    assert(early.outputs == late.outputs);
    assert(early.pose().stamp == double(round_start)*1e-9);
  }
}

void correlated_fcu_results() {
  Boundary fleet("fs150", "robots = 2\ninitial_poses = [0,0,0,0,1,0,0,0]\n");
  xgc_fcu_request_v2 request{};
  request.stamp = 1.0;
  request.request_id = 91;
  request.kind = 1;
  request.arm = 1;
  // Equal IDs in different robot lanes remain distinct executed requests.
  fleet.send(2, request, Boundary::epoch);
  fleet.send(12, request, Boundary::epoch);
  fleet.tick(Boundary::epoch + 1000000);
  unsigned results = 0;
  for (const auto &sample : fleet.published) {
    if (sample.first != 60) continue;
    xgc_fcu_result_v1 result{};
    assert(sample.second.size() == sizeof result);
    std::memcpy(&result, sample.second.data(), sizeof result);
    assert(result.request_id == 91 && result.request_stamp == 1.0);
    assert(result.kind == 1 && result.result == 0 && result.robot_index == results);
    ++results;
  }
  assert(results == 2);
  xgc_fcu_extended_state_v1 state{};
  std::memcpy(&state, fleet.outputs.at(61).data(), sizeof state);
  assert(state.count == 2 && state.landed_state[0] == 1 && state.landed_state[1] == 1);
  assert(state.vtol_state[0] == 0 && state.vtol_state[1] == 0);
  // A force flag is explicitly unsupported; it cannot become a normal
  // accepted ground disarm merely because the service transport succeeded.
  request.request_id = 92;
  request.arm = 0;
  request.flags = 1;
  fleet.send(2, request, Boundary::epoch + 1000000);
  fleet.tick(Boundary::epoch + 2000000);
  xgc_fcu_result_v1 result{};
  std::memcpy(&result, fleet.outputs.at(60).data(), sizeof result);
  assert(result.request_id == 92 && result.robot_index == 0 && result.result == 3);
  assert(fleet.fcu_state().armed && fleet.fcu_state().system_status == 4);
  request.flags = 0;
  request.kind = 2;
  request.request_id = 93;
  std::memset(request.mode, 'X', sizeof request.mode);
  fleet.send(2, request, Boundary::epoch + 2000000);
  fleet.tick(Boundary::epoch + 3000000);
  std::memcpy(&result, fleet.outputs.at(60).data(), sizeof result);
  assert(result.request_id == 93 && result.result == 3);
  assert(fleet.outputs.at(13).size() == sizeof(xgc_pose_v1));
}

// Robot r owns block r; block 0 keeps the single-robot names and indices.
void port_blocks() {
  const auto *plugin = xgc_rt_plugin_v1();
  assert(plugin->port_count == 64 && plugin->port_count <= XGC_RT_MAX_PORTS);
  const char *names[kBlock] = {"setpoint", "cmd_vel",   "fcu_request",
                               "pose",     "velocity",  "imu",
                               "fcu_state", "paired_state", "attitude_command", "attitude_target"};
  for (uint32_t kind = 0; kind != kBlock; ++kind) {
    const auto &first = plugin->ports[kind];
    const auto &last = plugin->ports[5 * kBlock + kind];
    assert(std::string(first.name) == names[kind]);
    assert(std::string(last.name) == std::string(names[kind]) + "_5");
    assert(std::string(last.schema_id) == first.schema_id);
    assert(last.qos == first.qos);
  }
  assert(plugin->ports[3].dir == XGC_PORT_OUT);
  assert(plugin->ports[kBlock + 3].dir == XGC_PORT_OUT_OPTIONAL);
  const std::string base = "model = \"fs150\"\nepoch_ns = 1000000000\n";
  assert(configures(base + "robots = 6\n"));
  assert(std::string(plugin->ports[62].name) == "provider_request");
  assert(std::string(plugin->ports[63].name) == "provider_result");
  assert(plugin->ports[62].dir == XGC_PORT_IN_OPTIONAL);
  assert(std::string(plugin->ports[62].schema_id) == "xgc.sim_provider_request/1");
  assert(std::string(plugin->ports[63].schema_id) == "xgc.sim_provider_result/1");
  assert(!configures(base + "offboard_timeout_ms = 500\n"));
  assert(!configures(base + "hover_thrust_ratio = .5\n"));
  assert(!configures(base + "world_ground_z = nan\n"));
  assert(!configures(base + "robots = 7\n"));
  assert(!configures(base + "robots = 12\n"));
  assert(configures(base + "robots = 1\n"));
  assert(!configures(base + "robots = 0\n"));
  assert(!configures(base + "robots = 2\ninitial_pose = [0, 0, 0, 0]\n"));
  assert(!configures(base + "robots = 2\ninitial_poses = [0, 0, 0, 0]\n"));
  assert(!configures(base + "initial_pose = [0, 0, 0, 0]\n"
                            "initial_poses = [0, 0, 0, 0]\n"));
  assert(configures(base + "robots = 2\n"
                           "initial_poses = [0, 0, 0, 0, 1, 1, 0, 0]\n"));
}

// A batch fed the controls a set of independent single-robot instances get,
// with the same receive times, publishes byte-identical states and stamps.
// A batch woken on another schedule matches them at every common stamp.
void batch_matches_independent_robots(const char *model, uint32_t count) {
  constexpr int64_t ms = 1000000, epoch = Boundary::epoch;
  const uint32_t n = count;
  const bool flight = std::string(model) == "fs150";
  std::vector<std::string> pose;
  std::string poses;
  for (uint32_t r = 0; r != n; ++r) {
    pose.push_back(std::to_string(0.5 * r) + ", " + std::to_string(-0.25 * r) +
                   ", " + std::to_string(0.1 * r) + ", " +
                   std::to_string(0.3 * r));
    poses += (r ? ", " : "") + pose.back();
  }
  const std::string batch_config =
      "robots = " + std::to_string(n) + "\ninitial_poses = [" + poses + "]\n";
  Boundary batch(model, batch_config), sparse(model, batch_config);
  std::vector<std::unique_ptr<Boundary>> singles;
  for (uint32_t r = 0; r != n; ++r)
    singles.push_back(std::make_unique<Boundary>(
        model, "initial_pose = [" + pose[r] + "]\n"));

  struct Event {
    int64_t received;
    uint32_t robot, port;
    std::vector<uint8_t> bytes;
  };
  std::vector<Event> events;
  auto add = [&](int64_t received, uint32_t robot, uint32_t port,
                 const auto &value) {
    auto *bytes = reinterpret_cast<const uint8_t *>(&value);
    events.push_back({received, robot, port, {bytes, bytes + sizeof value}});
  };
  auto request = [&](int64_t received, uint32_t robot, uint32_t kind,
                     const char *mode, uint32_t arm) {
    xgc_fcu_request_v2 value{};
    value.stamp = double(received) * 1e-9;
    value.kind = kind;
    value.arm = arm;
    std::strcpy(value.mode, mode);
    add(received, robot, 2, value);
  };
  for (uint32_t r = 0; r != n; ++r) {
    for (int64_t t = int64_t(r) * ms; t < 1400 * ms; t += 20 * ms) {
      const int64_t received = epoch + t;
      const double phase = 1e-9 * double(t) + r;
      if (flight) {
        if (r == 1 && t > 300 * ms)
          break; // the stream stops: offboard timeout, AUTO.LOITER
        auto value = acceleration(double(received) * 1e-9, 0.3 * std::sin(phase));
        value.acceleration[1] = 0.1 * r;
        value.acceleration[2] = 0.4 * std::cos(phase);
        value.type_mask = 3135 & ~2048; // with a yaw rate
        value.yaw_rate = 0.2 * r;
        if (r == 3)
          value.stamp -= 0.015; // late: effective at receipt
        if (r == 4 && t == 404 * ms)
          value.stamp += 0.030; // future: effective 30 ms after receipt
        add(received, r, 0, value);
        if (t == int64_t(r) * ms) {
          request(received, r, 1, "", 1);
          request(received, r, 2, "OFFBOARD", 0);
        }
      } else {
        xgc_twist_v1 value{};
        value.stamp = double(received) * 1e-9;
        value.linear[0] = 0.5 + 0.1 * r;
        value.linear[1] = 0.2 * std::sin(phase);
        value.angular[2] = 0.3 * std::cos(phase);
        add(received, r, 1, value);
      }
    }
  }
  if (flight && n >= 4) {
    request(epoch + 600 * ms, 2, 2, "AUTO.LAND", 0);
    request(epoch + 1000 * ms, 0, 1, "", 0); // in-air disarm: refused
    request(epoch + 1100 * ms, 3, 2, "STABILIZED", 0);
  }
  std::stable_sort(events.begin(), events.end(),
                   [](const Event &a, const Event &b) {
                     return a.received < b.received;
                   });

  // Irregular reference wakes that include every 50 ms point; sparse wakes
  // every 50 ms only.
  std::vector<int64_t> wakes;
  for (int64_t t = epoch, k = 0; t < epoch + 1500 * ms; ++k)
    wakes.push_back(t += (1 + (k * 7919) % 7) * ms);
  for (int64_t t = epoch; t <= epoch + 1500 * ms; t += 50 * ms)
    wakes.push_back(t);
  std::sort(wakes.begin(), wakes.end());
  wakes.erase(std::unique(wakes.begin(), wakes.end()), wakes.end());

  auto run = [&](const std::vector<int64_t> &schedule, auto &&deliver,
                 auto &&tick) {
    size_t next = 0;
    for (int64_t now : schedule) {
      for (; next != events.size() && events[next].received <= now; ++next)
        deliver(events[next]);
      tick(now);
    }
  };
  run(
      wakes,
      [&](const Event &e) {
        batch.inputs[e.robot * kBlock + e.port].push_back(
            {e.received, e.bytes});
        singles[e.robot]->inputs[e.port].push_back({e.received, e.bytes});
      },
      [&](int64_t now) {
        batch.tick(now);
        for (auto &single : singles)
          single->tick(now);
      });
  std::vector<int64_t> every_50;
  for (int64_t t = epoch; t <= epoch + 1500 * ms; t += 50 * ms)
    every_50.push_back(t);
  run(
      every_50,
      [&](const Event &e) {
        sparse.inputs[e.robot * kBlock + e.port].push_back(
            {e.received, e.bytes});
      },
      [&](int64_t now) { sparse.tick(now); });

  size_t single_warnings = 0;
  for (uint32_t r = 0; r != n; ++r) {
    const auto independent = robot_outputs(*singles[r], 0);
    assert(independent.size() > 1000);
    assert(robot_outputs(batch, r) == independent);
    const auto late = robot_outputs(sparse, r);
    assert(late.size() == 31 * (flight ? 6 : 3));
    for (const auto &[kind, bytes] : late) {
      bool found = false;
      for (const auto &item : independent)
        if (item.first == kind && stamp_of(item.second) == stamp_of(bytes)) {
          assert(item.second == bytes);
          found = true;
        }
      assert(found);
    }
    single_warnings += singles[r]->warnings.size();
  }
  assert(batch.warnings.size() == single_warnings);
  if (flight && n >= 4) {
    // The scenario exercised the FCU paths it names.
    const auto state = [&](uint32_t r) {
      xgc_fcu_state_v1 value;
      std::memcpy(&value, singles[r]->outputs.at(6).data(), sizeof value);
      return value;
    };
    assert(std::string(state(1).mode) == "AUTO.LOITER");
    assert(std::string(state(2).mode) == "AUTO.LAND");
    assert(state(0).armed && std::string(state(3).mode) == "OFFBOARD");
    assert(has_warning(batch, "robot 1: no setpoint for offboard_timeout_ms"));
    assert(has_warning(batch, "robot 0: disarm refused while airborne"));
    assert(has_warning(batch, "robot 3: FCU mode STABILIZED is not modelled"));
  }
}


void full_attitude_and_specific_force() {
  constexpr int64_t ms = 1000000;
  Boundary model;
  model.send(0, position(1.0, 2.0), Boundary::epoch);
  model.enable_flight();
  struct Observed {
    xgc_pose_v1 pose;
    xgc_twist_v1 velocity;
    xgc_imu_v1 imu;
  };
  std::vector<Observed> samples;
  double max_tilt = 0.0, max_body_rate = 0.0;
  for (int step = 0; step <= 6000; ++step) {
    const auto now = Boundary::epoch + step * ms;
    auto command = position(double(now) * 1e-9, 2.0);
    if (step >= 2500) {
      command.position[0] = 1.0;
      command.position[1] = -0.5;
      command.type_mask &= ~uint16_t(1024);
      command.yaw = 0.6;
    }
    model.send(0, command, now);
    model.tick(now);
    Observed s{model.pose(), model.velocity(), {}};
    std::memcpy(&s.imu, model.outputs.at(5).data(), sizeof s.imu);
    assert(s.imu.stamp == s.pose.stamp && s.pose.stamp == s.velocity.stamp);
    xgc_paired_state_v1 paired{};
    std::memcpy(&paired, model.outputs.at(7).data(), sizeof paired);
    assert(paired.pose_stamp_sec == s.pose.stamp);
    assert(paired.twist_stamp_sec == s.velocity.stamp);
    const auto *q = s.pose.q_wxyz;
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double r[3][3] = {
        {1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)},
        {2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)},
        {2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)}};
    assert(std::abs(w*w+x*x+y*y+z*z-1.0) < 1e-12);
    max_tilt = std::max(max_tilt, std::hypot(x, y));
    for (int i = 0; i < 3; ++i) {
      assert(paired.position[i] == s.pose.position[i]);
      assert(paired.linear_velocity[i] == s.velocity.linear[i]);
      assert(paired.orientation_xyzw[i] == q[i+1]);
      double world_omega = 0.0;
      for (int j = 0; j < 3; ++j) world_omega += r[i][j] * s.imu.gyro[j];
      assert(std::abs(world_omega - s.velocity.angular[i]) < 1e-12);
      max_body_rate = std::max(max_body_rate, std::abs(s.imu.gyro[i]));
    }
    assert(paired.orientation_xyzw[3] == q[0]);
    if (step >= 1500) samples.push_back(s); // airborne, no ground impulse
  }
  assert(max_tilt > 0.01 && max_body_rate > 0.05);
  double max_acceleration_error = 0.0, max_gyro_error = 0.0;
  for (size_t k = 1; k+1 < samples.size(); ++k) {
    const auto &before = samples[k-1], &s = samples[k], &after = samples[k+1];
    // Commands are discontinuous at 2.5 s: omit that immediate derivative
    // boundary, not the following transient response.
    if (std::abs(s.pose.stamp - 3.5) < 0.002) continue;
    const auto *q=s.pose.q_wxyz;
    const double w=q[0], x=q[1], y=q[2], z=q[3];
    const double r[3][3] = {
        {1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)},
        {2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)},
        {2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)}};
    const double dt=after.pose.stamp-before.pose.stamp;
    double qdot[4];
    for (int j=0;j<4;++j) qdot[j]=(after.pose.q_wxyz[j]-before.pose.q_wxyz[j])/dt;
    const double gyro_from_q[3]={
        2*(-x*qdot[0]+w*qdot[1]+z*qdot[2]-y*qdot[3]),
        2*(-y*qdot[0]-z*qdot[1]+w*qdot[2]+x*qdot[3]),
        2*(-z*qdot[0]+y*qdot[1]-x*qdot[2]+w*qdot[3])};
    for (int i=0;i<3;++i) {
      double from_imu=(i==2 ? -9.8066 : 0.0);
      for (int j=0;j<3;++j) from_imu+=r[i][j]*s.imu.accel[j];
      const double from_velocity=(after.velocity.linear[i]-before.velocity.linear[i])/dt;
      max_acceleration_error=std::max(max_acceleration_error,std::abs(from_imu-from_velocity));
      max_gyro_error=std::max(max_gyro_error,std::abs(s.imu.gyro[i]-gyro_from_q[i]));
    }
  }
  std::cout << "wire feedback derivative errors: accel=" << max_acceleration_error
            << " gyro=" << max_gyro_error << "\n";
  assert(max_acceleration_error < 0.03);
  assert(max_gyro_error < 0.002);
}

void provider_and_world_ground() {
  constexpr int64_t ms = 1000000, epoch = Boundary::epoch;
  Boundary b("fs150", "robots=2\ninitial_poses=[0,0,1.4,0,2,0,1.4,0]\nworld_ground_z=.4\n", false);
  b.tick(epoch);
  assert(!b.fcu_state().connected && !b.fcu_state().armed);
  auto provider = [&](uint32_t slot, uint32_t action, uint64_t gen, int64_t now) {
    xgc_sim_provider_request_v1 r{};
    r.stamp = double(now)*1e-9; r.request_id = uint64_t(now); r.robot_index=slot;
    r.action=action; r.generation=gen; b.send(62,r,now); b.tick(now);
    xgc_sim_provider_result_v1 result{};
    std::memcpy(&result,b.outputs.at(63).data(),sizeof result);
    assert(result.request_id==r.request_id && result.robot_index==slot);
    return result;
  };
  auto result=provider(0,0,0,epoch+ms);
  assert(result.accepted && !result.enabled && result.generation==0);
  result=provider(0,1,0,epoch+2*ms);
  assert(result.accepted && result.enabled && result.generation==1);
  assert(b.fcu_state().connected);
  xgc_fcu_state_v1 sibling{};
  std::memcpy(&sibling,b.outputs.at(16).data(),sizeof sibling);
  assert(!sibling.connected);
  b.tick(epoch+102*ms);
  const double falling=b.pose().position[2];
  assert(falling<1.39 && falling>.4);
  result=provider(0,1,0,epoch+103*ms);
  assert(result.accepted && result.generation==1 && b.pose().position[2]<1.39);
  auto future=position(1.9,10); b.send(0,future,epoch+104*ms); b.tick(epoch+104*ms);
  result=provider(0,2,1,epoch+105*ms);
  assert(result.accepted && !result.enabled);
  b.tick(epoch+200*ms);
  assert(b.pose().position[2]<falling); // stop does not reset physics/world time
  result=provider(0,1,1,epoch+201*ms);
  assert(result.accepted && result.enabled && result.generation==2);
  assert(b.pose().position[2]>1.3999 && b.pose().stamp==1.201);
  result=provider(0,2,1,epoch+202*ms);
  assert(!result.accepted && result.enabled && result.generation==2);
  b.tick(epoch+1200*ms);
  assert(std::abs(b.pose().position[2]-.4)<1e-10);
  assert(!b.fcu_state().armed); // old future control never crossed restart
}

void input_rejection_is_local() {
  constexpr int64_t ms=1000000, epoch=Boundary::epoch;
  const std::string config="robots=2\ninitial_poses=[0,0,0,0,2,0,0,0]\n";
  Boundary bad("fs150",config), reference("fs150",config);
  for(auto *b:{&bad,&reference}) {
    for(uint32_t slot=0;slot!=2;++slot) {
      b->send(slot*10,position(1.0,1.0),epoch);
      xgc_fcu_request_v2 arm{};arm.stamp=1;arm.kind=1;arm.arm=1;
      b->send(slot*10+2,arm,epoch);arm.kind=2;std::strcpy(arm.mode,"OFFBOARD");
      b->send(slot*10+2,arm,epoch);
    }
    b->tick(epoch+ms);
  }
  auto unsupported=position(1.002,9);unsupported.coordinate_frame=7;
  bad.send(0,unsupported,epoch+2*ms);
  auto invalid=position(std::numeric_limits<double>::quiet_NaN(),9);
  bad.send(0,invalid,epoch+2*ms);
  auto all_ignored=position(1.002,9);all_ignored.type_mask=4095;
  bad.send(0,all_ignored,epoch+2*ms);
  xgc_attitude_target_v2 raw{};raw.stamp=1.002;raw.type_mask=192;raw.thrust=1;
  bad.send(8,raw,epoch+2*ms);
  bad.inputs[0].push_back({epoch+2*ms,{1,2,3}});
  bad.tick(epoch+3*ms);reference.tick(epoch+3*ms);
  assert(bad.outputs==reference.outputs);
  assert(has_warning(bad,"rejected PVA receiver input"));
  assert(has_warning(bad,"invalid control timestamp"));
  assert(has_warning(bad,"malformed control payload"));
  assert(has_warning(bad,"unsupported attitude mask"));
  for(int k=4;k<=100;++k){
    for(auto *b:{&bad,&reference}) {
      if(k%10==0) for(uint32_t slot=0;slot!=2;++slot)
        b->send(slot*10,position(1.0+k*.001,1.0),epoch+k*ms);
      b->tick(epoch+k*ms);
    }
  }
  assert(bad.outputs==reference.outputs); // rejected robot0 ingress cannot poison sibling
}

void semantic_inputs_are_robot_local() {
  constexpr int64_t epoch=Boundary::epoch, ms=1000000;
  for(const char *kind:{"fs150","scout","mecanum"}) {
    const bool flight=std::string(kind)=="fs150";
    const std::string config="robots=2\ninitial_poses=[0,0,0,0,2,0,0,0]\n";
    Boundary b(kind,config), reference(kind,config);
    for(auto *boundary:{&b,&reference}) {
      boundary->tick(epoch);
      if(flight) {
        boundary->send(10,position(1.001,1),epoch+ms);
        xgc_fcu_request_v2 request{};request.stamp=1.001;request.kind=1;request.arm=1;
        boundary->send(12,request,epoch+ms);
        request.kind=2;std::strcpy(request.mode,"OFFBOARD");
        boundary->send(12,request,epoch+ms);
      } else {
        xgc_twist_v1 valid{};valid.stamp=1.001;valid.linear[0]=.5;
        boundary->send(11,valid,epoch+ms);
      }
    }
    if(flight) {
      xgc_twist_v1 wrong{};wrong.stamp=1.001;wrong.linear[0]=.5;
      b.send(1,wrong,epoch+ms);
    } else {
      xgc_twist_v1 invalid{};invalid.stamp=1.001;
      invalid.linear[0]=std::numeric_limits<double>::quiet_NaN();
      b.send(1,invalid,epoch+ms);
      b.send(0,position(1.001,1),epoch+ms);
    }
    b.tick(epoch+2*ms);reference.tick(epoch+2*ms);
    assert(!b.warnings.empty() && b.outputs==reference.outputs);
    for(int k=3;k<=250;++k) {
      for(auto *boundary:{&b,&reference}) {
        if(flight && k%10==0)
          boundary->send(10,position(1+k*.001,1),epoch+k*ms);
        boundary->tick(epoch+k*ms);
      }
      assert(b.outputs==reference.outputs);
    }
    xgc_pose_v1 sibling{};
    std::memcpy(&sibling,b.outputs.at(13).data(),sizeof sibling);
    assert(sibling.stamp==1.25);
    assert(flight ? sibling.position[2]>.01 : sibling.position[0]>2.01);
    std::cout << "PASS robot-local semantic refusal and sibling bytes: " << kind << "\n";
  }
}

int main(int argc, char **argv) {
  if(argc==2 && std::string(argv[1])=="--semantic-refusal") {
    semantic_inputs_are_robot_local();
    return 0;
  }
  assert(argc==1);
  semantic_inputs_are_robot_local();
  // Different wake schedules produce the same plant trajectory when the
  // actual arrivals are equal. Future controls cannot affect earlier steps.
  Boundary frequent, delayed;
  for (auto *boundary : {&frequent, &delayed}) {
    boundary->enable_flight();
    boundary->send(0, acceleration(1.0, 2.0), Boundary::epoch);
    boundary->send(0, acceleration(1.015, 4.0), Boundary::epoch + 10000000);
    boundary->tick(Boundary::epoch);
  }
  for (int i = 1; i <= 20; ++i)
    frequent.tick(Boundary::epoch + i * 1000000);
  delayed.tick(Boundary::epoch + 20000000);
  assert(std::abs(frequent.pose().position[0] - delayed.pose().position[0]) <
         1e-14);
  assert(frequent.outputs == delayed.outputs);

  // Old header but late reception: only the remaining interval receives it.
  Boundary late, received_on_time;
  for (auto *boundary : {&late, &received_on_time}) {
    boundary->send(0, acceleration(1.0, 0.0), Boundary::epoch);
    boundary->enable_flight();
  }
  late.send(0, acceleration(1.0, 2.0), Boundary::epoch + 10000000);
  received_on_time.send(0, acceleration(1.01, 2.0), Boundary::epoch + 10000000);
  late.tick(Boundary::epoch + 20000000);
  received_on_time.tick(Boundary::epoch + 20000000);
  assert(late.outputs == received_on_time.outputs);
  late.send(0, acceleration(1.0, 0.0), Boundary::epoch + 5000000);
  received_on_time.send(0, acceleration(1.02, 0.0), Boundary::epoch + 20000000);
  late.tick(Boundary::epoch + 30000000);
  received_on_time.tick(Boundary::epoch + 30000000);
  assert(late.outputs == received_on_time.outputs);

  // Drift-free command/step times over many exact-nanosecond boundaries.
  Boundary scout("scout"), mecanum("mecanum");
  for (int i = 0; i != 10000; ++i) {
    const auto now = Boundary::epoch + int64_t(i) * 1000000;
    xgc_twist_v1 command{};
    command.stamp = double(now) * 1e-9;
    command.linear[0] = 1.0;
    command.linear[1] = 0.5;
    for (auto *boundary : {&scout, &mecanum}) {
      boundary->send(1, command, now);
      boundary->tick(now);
    }
  }
  assert(scout.pose().position[1] == 0.0);
  assert(std::abs(mecanum.pose().position[1] - 4.999) < 0.002);
  assert(scout.outputs.count(6) == 0 && mecanum.outputs.count(6) == 0);
  provider_and_world_ground();
  input_rejection_is_local();
  full_attitude_and_specific_force();
  rate_commands_and_target_feedback();
  centralized_round_target_is_shared();
  correlated_fcu_results();
  fcu_semantics();
  bounded_future_controls();
  port_blocks();
  for (const char *model : {"fs150", "scout", "mecanum"})
    for (uint32_t count : {1u, 6u}) batch_matches_independent_robots(model, count);
  std::cout
      << "lightweight ABI arrival timing and independent time grid: passed\n";
}
