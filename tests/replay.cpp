// Same-input replay against the actual pre-xsim owning plugin, loaded
// privately. The reference source is extracted by validate.sh at the pinned
// commit; no old implementation is compiled into or installed with xsim.
#include "core/world.hpp"
#include "io/config.hpp"
#include <cassert>
#include <cstring>
#include <dlfcn.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <xgc-lightweight-sim/simulation_records_v1.h>
#include <xgc-robotics-interfaces/robotics_interfaces_v1.h>
#include <xgc_rt.h>
using namespace xsim;
struct Baseline {
  struct Input {
    int64_t at;
    std::vector<uint8_t> data;
  };
  void *library;
  const xgc_plugin_descriptor *plugin;
  void *instance;
  xgc_host_api api{};
  std::map<unsigned, std::deque<Input>> inputs;
  std::map<unsigned, std::vector<uint8_t>> outputs;
  std::vector<uint8_t> popped;
  std::vector<xgc_fcu_result_v1> results;
  std::string error;
  int64_t epoch;
  Baseline(const char *file, const std::string &kind, int64_t e) : epoch(e) {
    library = dlopen(file, RTLD_NOW | RTLD_LOCAL);
    if (!library)
      throw std::runtime_error(dlerror());
    plugin = reinterpret_cast<const xgc_plugin_descriptor *(*)()>(
        dlsym(library, "xgc_rt_plugin_v1"))();
    api.abi_version = XGC_RT_ABI_VERSION;
    api.abi_minor = XGC_RT_ABI_MINOR;
    api.host = this;
    api.next = [](void *p, uint32_t port, xgc_sample_view *v) {
      auto &s = *static_cast<Baseline *>(p);
      auto &q = s.inputs[port];
      if (q.empty())
        return XGC_ERR_AGAIN;
      auto at = q.front().at;
      s.popped = std::move(q.front().data);
      q.pop_front();
      *v = {};
      v->data = s.popped.data();
      v->len = s.popped.size();
      v->t_rx = at;
      return XGC_OK;
    };
    api.publish = [](void *p, uint32_t port, uint64_t, const uint8_t *b,
                     uint32_t n) {
      auto &self = *static_cast<Baseline *>(p);
      self.outputs[port] = {b, b + n};
      if (port == 60) {
        xgc_fcu_result_v1 r{};
        assert(n == sizeof r);
        std::memcpy(&r, b, n);
        self.results.push_back(r);
      }
      return XGC_OK;
    };
    api.log = [](void *p, xgc_log_level level, const char *text) {
      if (level >= XGC_LOG_ERROR)
        static_cast<Baseline *>(p)->error = text;
    };
    instance = plugin->vtbl->create(&api);
    assert(instance);
    auto cfg =
        "model = \"" + kind + "\"\nepoch_ns = " + std::to_string(e) +
        "\nstep_ms = 1\noutput_ms = 1\ninitial_pose = [2.0, -1.0, 0.0, 0.35]\n";
    assert(plugin->vtbl->configure(instance, cfg.c_str()) == XGC_OK);
    assert(plugin->vtbl->activate(instance) == XGC_OK);
    tick(e);
  }
  ~Baseline() {
    plugin->vtbl->deactivate(instance);
    plugin->vtbl->destroy(instance);
    dlclose(library);
  }
  template <class T> void send(unsigned p, const T &v, int64_t at) {
    auto b = reinterpret_cast<const uint8_t *>(&v);
    inputs[p].push_back({at, {b, b + sizeof v}});
  }
  template <class T> T get(unsigned p) {
    T v{};
    assert(outputs.at(p).size() == sizeof v);
    std::memcpy(&v, outputs.at(p).data(), sizeof v);
    return v;
  }
  void tick(int64_t at) {
    xgc_step_ctx c{};
    c.now = c.round_start = at;
    c.round = (at - epoch) / 1000000;
    c.round_advanced = 1;
    assert(plugin->vtbl->step(instance, &c) == XGC_OK);
    if (!error.empty())
      throw std::runtime_error(error);
  }
};
Ticket command(World &w, Op op, Key k) {
  auto t = std::make_shared<Command>();
  t->op = op;
  t->key = k;
  w.submit(t);
  return t;
}
int main(int argc, char **argv) {
  assert(argc == 4);
  const std::string kind = argv[2];
  const int64_t epoch = 1700000000000000000LL;
  Config cfg = parse_entity({{"name", "replay"},
                             {"kind", kind},
                             {"position", {2, -1, 0}},
                             {"yaw", .35}});
  World w(epoch, 1000000, 1000000);
  auto entity = std::make_shared<Entity>(cfg);
  auto add = std::make_shared<Command>();
  add->op = Op::Add;
  add->prepared =
      std::make_unique<Prepared>(Prepared{entity, prepare_model(cfg)});
  w.submit(add);
  w.boundary();
  assert(add->result.success);
  auto baseline = std::make_unique<Baseline>(argv[1], kind, epoch);
  auto provider = [&](int action, uint64_t generation, int64_t now) {
    auto t = std::make_shared<Command>();
    t->op = Op::Provider;
    t->key = {entity->id, generation};
    t->action = action;
    if (action == 1)
      t->prepared =
          std::make_unique<Prepared>(Prepared{entity, prepare_model(cfg)});
    w.submit(t);
    w.boundary();
    if (kind == "fs150") {
      xgc_sim_provider_request_v1 p{};
      p.stamp = double(now) * 1e-9;
      p.request_id = now;
      p.action = action;
      p.generation = generation;
      baseline->send(62, p, now);
      baseline->tick(now);
      auto r = baseline->get<xgc_sim_provider_result_v1>(63);
      assert(t->result.success == bool(r.accepted));
      assert(t->result.enabled == bool(r.enabled));
      assert(t->result.key.generation == r.generation);
    }
    return t;
  };
  provider(1, 0, epoch);
  Json events = Json::array(), checkpoints = Json::array();
  std::map<std::string, double> max_error;
  auto compare = [&](const std::string &field, double actual,
                     double reference) {
    double e = std::abs(actual - reference);
    max_error[field] = std::max(max_error[field], e);
    if (!std::isfinite(e) || e > 1e-10) {
      std::cerr << kind << " " << field << " " << std::setprecision(17)
                << actual << " != " << reference << " error=" << e << "\n";
      std::abort();
    }
  };
  for (int k = 0; k < 8000; ++k) {
    int64_t now = epoch + int64_t(k) * 1000000;
    if (kind == "fs150" && k == 6000)
      provider(2, 1, now);
    if (kind == "fs150" && k == 6500)
      provider(1, 1, now);
    Key key{entity->id, entity->generation};
    std::vector<std::pair<uint64_t, Ticket>> replies;
    if (kind == "fs150") {
      if (k % 20 == 0 && (k < 6000 || k >= 6500)) {
        xgc_position_target_v1 p{};
        p.stamp = double(now) * 1e-9;
        p.coordinate_frame = 1;
        p.type_mask = 8 | 16 | 32 | 64 | 128 | 256 | 2048;
        p.position[0] = k < 3000 ? 2.5 : 1.5;
        p.position[1] = -.5;
        p.position[2] = 1.0;
        p.yaw = .6;
        baseline->send(0, p, now);
        auto t = std::make_shared<Command>();
        t->op = Op::Pva;
        t->key = key;
        t->pva = p;
        w.submit(t);
      }
      if (k == 0 || k == 6500 || k == 4000 || k == 4500) {
        const bool arm = k == 0 || k == 6500;
        const bool mode = k == 4500;
        xgc_fcu_request_v2 p{};
        p.stamp = double(now) * 1e-9;
        p.request_id = k + 1;
        p.kind = mode ? 2 : 1;
        p.arm = arm;
        if (mode)
          std::strcpy(p.mode, "UNSUPPORTED");
        baseline->send(2, p, now);
        auto t = std::make_shared<Command>();
        t->op = mode ? Op::Mode : Op::Arm;
        t->key = key;
        t->arm = arm;
        t->mode = "UNSUPPORTED";
        w.submit(t);
        w.boundary();
        replies.emplace_back(p.request_id, t);
        // The baseline applies FCU requests at its next physical boundary.
        events.push_back({{"step", k},
                          {"operation", mode ? "unsupported_mode"
                                             : arm ? "arm" : "airborne_disarm"},
                          {"xsim_success", t->result.success},
                          {"xsim_reason", t->result.reason}});
      }
      if (k == 0 || k == 6500) {
        xgc_fcu_request_v2 p{};
        p.stamp = double(now) * 1e-9;
        p.request_id = k + 2;
        p.kind = 2;
        std::strcpy(p.mode, "OFFBOARD");
        baseline->send(2, p, now);
        auto t = std::make_shared<Command>();
        t->op = Op::Mode;
        t->key = key;
        t->mode = "OFFBOARD";
        w.submit(t);
        replies.emplace_back(p.request_id, t);
      }
    } else if (k % 1000 == 0) {
      static const double commands[][3] = {
          {1, 0, 0}, {1, 0, .5},     {0, 0, 0}, {-1, 0, 0},
          {0, 1, 0}, {.7, -.5, -.4}, {0, 0, 0}, {0, 0, 0}};
      const auto *u = commands[k / 1000];
      xgc_twist_v1 v{};
      v.stamp = double(now) * 1e-9;
      v.linear[0] = u[0];
      v.linear[1] = u[1];
      v.angular[2] = u[2];
      baseline->send(1, v, now);
      auto t = std::make_shared<Command>();
      t->op = Op::Velocity;
      t->key = key;
      t->velocity = {u[0], u[1], u[2]};
      w.submit(t);
    }
    w.boundary();
    w.advance();
    baseline->tick(now + 1000000);
    for (auto &reply : replies) {
      auto found = std::find_if(
          baseline->results.begin(), baseline->results.end(),
          [&](const auto &r) { return r.request_id == reply.first; });
      assert(found != baseline->results.end());
      const auto &result = reply.second->result;
      assert(result.applied &&
             found->result == (result.success ? 0 : result.reason));
      events.push_back({{"step", k},
                        {"request_id", reply.first},
                        {"baseline_result", found->result},
                        {"xsim_result", result.success ? 0 : result.reason}});
    }
    baseline->results.clear();
    const auto state = w.capture().states.at(0);
    const auto pose = baseline->get<xgc_pose_v1>(3);
    const auto vel = baseline->get<xgc_twist_v1>(4);
    compare("stamp", double(state.stamp) * 1e-9, pose.stamp);
    for (int a = 0; a < 3; ++a) {
      compare("position", state.position[a], pose.position[a]);
      compare("velocity", state.velocity[a], vel.linear[a]);
      compare("angular_velocity", (state.orientation * state.omega)[a],
              vel.angular[a]);
    }
    compare("quaternion", state.orientation.w(), pose.q_wxyz[0]);
    for (int a = 0; a < 3; ++a)
      compare("quaternion", state.orientation.coeffs()[a], pose.q_wxyz[a + 1]);
    if (kind == "fs150") {
      auto imu = baseline->get<xgc_imu_v1>(5);
      auto fcu = baseline->get<xgc_fcu_state_v1>(6);
      auto target = baseline->get<xgc_attitude_target_v2>(9);
      assert(state.armed == bool(fcu.armed) || !state.enabled);
      assert(state.enabled == bool(fcu.connected));
      assert(std::string(state.mode.data()) == fcu.mode);
      for (int a = 0; a < 3; ++a) {
        compare("imu_specific_force", state.specific_force[a], imu.accel[a]);
        compare("imu_gyro", state.omega[a], imu.gyro[a]);
        compare("target_rate", state.control.desired_body_rate[a],
                target.body_rate[a]);
        compare("target_quaternion",
                state.control.desired_orientation.coeffs()[a],
                target.q_wxyz[a + 1]);
      }
      compare("target_quaternion", state.control.desired_orientation.w(),
              target.q_wxyz[0]);
      compare("thrust", state.control.normalized_thrust, target.thrust);
      if (k == 0 || k == 4000 || k == 4500 || k == 6500) {
        auto r = baseline->get<xgc_fcu_result_v1>(60);
        events.push_back({{"step", k}, {"baseline_last_fcu_result", r.result}});
      }
    }
    if (k % 1000 == 999)
      checkpoints.push_back(
          {{"step", k + 1},
           {"position",
            {state.position.x(), state.position.y(), state.position.z()}},
           {"velocity",
            {state.velocity.x(), state.velocity.y(), state.velocity.z()}},
           {"generation", state.key.generation}});
  }
  Json out = {{"kind", kind},
              {"baseline", "fed3fdc3d6625d9020d363bb5b9c3c8f8f2c72a6"},
              {"steps", 8000},
              {"dt_ns", 1000000},
              {"epoch_ns", epoch},
              {"initial_pose", {2, -1, 0, .35}},
              {"noise", "disabled in both"},
              {"absolute_tolerance", 1e-10},
              {"max_absolute_error", max_error},
              {"events", events},
              {"checkpoints", checkpoints},
              {"result", "PASS"}};
  std::ofstream(argv[3]) << out.dump(2) << '\n';
  std::cout << out.dump() << '\n';
}
