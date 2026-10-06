#include "world.hpp"
#include <cstring>
#include <limits>
#include <pthread.h>
namespace xsim {
namespace {
Eigen::Vector3d vec3(const Json &v) {
  if (!v.is_array() || v.size() != 3)
    throw std::invalid_argument("expected three numbers");
  Eigen::Vector3d r(v[0].get<double>(), v[1].get<double>(), v[2].get<double>());
  if (!r.allFinite())
    throw std::invalid_argument("nonfinite vector");
  return r;
}
bool continuous(Op op) {
  return op == Op::Pva || op == Op::Attitude || op == Op::Velocity;
}
} // namespace
Config parse_entity(const Json &j) {
  Config c;
  c.name = j.at("name").get<std::string>();
  if (c.name.empty() ||
      c.name.find_first_not_of(
          "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") !=
          std::string::npos)
    throw std::invalid_argument("name must be a ROS namespace segment");
  const auto kind = j.at("kind").get<std::string>();
  if (kind == "fs150")
    c.kind = Kind::FS150;
  else if (kind == "scout")
    c.kind = Kind::Scout;
  else if (kind == "mecanum")
    c.kind = Kind::Mecanum;
  else
    throw std::invalid_argument("unknown kind");
  if (j.contains("position"))
    c.initial = vec3(j.at("position"));
  c.yaw = j.value("yaw", 0.0);
  c.ground_z = j.value("ground_z", 0.0);
  if (!std::isfinite(c.yaw) || !std::isfinite(c.ground_z))
    throw std::invalid_argument("nonfinite pose");
  if (j.contains("local_origin"))
    c.local_origin = vec3(j.at("local_origin"));
  if (j.contains("fcu_parameters")) {
    if (c.kind != Kind::FS150)
      throw std::invalid_argument("FCU parameters require fs150");
    for (auto i = j.at("fcu_parameters").begin();
         i != j.at("fcu_parameters").end(); ++i)
      if (!c.fcu.set(i.key(), i.value().get<double>()))
        throw std::invalid_argument("invalid FCU parameter: " + i.key());
  }
  c.ros = j.value("ros", Json::object());
  c.sensor = j.value("sensor", Json::object());
  return c;
}
Model prepare_model(const Config &c) {
  if (c.kind == Kind::FS150)
    return Flight(c);
  if (c.kind == Kind::Scout)
    return Scout(c);
  return Mecanum(c);
}
World::World(int64_t e, int64_t d, int64_t o, unsigned c)
    : epoch(e), dt(d), output_period(o), time_(e), next_output_(e),
      catchup_(c) {
  if (e <= 0 || d <= 0 || o <= 0 || d > INT64_MAX-e || !c || e > INT64_MAX - o)
    throw std::invalid_argument(
        "explicit positive session epoch, step and output period required");
  metrics.sim_ns = e;
  next_output_ = epoch + output_period;
}
World::~World() { stop(); }
void World::submit(const Ticket &t) {
  {
    std::lock_guard<std::mutex> l(input_mutex_);
    if (continuous(t->op)) {
      for (auto i = inbox_.rbegin(); i != inbox_.rend(); ++i) {
        if (!continuous((*i)->op))
          break; // never coalesce across an ordered discrete operation
        if ((*i)->key.id == t->key.id &&
            (*i)->key.generation == t->key.generation && (*i)->op == t->op &&
            (*i)->at == t->at) {
          (*i)->phase = 3;
          *i = t;
          ++metrics.input_misses;
          wake_.notify_one();
          return;
        }
      }
    }
    inbox_.push_back(t);
  }
  wake_.notify_one();
}
bool World::wait(const Ticket &t) {
  std::unique_lock<std::mutex> l(t->mutex);
  while (t->phase.load(std::memory_order_acquire) < 2) {
    if (Clock::now() >= t->deadline) {
      int queued = 0;
      if (t->phase.compare_exchange_strong(queued, 3))
        return false;
      // Claimed work has won the race: await its actual result. World never
      // takes this caller-owned mutex. Short timed waits also cover a notify
      // between the atomic predicate read and condition-variable sleep.
    }
    t->done.wait_for(l, std::chrono::milliseconds(1));
  }
  return t->phase.load(std::memory_order_acquire) == 2;
}
void World::finish(const Ticket &t, Result r) {
  r.applied = true;
  r.stamp = time_;
  r.step = steps_;
  t->result = r;
  t->phase.store(2, std::memory_order_release);
  t->done.notify_all();
}
void World::reset(Slot &s, Model &&m) {
  auto &e = *s.entity;
  const auto i = s.dense;
  if (e.config.kind == Kind::FS150) {
    flights_[i] = std::get<Flight>(std::move(m));
    flights_[i].model.bind(bodies_, i);
    flights_[i].ever_started = e.enabled;
  } else if (e.config.kind == Kind::Scout) {
    scouts_[i] = std::get<Scout>(std::move(m));
    scouts_[i].model.bind(scout_poses_, i);
  } else {
    mecanums_[i] = std::get<Mecanum>(std::move(m));
    mecanums_[i].model.bind(mecanum_poses_, i);
  }
  ++e.generation;
  e.generation_stamp = time_;
}
Result World::apply(Command &c) {
  Result r;
  r.success = true;
  if (c.op == Op::Pause) {
    metrics.paused = true;
    return r;
  }
  if (c.op == Op::Resume) {
    metrics.paused = false;
    return r;
  }
  if (c.op == Op::Step) {
    r.success = metrics.paused && c.steps > 0 && stepping_ == 0 && c.steps<=uint64_t((INT64_MAX-time_)/dt);
    if (r.success)
      stepping_ = c.steps;
    return r;
  }
  if (c.op == Op::Add) {
    auto &p = *c.prepared;
    auto e = p.entity;
    for (const auto &s : slots_)
      if (s.second.entity->config.name == e->config.name) {
        r.success = false;
        r.reason = 4;
        return r;
      }
    e->id = next_id_++;
    e->generation_stamp = time_;
    size_t i;
    if (e->config.kind == Kind::FS150) {
      i = flights_.size();
      bodies_.append(std::get<Flight>(p.model).model.body_state());
      flights_.push_back(std::get<Flight>(std::move(p.model)));
      flights_.back().model.bind(bodies_, i);
      flight_ids_.push_back(e->id);
    } else if (e->config.kind == Kind::Scout) {
      i = scouts_.size();
      scout_poses_.append(std::get<Scout>(p.model).model.pose());
      scouts_.push_back(std::get<Scout>(std::move(p.model)));
      scouts_.back().model.bind(scout_poses_, i);
      scout_ids_.push_back(e->id);
    } else {
      i = mecanums_.size();
      mecanum_poses_.append(std::get<Mecanum>(p.model).model.pose());
      mecanums_.push_back(std::get<Mecanum>(std::move(p.model)));
      mecanums_.back().model.bind(mecanum_poses_, i);
      mecanum_ids_.push_back(e->id);
    }
    slots_.emplace(e->id, Slot{e, i});
    e->alive = true;
    scratch_.states.reserve(slots_.size());
    r.key = {e->id, e->generation};
    return r;
  }
  if (c.op == Op::Reset && !c.key.id) {
    if (c.resets.size() != slots_.size()) {
      r.success = false;
      r.reason = 1;
      return r;
    }
    for (auto &v : c.resets) {
      auto s = slots_.find(v.first.id);
      if (s == slots_.end() ||
          s->second.entity->generation != v.first.generation) {
        r.success = false;
        r.reason = 1;
        return r;
      }
    }
    for (auto &v : c.resets)
      reset(slots_.at(v.first.id), std::move(v.second));
    return r;
  }
  auto found = slots_.find(c.key.id);
  if (found == slots_.end()) {
    r.success = false;
    r.reason = 1;
    return r;
  }
  auto &s = found->second;
  auto &e = *s.entity;
  auto i = s.dense;
  r.key = {e.id, e.generation};
  r.enabled = e.enabled;
  const bool observe = c.op == Op::Provider && c.action == 0;
  const bool repeated_start = c.op == Op::Provider && c.action == 1 &&
                              e.enabled && e.generation > 0 &&
                              c.key.generation == e.generation - 1;
  if (!observe && !repeated_start && c.key.generation != e.generation) {
    r.success = false;
    r.reason = 1;
    return r;
  }
  if (c.op == Op::Remove) {
    c.retired = s.entity;
    e.alive = false;
    e.enabled = false;
    auto erase = [&](auto &models, auto &ids, auto &columns) {
      if (i + 1 != models.size()) {
        models[i] = std::move(models.back());
        ids[i] = ids.back();
        slots_.at(ids[i]).dense = i;
        models[i].model.rebind(i);
      }
      columns.erase(i);
      models.pop_back();
      ids.pop_back();
    };
    if (e.config.kind == Kind::FS150)
      erase(flights_, flight_ids_, bodies_);
    else if (e.config.kind == Kind::Scout)
      erase(scouts_, scout_ids_, scout_poses_);
    else
      erase(mecanums_, mecanum_ids_, mecanum_poses_);
    slots_.erase(found);
    return r;
  }
  if (c.op == Op::Reset) {
    reset(s, std::move(c.prepared->model));
    r.key.generation = e.generation;
    return r;
  }
  if (c.op == Op::Provider) {
    if (c.action == 1 && !e.enabled) {
      reset(s, std::move(c.prepared->model));
      e.enabled = true;
      if (e.config.kind == Kind::FS150)
        flights_[i].ever_started = true;
    } else if (c.action == 2) {
      e.enabled = false;
      if (e.config.kind == Kind::Scout)
        scouts_[i].model.command(scouts_[i].age, 0, 0);
      if (e.config.kind == Kind::Mecanum)
        mecanums_[i].model.command(0, 0, 0);
    } else if (c.action < 0 || c.action > 2) {
      r.success = false;
      r.reason = 2;
    }
    r.key.generation = e.generation;
    r.enabled = e.enabled;
    return r;
  }
  if (!e.enabled) {
    r.success = false;
    r.reason = 2;
    return r;
  }
  if (c.op == Op::Velocity && e.config.kind != Kind::FS150) {
    if (!c.velocity.allFinite()) {
      r.success = false;
      r.reason = 2;
      return r;
    }
    if (e.config.kind == Kind::Scout)
      scouts_[i].model.command(scouts_[i].age, c.velocity.x(), c.velocity.z());
    else
      mecanums_[i].model.command(c.velocity.x(), c.velocity.y(),
                                 c.velocity.z());
    return r;
  }
  if (e.config.kind != Kind::FS150) {
    r.success = false;
    r.reason = 3;
    return r;
  }
  auto &f = flights_[i];
  if (c.op == Op::Arm && c.action != 0) {
    r.success = false;
    r.reason = 3;
    return r;
  }
  if (c.op == Op::Arm)
    r.success = f.model.request_arm(c.arm);
  else if (c.op == Op::Mode) {
    FlightMode mode;
    if (c.mode == "OFFBOARD")
      mode = FlightMode::Offboard;
    else if (c.mode == "POSCTL" || c.mode == "ALTCTL" ||
             c.mode == "AUTO.LOITER")
      mode = FlightMode::Hold;
    else if (c.mode == "AUTO.LAND")
      mode = FlightMode::Land;
    else {
      r.success = false;
      r.reason = 3;
      return r;
    }
    r.success = f.model.request_mode(mode);
    if (r.success)
      f.mode = c.mode;
  } else if (c.op == Op::Pva) {
    // ROS local position coordinates differ by the declared origin translation.
    auto p = c.pva;
    if (p.coordinate_frame == 1)
      for (int axis = 0; axis < 3; ++axis)
        p.position[axis] += e.config.local_origin[axis];
    r.success = f.model.setpoint(
        decodePositionTarget(p, f.model.orientation(), f.model.yaw()));
  } else if (c.op == Op::Attitude)
    r.success = f.model.attitude_setpoint(c.attitude);
  else {
    r.success = false;
    r.reason = 3;
  }
  if (!r.success && !r.reason)
    r.reason = 2;
  return r;
}
void World::boundary() {
  {
    std::unique_lock<std::mutex> l(input_mutex_, std::try_to_lock);
    if (l.owns_lock())
      commands_.splice(commands_.end(), inbox_);
  }
  for (auto i = commands_.begin(); i != commands_.end();) {
    auto t = *i;
    if (t->at > time_ && continuous(t->op) && Clock::now() < t->deadline &&
        t->phase == 0) {
      ++i;
      continue;
    }
    i = commands_.erase(i);
    int expected = 0;
    if (Clock::now() >= t->deadline) {
      t->phase.compare_exchange_strong(expected, 3);
      t->done.notify_all();
      continue;
    }
    if (!t->phase.compare_exchange_strong(expected, 1))
      continue;
    Result r;
    try {
      r = apply(*t);
    } catch (const std::exception &) {
      r.success = false;
      r.reason = 5;
    }
    if (r.success)
      ++revision_;
    if (t->op == Op::Step && r.success)
      step_waiters_.push_back(t);
    else
      finish(t, r);
  }
}
void World::advance() {
  const auto begin = Clock::now();
  if(steps_>=uint64_t((INT64_MAX-epoch)/dt)) {metrics.paused=true;return;}
  const double h = double(dt) * 1e-9;
  for (size_t i = 0; i < flights_.size(); ++i) {
    auto &f = flights_[i];
    auto &e = *slots_.at(flight_ids_[i]).entity;
    if (e.enabled) {
      if (f.model.step(h) == FlightEvent::OffboardLost)
        f.mode = "AUTO.LOITER";
    } else if (f.ever_started)
      f.model.stepPhysicsOnly(h);
  }
  for (auto &s : scouts_) {
    ++s.steps;
    s.age = double(int64_t(s.steps) * dt) * 1e-9;
    s.model.advance(s.age);
  }
  for (auto &m : mecanums_)
    m.model.step(h);
  ++steps_;
  time_ = epoch + int64_t(steps_) * dt;
  metrics.steps = steps_;
  metrics.sim_ns = time_;
  if (sample_sensor)
    for (const auto &s : slots_)
      if (s.second.entity->sensor && s.second.entity->enabled)
        sample_sensor(state(s.first, s.second));
  if (time_ >= next_output_) {
    emit();
    const auto next_index=(time_-epoch)/output_period+1;
    next_output_=next_index>(INT64_MAX-epoch)/output_period?INT64_MAX:epoch+next_index*output_period;
  }
  if (stepping_ && --stepping_ == 0) {
    for (auto &t : step_waiters_) {
      Result r;
      r.success = true;
      finish(t, r);
    }
    step_waiters_.clear();
    emit();
  }
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - begin)
          .count();
  metrics.step_latency_ns = elapsed;
  if (elapsed > metrics.max_step_latency_ns)
    metrics.max_step_latency_ns = elapsed;
}
State World::state(uint64_t id, const Slot &s) const {
  State v;
  v.entity = s.entity;
  auto &e = *s.entity;
  v.key = {id, e.generation};
  v.stamp = time_;
  v.enabled = e.enabled;
  const auto i = s.dense;
  if (e.config.kind == Kind::FS150) {
    const auto &f = flights_[i];
    v.position = f.model.state().position;
    v.velocity = f.model.state().velocity;
    v.orientation = f.model.orientation();
    v.omega = f.model.angular_velocity_body();
    v.specific_force = f.model.specific_force_body();
    v.armed = f.model.armed();
    v.landed = f.model.landed();
    v.control = f.model.control_output();
    v.rotors = f.model.rotor_speed();
    std::strncpy(v.mode.data(), f.mode.c_str(), v.mode.size() - 1);
  } else {
    auto pose = e.config.kind == Kind::Scout ? scouts_[i].model.pose()
                                             : mecanums_[i].model.pose();
    Eigen::Vector2d body;
    if (e.config.kind == Kind::Scout) {
      auto u = scouts_[i].model.velocity();
      body = {u.linear_m_s, 0};
      v.omega.z() = u.yaw_rad_s;
    } else {
      body = mecanums_[i].model.body_velocity();
      v.omega.z() = mecanums_[i].model.yaw_rate();
    }
    v.position = {pose.position.x(), pose.position.y(), e.config.initial.z()};
    v.orientation = Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ());
    v.velocity.head<2>() = xgc2_math::rotationMatrix2(pose.yaw) * body;
  }
  return v;
}
Frame World::capture() const {
  Frame f;
  f.stamp = time_;
  f.steps = steps_;
  for (const auto &s : slots_)
    f.states.push_back(state(s.first, s.second));
  return f;
}
void World::emit() {
  if (emitted_stamp_ == time_ && emitted_revision_ == revision_)
    return;
  std::unique_lock<std::mutex> l(output_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    ++metrics.output_misses;
    return;
  }
  if (ready_available_)
    ++metrics.output_misses;
  ready_.states.clear();
  ready_.states.reserve(slots_.size());
  for (const auto &s : slots_)
    ready_.states.push_back(state(s.first, s.second));
  ready_.stamp = time_;
  ready_.steps = steps_;
  ready_.revision = revision_;
  emitted_revision_ = revision_;
  emitted_stamp_ = time_;
  ready_available_ = true;
}
bool World::take_frame(Frame &f) {
  std::lock_guard<std::mutex> l(output_mutex_);
  if (!ready_available_)
    return false;
  std::swap(f, ready_);
  ready_available_ = false;
  return true;
}
void World::start() {
  if (running_.exchange(true))
    return;
  thread_ = std::thread([this] { run(); });
}
void World::stop() {
  running_ = false;
  wake_.notify_all();
  if (thread_.joinable())
    thread_.join();
  std::lock_guard<std::mutex> l(input_mutex_);
  commands_.splice(commands_.end(), inbox_);
  for (auto &t : commands_) {
    t->phase = 3;
    t->done.notify_all();
  }
  commands_.clear();
  for (auto &t : step_waiters_) {
    t->phase = 3;
    t->done.notify_all();
  }
  step_waiters_.clear();
}
void World::run() {
  pthread_setname_np(pthread_self(), "xsim-world");
  auto deadline = Clock::now() + std::chrono::nanoseconds(dt);
  bool was_paused = metrics.paused;
  emit();
  while (running_) {
    boundary();
    bool paused = metrics.paused;
    if (paused && !stepping_) {
      emit();
      std::unique_lock<std::mutex> l(wake_mutex_);
      wake_.wait_for(l, std::chrono::milliseconds(1));
      was_paused = true;
      continue;
    }
    if (was_paused) {
      deadline = Clock::now() + std::chrono::nanoseconds(dt);
      was_paused = false;
    }
    unsigned batch = 0;
    while (running_ && (!metrics.paused || stepping_) &&
           Clock::now() >= deadline && batch++ < catchup_) {
      advance();
      deadline += std::chrono::nanoseconds(dt);
      boundary();
    }
    metrics.lag_ns = std::max<int64_t>(
        0, std::chrono::duration_cast<std::chrono::nanoseconds>(
               Clock::now() - (deadline - std::chrono::nanoseconds(dt)))
               .count());
    if (Clock::now() < deadline) {
      std::unique_lock<std::mutex> l(wake_mutex_);
      wake_.wait_until(l, deadline);
    } else
      std::this_thread::yield();
  }
}
} // namespace xsim
