#include "flat_config.hpp"
#include "vehicle_model.hpp"
#include "fcu_parameter_config.hpp"
#include "xgc_dmpc_planner_v1.h"
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

#include <array>
#include <cstring>
#include <deque>
#include <limits>
#include <iomanip>
#include <sstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

// One instance advances a batch of same-type robots (one by default). Their
// model state is one contiguous vector; each step reads the host time once,
// advances every robot to that one grid point with the same per-robot function
// a single-robot instance uses, and publishes only at the output period.
// Robot r owns port block r: `setpoint` ... `paired_state` for robot 0 and
// `setpoint_r` ... `paired_state_r` after it. The ABI allows 64 ports, so one
// instance holds at most 6 robots of 10 ports.
namespace {
enum Port {
  Setpoint,
  VelocityCommand,
  FcuRequest,
  Pose,
  Velocity,
  Imu,
  FcuState,
  PairedState,
  AttitudeCommand,
  AttitudeTarget,
  kPortsPerRobot
};
constexpr uint32_t kMaxRobots = XGC_RT_MAX_PORTS / kPortsPerRobot;
constexpr uint32_t kFcuResultPort = kMaxRobots * kPortsPerRobot;
constexpr uint32_t kFcuExtendedStatePort = kFcuResultPort + 1;
constexpr uint32_t kProviderRequestPort = kFcuExtendedStatePort + 1;
constexpr uint32_t kProviderResultPort = kProviderRequestPort + 1;
static_assert(kProviderResultPort < XGC_RT_MAX_PORTS, "FCU batch outputs fit the ABI");

struct Input {
  int64_t at;
  Port port;
  uint64_t generation{0};
  std::array<uint8_t, sizeof(xgc_position_target_v1)> data{};
};

int64_t nanoseconds(double seconds) {
  if (!std::isfinite(seconds) || seconds < 0 ||
      seconds >= double(INT64_MAX) * 1e-9)
    throw std::invalid_argument("lightweight-vehicle: invalid command time");
  return static_cast<int64_t>(std::llround(seconds * 1e9));
}

// Counts a repeated condition; true on the 1st, 2nd, 4th, 8th... occurrence so
// a persistent fault stays visible without flooding the health log.
struct Occurrences {
  uint64_t count{0};
  bool report() {
    ++count;
    return (count & (count - 1)) == 0;
  }
};

bool flight_mode(const std::string &name, xgc_lightweight::FlightMode *mode) {
  using xgc_lightweight::FlightMode;
  if (name == "OFFBOARD")
    *mode = FlightMode::Offboard;
  else if (name == "POSCTL" || name == "ALTCTL" || name == "AUTO.LOITER")
    *mode = FlightMode::Hold;
  else if (name == "AUTO.LAND")
    *mode = FlightMode::Land;
  else
    return false;
  return true;
}

// The time grid and limits every robot of the instance shares.
struct Grid {
  bool advance_on_round{false}, trace_state{false};
  const xgc_host_api *host;
  uint32_t robots{1};
  int64_t epoch{0}, time{0}, step_ns{1000000}, output_ns{10000000},
      next_output{0}, max_future_ns{1000000000};
  size_t max_pending{1024};
  uint64_t round{0};

  void log(xgc_log_level level, uint32_t robot,
           const std::string &message) const {
    const auto who =
        robots > 1 ? "robot " + std::to_string(robot) + ": " : std::string();
    host->log(host->host, level,
              ("lightweight-vehicle: " + who + message).c_str());
  }
  template <class T>
  void publish(uint32_t robot, Port port, uint64_t round,
               const T &value) const {
    if (host->publish(host->host, robot * kPortsPerRobot + port, round,
                      reinterpret_cast<const uint8_t *>(&value),
                      sizeof value) != XGC_OK)
      throw std::runtime_error("lightweight-vehicle: state publish failed");
  }
  template <class T>
  void publish_batch(uint32_t port, const T &value) const {
    if (host->publish(host->host, port, round,
                      reinterpret_cast<const uint8_t *>(&value), sizeof value) != XGC_OK)
      throw std::runtime_error("lightweight-vehicle: FCU batch publication failed");
  }
};

// One robot's received controls, ordered by the grid time they take effect.
struct Commands {
  std::deque<Input> pending;
  Occurrences future_drops, full_drops, invalid_drops;

  void drain(const Grid &grid, uint32_t robot, Port port, size_t size) {
    xgc_sample_view sample{};
    while (grid.host->next(grid.host->host, robot * kPortsPerRobot + port,
                           &sample) == XGC_OK) {
      if (sample.len != size || sample.data == nullptr) {
        if (invalid_drops.report())
          grid.log(XGC_LOG_WARN, robot, "rejected malformed control payload");
        continue;
      }
      double stamp;
      std::memcpy(&stamp, sample.data, sizeof(stamp));
      int64_t effective;
      try {
        effective = nanoseconds(stamp);
      } catch (const std::invalid_argument &) {
        if (invalid_drops.report())
          grid.log(XGC_LOG_WARN, robot, "rejected invalid control timestamp");
        continue;
      }
      int64_t ahead;
      const bool unrepresentable =
          __builtin_sub_overflow(effective, sample.t_rx, &ahead);
      if (unrepresentable || ahead > grid.max_future_ns) {
        if (future_drops.report())
          grid.log(XGC_LOG_WARN, robot,
                   "dropped " + std::to_string(future_drops.count) +
                       " control(s) stamped beyond max_future_ms after "
                       "receipt (last " +
                       (unrepresentable ? std::string("out of range")
                                        : std::to_string(ahead / 1000000) +
                                              " ms ahead") +
                       ")");
        continue;
      }
      if (pending.size() >= grid.max_pending) {
        if (full_drops.report())
          grid.log(XGC_LOG_WARN, robot,
                   "dropped " + std::to_string(full_drops.count) +
                       " control(s): max_pending controls already queued");
        continue;
      }
      // Never use a future control early or rewrite an already integrated past.
      Input input{std::max({grid.time, sample.t_rx, effective}), port, 0, {}};
      std::memcpy(input.data.data(), sample.data, size);
      auto at = std::upper_bound(
          pending.begin(), pending.end(), input.at,
          [](int64_t t, const Input &item) { return t < item.at; });
      pending.insert(at, input);
    }
  }
};

xgc_twist_v1 body_velocity(const Input &input) {
  xgc_twist_v1 value;
  std::memcpy(&value, input.data.data(), sizeof value);
  if (!std::isfinite(value.linear[0]) || !std::isfinite(value.linear[1]) ||
      !std::isfinite(value.angular[2]))
    throw std::invalid_argument("lightweight-vehicle: nonfinite body velocity");
  return value;
}

void planar_output(const Grid &grid, uint32_t robot, uint64_t round,
                   const xgc2_math::Pose2 &state, double z,
                   const Eigen::Vector2d &body, double yaw_rate) {
  const double stamp = double(grid.time) * 1e-9;
  xgc_pose_v1 pose{};
  pose.stamp = stamp;
  pose.position[0] = state.position.x();
  pose.position[1] = state.position.y();
  pose.position[2] = z;
  pose.q_wxyz[0] = std::cos(state.yaw * 0.5);
  pose.q_wxyz[3] = std::sin(state.yaw * 0.5);
  xgc_twist_v1 velocity{};
  velocity.stamp = stamp;
  const auto world = (xgc2_math::rotationMatrix2(state.yaw) * body).eval();
  velocity.linear[0] = world.x();
  velocity.linear[1] = world.y();
  velocity.angular[2] = yaw_rate;
  xgc_dmpc_paired_state_v1 paired{};
  paired.pose_stamp_sec = paired.twist_stamp_sec = stamp;
  std::copy(pose.position, pose.position + 3, paired.position);
  paired.orientation_xyzw[2] = pose.q_wxyz[3];
  paired.orientation_xyzw[3] = pose.q_wxyz[0];
  std::copy(velocity.linear, velocity.linear + 3, paired.linear_velocity);
  grid.publish(robot, Pose, round, pose);
  grid.publish(robot, Velocity, round, velocity);
  grid.publish(robot, PairedState, round, paired);
}

struct FlightRobot {
  xgc_lightweight::FlightModel model;
  Commands commands;
  uint64_t generation{0};
  bool provider_enabled{false}, ever_started{false};
  std::string fcu_mode{"POSCTL"};
  Occurrences refused_disarm, refused_mode, refused_offboard, refused_attitude, refused_setpoint, actuator_saturation;

  FlightRobot(const double *initial, const xgc_lightweight::FlightControllerParameters &parameters, double ground_z)
      : model(Eigen::Vector3d(initial[0], initial[1], initial[2]), initial[3], parameters, ground_z) {}

  void apply(const Grid &grid, uint32_t robot, const Input &input, int64_t applied_at) {
    if (!provider_enabled || input.generation != generation) return;
    if (input.port == VelocityCommand) {
      grid.log(XGC_LOG_WARN, robot, "rejected ground velocity on flight model");
      return;
    }
    if (input.port == AttitudeCommand) {
      xgc_attitude_target_v2 wire;
      std::memcpy(&wire, input.data.data(), sizeof wire);
      xgc_lightweight::FlightAttitudeSetpoint value;
      value.q = Eigen::Quaterniond(wire.q_wxyz[0], wire.q_wxyz[1],
                                   wire.q_wxyz[2], wire.q_wxyz[3]);
      value.body_rate = Eigen::Vector3d(wire.body_rate[0], wire.body_rate[1], wire.body_rate[2]);
      value.thrust = wire.thrust;
      value.type_mask = wire.type_mask;
      if (!model.attitude_setpoint(value) && refused_attitude.report())
        grid.log(XGC_LOG_WARN, robot,
                 "refused invalid or unsupported attitude mask; previous command retained (" +
                     std::to_string(refused_attitude.count) + " time(s))");
      return;
    }
    if (input.port == Setpoint) {
      xgc_position_target_v1 wire;
      std::memcpy(&wire, input.data.data(), sizeof wire);
      const auto value = xgc_lightweight::decodePositionTarget(wire, model.orientation(), model.yaw());
      if (!model.setpoint(value) && refused_setpoint.report())
        grid.log(XGC_LOG_WARN, robot, "rejected PVA receiver input; valid stream not refreshed");
      return;
    }
    xgc_fcu_request_v2 request;
    std::memcpy(&request, input.data.data(), sizeof request);
    xgc_fcu_result_v1 result{};
    result.stamp = double(applied_at) * 1e-9;
    result.request_stamp = request.stamp;
    result.request_id = request.request_id;
    result.robot_index = robot;
    result.kind = request.kind;
    result.result = 3; // unsupported until a defined request is executed
    if (request.flags != 0) {
      // This virtual FCU does not implement forced disarm. Preserve and
      // explicitly refuse it; never turn it into an ordinary disarm.
    } else if (request.kind == 1 && request.arm <= 1) {
      const bool accepted = model.request_arm(request.arm != 0);
      result.result = accepted ? 0 : 2;
      if (!accepted && refused_disarm.report())
        grid.log(XGC_LOG_WARN, robot,
                 "disarm refused while airborne (" +
                     std::to_string(refused_disarm.count) + " time(s))");
    } else if (request.kind == 2) {
      const auto end = static_cast<const char *>(
          std::memchr(request.mode, 0, sizeof request.mode));
      const std::string name(request.mode, end ? static_cast<size_t>(end-request.mode) : sizeof request.mode);
      xgc_lightweight::FlightMode mode;
      if (!end || !flight_mode(name, &mode)) {
        if (refused_mode.report())
          grid.log(XGC_LOG_WARN, robot,
                   "FCU mode " + name + " is not modelled; " + fcu_mode +
                       " kept (" + std::to_string(refused_mode.count) +
                       " unmodelled mode request(s))");
      } else if (!model.request_mode(mode)) {
        result.result = 2;
        if (refused_offboard.report())
          grid.log(XGC_LOG_WARN, robot,
                   "OFFBOARD refused without a fresh setpoint; " + fcu_mode +
                       " kept (" + std::to_string(refused_offboard.count) +
                       " time(s))");
      } else {
        fcu_mode = name;
        result.result = 0;
      }
    }
    grid.publish_batch(kFcuResultPort, result);
  }

  void step(const Grid &grid, uint32_t robot, int64_t) {
    if (!provider_enabled) {
      if (ever_started) model.stepPhysicsOnly(double(grid.step_ns) * 1e-9);
      return;
    }
    const auto event = model.step(double(grid.step_ns) * 1e-9);
    if (model.control_output().allocation.saturated() && actuator_saturation.report())
      grid.log(XGC_LOG_WARN, robot,
               "rotor allocation saturated (" +
                   std::to_string(actuator_saturation.count) + " step(s))");
    if (event == xgc_lightweight::FlightEvent::OffboardLost) {
      fcu_mode = "AUTO.LOITER";
      grid.log(XGC_LOG_WARN, robot,
               "no setpoint for offboard_timeout_ms; holding in AUTO.LOITER");
    } else if (event == xgc_lightweight::FlightEvent::Landed)
      grid.log(XGC_LOG_INFO, robot, "AUTO.LAND touchdown; disarmed");
  }

  void output(const Grid &grid, uint32_t robot, uint64_t round) const {
    const double stamp = double(grid.time) * 1e-9;
    const auto &orientation = model.orientation();
    xgc_pose_v1 pose{};
    pose.stamp = stamp;
    xgc_twist_v1 velocity{};
    velocity.stamp = stamp;
    for (int i = 0; i != 3; ++i) {
      pose.position[i] = model.state().position[i];
      velocity.linear[i] = model.state().velocity[i];
    }
    pose.q_wxyz[0] = orientation.w();
    pose.q_wxyz[1] = orientation.x();
    pose.q_wxyz[2] = orientation.y();
    pose.q_wxyz[3] = orientation.z();
    // The pose is body FLU -> world ENU. Twist is wholly world-frame;
    // IMU specific force and gyro stay in body axes at the same base origin.
    const Eigen::Vector3d omega_body = model.angular_velocity_body();
    const Eigen::Vector3d omega_world = orientation * omega_body;
    const Eigen::Vector3d specific_force = model.specific_force_body();
    xgc_imu_v1 imu{};
    imu.stamp = stamp;
    for (int i = 0; i != 3; ++i) {
      velocity.angular[i] = omega_world[i];
      imu.gyro[i] = omega_body[i];
      imu.accel[i] = specific_force[i];
    }
    xgc_fcu_state_v1 state{};
    state.stamp = stamp;
    state.connected = provider_enabled;
    state.armed = provider_enabled && model.armed();
    state.guided = provider_enabled;
    state.system_status = !provider_enabled ? 0 : model.armed() ? 4 : 3; // MAV_STATE_ACTIVE/STANDBY
    std::strcpy(state.mode, fcu_mode.c_str());
    xgc_dmpc_paired_state_v1 paired{};
    paired.pose_stamp_sec = paired.twist_stamp_sec = stamp;
    std::copy(pose.position, pose.position + 3, paired.position);
    for (int i = 0; i != 3; ++i)
      paired.orientation_xyzw[i] = pose.q_wxyz[i + 1];
    paired.orientation_xyzw[3] = pose.q_wxyz[0];
    std::copy(velocity.linear, velocity.linear + 3, paired.linear_velocity);
    const auto &control = model.control_output();
    xgc_attitude_target_v2 target{};
    target.stamp = stamp;
    target.q_wxyz[0] = control.desired_orientation.w();
    target.q_wxyz[1] = control.desired_orientation.x();
    target.q_wxyz[2] = control.desired_orientation.y();
    target.q_wxyz[3] = control.desired_orientation.z();
    for (int i = 0; i != 3; ++i)
      target.body_rate[i] = control.desired_body_rate[i];
    target.thrust = control.normalized_thrust;
    target.type_mask = control.type_mask;
    grid.publish(robot, FcuState, round, state);
    grid.publish(robot, Pose, round, pose);
    grid.publish(robot, Velocity, round, velocity);
    grid.publish(robot, Imu, round, imu);
    grid.publish(robot, PairedState, round, paired);
    grid.publish(robot, AttitudeTarget, round, target);
    if (grid.trace_state) {
      // Read-only audit of the same physical step; never synthesize motors from
      // thrust or infer state from setpoints. Bounded by the output period.
      std::ostringstream line;
      line << std::setprecision(17) << "{\"kind\":\"fs150_state_audit/1\",\"stamp\":" << stamp
           << ",\"robot_index\":" << robot << ",\"generation\":" << generation;
      const auto array = [&](const char *name, const auto &values, int count) {
        line << ",\"" << name << "\":[";
        for (int i = 0; i < count; ++i) { if (i) line << ','; line << values[i]; }
        line << ']';
      };
      array("q_flu_to_enu_wxyz", pose.q_wxyz, 4);
      array("position_base_enu", pose.position, 3);
      array("omega_body", omega_body, 3);
      array("rotor_omega", model.rotor_speed(), 4);
      array("target_rotor_omega", control.allocation.target_rotor_speed, 4);
      line << ",\"normalized_thrust\":" << control.normalized_thrust
           << ",\"mixer_flags\":" << control.allocation.saturation_status << '}';
      grid.log(XGC_LOG_INFO, robot, line.str());
    }
  }
};

struct ScoutRobot {
  xgc_lightweight::ScoutModel model;
  Commands commands;
  double z;

  ScoutRobot(const double *initial, double)
      : model(xgc2_math::Pose2{{initial[0], initial[1]}, initial[3]}),
        z(initial[2]) {}

  void apply(const Grid &grid, uint32_t robot, const Input &input, int64_t t) {
    if (input.port != VelocityCommand) {
      grid.log(XGC_LOG_WARN, robot, "rejected flight command on ground model");
      return;
    }
    xgc_twist_v1 value;
    try { value = body_velocity(input); }
    catch (const std::invalid_argument &) {
      grid.log(XGC_LOG_WARN, robot, "rejected nonfinite body velocity");
      return;
    }
    model.command(double(t - grid.epoch) * 1e-9, value.linear[0],
                  value.angular[2]);
  }
  void step(const Grid &grid, uint32_t, int64_t t) {
    model.advance(double(t + grid.step_ns - grid.epoch) * 1e-9);
  }
  void output(const Grid &grid, uint32_t robot, uint64_t round) const {
    const auto response = model.velocity();
    planar_output(grid, robot, round, model.pose(), z,
                  {response.linear_m_s, 0.0}, response.yaw_rad_s);
  }
};

struct MecanumRobot {
  xgc_lightweight::MecanumModel model;
  Commands commands;
  double z;

  MecanumRobot(const double *initial, double)
      : model(xgc2_math::Pose2{{initial[0], initial[1]}, initial[3]}),
        z(initial[2]) {}

  void apply(const Grid &grid, uint32_t robot, const Input &input, int64_t) {
    if (input.port != VelocityCommand) {
      grid.log(XGC_LOG_WARN, robot, "rejected flight command on ground model");
      return;
    }
    xgc_twist_v1 value;
    try { value = body_velocity(input); }
    catch (const std::invalid_argument &) {
      grid.log(XGC_LOG_WARN, robot, "rejected nonfinite body velocity");
      return;
    }
    model.command(value.linear[0], value.linear[1], value.angular[2]);
  }
  void step(const Grid &grid, uint32_t, int64_t) {
    model.step(double(grid.step_ns) * 1e-9);
  }
  void output(const Grid &grid, uint32_t robot, uint64_t round) const {
    planar_output(grid, robot, round, model.pose(), z, model.body_velocity(),
                  model.yaw_rate());
  }
};

// The one advance function of single, batched and distributed placement:
// apply the controls due at each grid boundary, then integrate one step.
template <class Robot>
void advance(const Grid &grid, uint32_t index, Robot &robot, int64_t target) {
  auto &pending = robot.commands.pending;
  for (int64_t t = grid.time; t < target; t += grid.step_ns) {
    while (!pending.empty() && pending.front().at <= t) {
      robot.apply(grid, index, pending.front(), t);
      pending.pop_front();
    }
    robot.step(grid, index, t);
  }
}

struct Plant {
  Grid grid;
  std::vector<FlightRobot> flights;
  std::vector<ScoutRobot> scouts;
  std::vector<MecanumRobot> mecanums;

  explicit Plant(const xgc_host_api *host) { grid.host = host; }

  void configure(const char *text) {
    namespace cfg = xgc_rt_config;
    std::string config(text ? text : ""), epoch_text;
    const auto model = cfg::text_or(config, "model", "fs150");
    if (model != "fs150" && model != "scout" && model != "mecanum")
      throw std::invalid_argument("lightweight-vehicle: unknown model");
    if (!cfg::value(config, "epoch_ns", &epoch_text))
      throw std::invalid_argument(
          "lightweight-vehicle: shared epoch_ns is required");
    grid.epoch = grid.time = grid.next_output = std::stoll(epoch_text);
    int robots = 1, step_ms = 1, output_ms = 10,
        max_future_ms = 1000, pending_limit = 1024;
    if (!cfg::integer(config, "robots", &robots) || robots < 1 ||
        robots > int(kMaxRobots))
      throw std::invalid_argument(
          "lightweight-vehicle: robots must be 1.." +
          std::to_string(kMaxRobots) + " (10 ports each, 64 per instance)");
    std::vector<double> initial(4 * size_t(robots), 0.0);
    std::string unused;
    const bool poses = cfg::value(config, "initial_poses", &unused);
    if ((poses && cfg::value(config, "initial_pose", &unused)) ||
        (!poses && robots > 1 && cfg::value(config, "initial_pose", &unused)))
      throw std::invalid_argument(
          "lightweight-vehicle: use initial_poses (4 numbers per robot) for a "
          "batch, initial_pose for one robot");
    if (!cfg::integer(config, "step_ms", &step_ms) || step_ms <= 0 ||
        !cfg::integer(config, "output_ms", &output_ms) || output_ms < step_ms ||
        !(poses ? cfg::numbers(config, "initial_poses", initial.data(),
                               initial.size())
                : cfg::numbers(config, "initial_pose", initial.data(), 4)) ||
        !std::all_of(initial.begin(), initial.end(),
                     [](double v) { return std::isfinite(v); }))
      throw std::invalid_argument(
          "lightweight-vehicle: invalid step, output period or initial pose");
    // PX4 COM_OF_LOSS_T role; a future-stamped control beyond max_future_ms
    // is a clock-domain error, not a schedule. Both bound what a silent or
    // mis-stamped controller can do to the plant.
    if (!cfg::integer(config, "max_future_ms", &max_future_ms) ||
        max_future_ms < 0 ||
        !cfg::integer(config, "max_pending", &pending_limit) ||
        pending_limit <= 0)
      throw std::invalid_argument(
          "lightweight-vehicle: invalid offboard timeout or command bounds");
    grid.robots = uint32_t(robots);
    grid.step_ns = int64_t(step_ms) * 1000000;
    grid.output_ns = int64_t(output_ms) * 1000000;
    grid.max_future_ns = int64_t(max_future_ms) * 1000000;
    grid.max_pending = size_t(pending_limit);
    if (!cfg::boolean(config, "trace_state", &grid.trace_state))
      throw std::invalid_argument("lightweight-vehicle: trace_state must be boolean");
    if (!cfg::boolean(config, "advance_on_round", &grid.advance_on_round))
      throw std::invalid_argument("lightweight-vehicle: advance_on_round must be boolean");
    flights.clear();
    scouts.clear();
    mecanums.clear();
    std::string retired;
    if (cfg::value(config, "offboard_timeout_ms", &retired) || cfg::value(config, "hover_thrust_ratio", &retired))
      throw std::invalid_argument("use the frozen fcu_parameters block; legacy FCU aliases are retired");
    double world_ground_z = 0.0;
    if (!cfg::number(config, "world_ground_z", &world_ground_z) || !std::isfinite(world_ground_z))
      throw std::invalid_argument("world_ground_z must be finite");
    if (model == "fs150") {
      xgc_lightweight::FlightControllerParameters parameters;
      xgc_lightweight::apply_fcu_parameter_config(config, parameters);
      flights.reserve(grid.robots);
      for (uint32_t i = 0; i != grid.robots; ++i)
        flights.emplace_back(&initial[4 * i], parameters, world_ground_z);
    } else {
      if (config.find("fcu_parameters") != std::string::npos)
        throw std::invalid_argument("FCU parameters require an FS150 plant");
      if (model == "scout") create(scouts, initial, 0.0);
      else create(mecanums, initial, 0.0);
    }
  }

  template <class Robot>
  void create(std::vector<Robot> &robots, const std::vector<double> &initial,
              double offboard_timeout_s) {
    robots.reserve(grid.robots);
    for (uint32_t i = 0; i != grid.robots; ++i)
      robots.emplace_back(&initial[4 * i], offboard_timeout_s);
  }

  void step(const xgc_step_ctx &context) {
    grid.round = context.round;
    if (!flights.empty())
      run(flights, context);
    else if (!scouts.empty())
      run(scouts, context);
    else
      run(mecanums, context);
  }

  // The physics owner is the only writer of provider/model state. ROS merely
  // enqueues requests. Every reply reports current generation, even a rejection.
  void providers() {
    xgc_sample_view sample{};
    while (grid.host->next(grid.host->host, kProviderRequestPort, &sample) == XGC_OK) {
      if (sample.len != sizeof(xgc_sim_provider_request_v1) || !sample.data) continue;
      xgc_sim_provider_request_v1 request{};
      std::memcpy(&request, sample.data, sizeof request);
      xgc_sim_provider_result_v1 result{};
      result.stamp = double(grid.time) * 1e-9;
      result.request_id = request.request_id;
      result.robot_index = request.robot_index;
      result.reason = 2;
      if (request.robot_index < flights.size() && std::isfinite(request.stamp)) {
        auto &robot = flights[request.robot_index];
        bool accepted = false;
        if (request.action == 0) accepted = true; // Observe never starts/reset.
        else if (request.action == 1) {
          if (robot.provider_enabled) {
            accepted = request.generation == robot.generation ||
                       (robot.generation > 0 && request.generation == robot.generation - 1);
          } else if (request.generation == robot.generation && robot.generation != UINT64_MAX) {
            robot.model.resetToInitial();
            robot.commands.pending.clear();
            robot.fcu_mode = "POSCTL";
            ++robot.generation;
            robot.provider_enabled = robot.ever_started = true;
            accepted = true;
          }
        } else if (request.action == 2 && request.generation == robot.generation) {
          robot.provider_enabled = false;
          robot.commands.pending.clear();
          accepted = true;
        }
        result.generation = robot.generation;
        result.enabled = robot.provider_enabled;
        result.accepted = accepted;
        result.reason = accepted ? 0 : request.action <= 2 ? 1 : 2;
      }
      grid.publish_batch(kProviderResultPort, result);
    }
  }

  template <class Robot>
  void run(std::vector<Robot> &robots, const xgc_step_ctx &context) {
    for (uint32_t i = 0; i != robots.size(); ++i) {
      auto &commands = robots[i].commands;
      commands.drain(grid, i, Setpoint, sizeof(xgc_position_target_v1));
      commands.drain(grid, i, VelocityCommand, sizeof(xgc_twist_v1));
      commands.drain(grid, i, AttitudeCommand, sizeof(xgc_attitude_target_v2));
      commands.drain(grid, i, FcuRequest, sizeof(xgc_fcu_request_v2));
      if constexpr (std::is_same_v<Robot, FlightRobot>) {
        // Tag arrival before lifecycle processing: reset drops both pending
        // future inputs and every old ingress drained in this same boundary.
        if (!robots[i].provider_enabled) commands.pending.clear();
        else for (auto &input : commands.pending)
          if (input.generation == 0) input.generation = robots[i].generation;
      }
    }
    if constexpr (std::is_same_v<Robot, FlightRobot>) providers();
    // One read of the host time sets one target grid point for the batch.
    // A centralized host freezes one target for every module in this round.
    // Distributed/HIL placement may instead catch up to its local synchronized
    // clock. Both retain the same epoch/grid and actual command arrival times.
    const int64_t requested_time = grid.advance_on_round ? context.round_start : context.now;
    if (requested_time - grid.time >= grid.step_ns) {
      const int64_t target =
          grid.time + (requested_time - grid.time) / grid.step_ns * grid.step_ns;
      for (uint32_t i = 0; i != robots.size(); ++i)
        advance(grid, i, robots[i], target);
      grid.time = target;
    }
    if (context.now >= grid.epoch && grid.time >= grid.next_output) {
      for (uint32_t i = 0; i != robots.size(); ++i)
        robots[i].output(grid, i, context.round);
      if (!flights.empty()) {
        xgc_fcu_extended_state_v1 state{};
        state.stamp = double(grid.time) * 1e-9;
        state.count = static_cast<uint32_t>(flights.size());
        for (uint32_t i = 0; i != state.count; ++i) {
          const auto &model = flights[i].model;
          state.landed_state[i] = model.landed()
              ? (model.armed() && model.state().velocity.z() > 0.0 ? 3 : 1)
              : (model.mode() == xgc_lightweight::FlightMode::Land ? 4 : 2);
        }
        grid.publish_batch(kFcuExtendedStatePort, state);
      }
      grid.next_output =
          grid.epoch +
          ((grid.time - grid.epoch) / grid.output_ns + 1) * grid.output_ns;
    }
  }
};

template <class F> xgc_status guarded(Plant *self, F &&action) {
  try {
    action();
    return XGC_OK;
  } catch (const std::exception &e) {
    self->grid.host->log(self->grid.host->host, XGC_LOG_ERROR, e.what());
    return XGC_ERR;
  }
}
void *create(const xgc_host_api *host) {
  try {
    return new Plant{host};
  } catch (...) {
    return nullptr;
  }
}
xgc_status configure(void *p, const char *text) {
  auto *self = static_cast<Plant *>(p);
  return guarded(self, [&] { self->configure(text); });
}
xgc_status activate(void *) { return XGC_OK; }
xgc_status step(void *p, const xgc_step_ctx *ctx) {
  auto *self = static_cast<Plant *>(p);
  return guarded(self, [&] { self->step(*ctx); });
}
xgc_status deactivate(void *) { return XGC_OK; }
void destroy(void *p) { delete static_cast<Plant *>(p); }
const char *state(void *) { return "Simulating"; }
const xgc_plugin_vtbl vtbl{create,     configure, activate, step,
                           deactivate, destroy,   state};

struct PortKind {
  const char *name;
  xgc_port_dir dir;
  const char *schema;
  xgc_qos qos;
};
// Block 0 keeps the single-robot ports, names and indices unchanged.
constexpr PortKind kPortKinds[kPortsPerRobot] = {
    {"setpoint", XGC_PORT_IN_OPTIONAL, "xgc.position_target/1",
     XGC_QOS_CONTROL},
    {"cmd_vel", XGC_PORT_IN_OPTIONAL, "xgc.twist/1", XGC_QOS_CONTROL},
    {"fcu_request", XGC_PORT_IN_OPTIONAL, "xgc.fcu_request/2", XGC_QOS_EVENT},
    {"pose", XGC_PORT_OUT, "xgc.pose/1", XGC_QOS_STATE},
    {"velocity", XGC_PORT_OUT, "xgc.twist/1", XGC_QOS_STATE},
    {"imu", XGC_PORT_OUT_OPTIONAL, "xgc.imu/1", XGC_QOS_STATE},
    {"fcu_state", XGC_PORT_OUT_OPTIONAL, "xgc.fcu_state/1", XGC_QOS_STATE},
    {"paired_state", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.paired_state/1",
     XGC_QOS_STATE},
    {"attitude_command", XGC_PORT_IN_OPTIONAL, "xgc.attitude_target/2",
     XGC_QOS_CONTROL},
    {"attitude_target", XGC_PORT_OUT_OPTIONAL, "xgc.attitude_target/2",
     XGC_QOS_STATE},
};

struct Descriptor {
  std::array<std::string, XGC_RT_MAX_PORTS> names;
  std::array<xgc_port_decl, XGC_RT_MAX_PORTS> ports{};
  xgc_plugin_descriptor descriptor{};

  Descriptor() {
    for (uint32_t robot = 0; robot != kMaxRobots; ++robot)
      for (uint32_t kind = 0; kind != kPortsPerRobot; ++kind) {
        const auto i = robot * kPortsPerRobot + kind;
        const auto &port = kPortKinds[kind];
        names[i] = port.name;
        if (robot != 0)
          names[i] += "_" + std::to_string(robot);
        // Only robot 0 must be bound; later blocks serve a batch.
        const auto dir = robot != 0 && port.dir == XGC_PORT_OUT
                             ? XGC_PORT_OUT_OPTIONAL
                             : port.dir;
        ports[i] = {names[i].c_str(), dir, port.schema, port.qos};
      }
    names[kFcuResultPort] = "fcu_result";
    ports[kFcuResultPort] = {names[kFcuResultPort].c_str(), XGC_PORT_OUT_OPTIONAL, "xgc.fcu_result/1", XGC_QOS_EVENT};
    names[kFcuExtendedStatePort] = "fcu_extended_state";
    ports[kFcuExtendedStatePort] = {names[kFcuExtendedStatePort].c_str(), XGC_PORT_OUT_OPTIONAL, "xgc.fcu_extended_state/1", XGC_QOS_STATE};
    names[kProviderRequestPort] = "provider_request";
    ports[kProviderRequestPort] = {names[kProviderRequestPort].c_str(), XGC_PORT_IN_OPTIONAL, "xgc.sim_provider_request/1", XGC_QOS_EVENT};
    names[kProviderResultPort] = "provider_result";
    ports[kProviderResultPort] = {names[kProviderResultPort].c_str(), XGC_PORT_OUT_OPTIONAL, "xgc.sim_provider_result/1", XGC_QOS_EVENT};
    descriptor = {XGC_RT_ABI_VERSION, kProviderResultPort + 1, "lightweight-vehicle",
                  "0.3.0",           ports.data(),     &vtbl};
  }
};
} // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor *
xgc_rt_plugin_v1() {
  static const Descriptor descriptor;
  return &descriptor.descriptor;
}
