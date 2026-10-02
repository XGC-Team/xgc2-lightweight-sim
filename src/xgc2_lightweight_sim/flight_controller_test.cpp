#include "flight_controller.hpp"

#include <cassert>
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace xgc_lightweight {
// The supplied original trace is NED/FRD and tests the four components
// separately, with explicit synthetic gains (not runtime defaults). Replay
// those histories on the actual component instances owned by the facade.
// Full ENU/FLU -> Rate -> Mixer pipeline comparisons follow below separately.
struct FlightControllerReplay {
  static matrix::Vector3f v(const std::vector<float> &in, size_t offset) {
    return {in[offset], in[offset + 1], in[offset + 2]};
  }
  static void append(std::vector<float> &out, const matrix::Vector3f &value) {
    for (int i = 0; i != 3; ++i)
      out.push_back(value(i));
  }
  static matrix::Vector3f body_roundtrip(const matrix::Vector3f &value) {
    return Px4Frame::body(Px4Frame::body(value));
  }
  static matrix::Vector3f world_roundtrip(const matrix::Vector3f &value) {
    return Px4Frame::world(Px4Frame::world(value));
  }
  static void rate_config(FlightController &c, const std::vector<float> &in) {
    c.rate_.setGains(v(in, 0), v(in, 3), v(in, 6));
    c.rate_.setIntegratorLimit(v(in, 9));
    c.rate_.setFeedForwardGain(v(in, 12));
  }
  static void position_config(FlightController &c, const std::vector<float> &in) {
    c.position_.setPositionGains(v(in, 0));
    c.position_.setVelocityGains(v(in, 3), v(in, 6), v(in, 9));
    c.position_.setVelocityLimits(in[12], in[13], in[14]);
    c.position_.setThrustLimits(in[15], in[16]);
    c.position_.setTiltLimit(in[17]);
    c.position_.setHoverThrust(in[18]);
  }
  static void integral(FlightController &c, std::vector<float> &out) {
    rate_ctrl_status_s status{};
    c.rate_.getRateControlStatus(status);
    append(out, {status.rollspeed_integ, status.pitchspeed_integ, status.yawspeed_integ});
  }
  static std::vector<float> attitude(FlightController &c, const std::vector<float> &in) {
    c.attitude_.setProportionalGain(v(in, 2), in[0]);
    c.attitude_.setRateLimit(v(in, 5));
    const matrix::Quatf actual(in.data() + 8), desired(in.data() + 12);
    c.attitude_.setAttitudeSetpoint(desired, in[1]);
    std::vector<float> out;
    append(out, body_roundtrip(c.attitude_.update(actual)));
    c.attitude_.setAttitudeSetpoint(
        matrix::Quatf(-desired(0), -desired(1), -desired(2), -desired(3)), in[1]);
    append(out, body_roundtrip(c.attitude_.update(actual)));
    return out;
  }
  static std::vector<float> rate(FlightController &c, const std::vector<float> &in) {
    c.saturation_.value = static_cast<uint16_t>(in[12]);
    c.rate_.setSaturationStatus(c.saturation_);
    if (in[11]) c.rate_.resetIntegral();
    std::vector<float> out;
    append(out, c.rate_.update(body_roundtrip(v(in, 0)), body_roundtrip(v(in, 3)),
                               body_roundtrip(v(in, 6)), in[9], bool(in[10])));
    integral(c, out);
    return out;
  }
  static std::vector<float> position(FlightController &c, const std::vector<float> &in) {
    if (in[1]) c.position_.resetIntegral();
    if (in[2]) c.position_.updateHoverThrust(in[2]); // component fixture only
    PositionControlStates state{};
    state.position = world_roundtrip(v(in, 3));
    state.velocity = world_roundtrip(v(in, 6));
    state.acceleration = world_roundtrip(v(in, 9));
    state.yaw = in[12];
    vehicle_local_position_setpoint_s point{};
    point.x = in[13]; point.y = in[14]; point.z = in[15];
    point.vx = in[16]; point.vy = in[17]; point.vz = in[18];
    world_roundtrip(v(in, 19)).copyTo(point.acceleration);
    point.yaw = in[22]; point.yawspeed = in[23];
    c.position_.setState(state); c.position_.setInputSetpoint(point);
    assert(c.position_.update(in[0]));
    vehicle_local_position_setpoint_s executed{};
    vehicle_attitude_setpoint_s target{};
    c.position_.getLocalPositionSetpoint(executed); c.position_.getAttitudeSetpoint(target);
    std::vector<float> out{executed.vx, executed.vy, executed.vz};
    append(out, world_roundtrip(matrix::Vector3f(executed.acceleration)));
    append(out, world_roundtrip(matrix::Vector3f(executed.thrust)));
    out.push_back(executed.yaw); out.push_back(executed.yawspeed);
    out.insert(out.end(), std::begin(target.q_d), std::end(target.q_d));
    return out;
  }
  static std::vector<float> mixer(FlightController &c, const std::vector<float> &in) {
    c.mixer_->set_airmode(static_cast<Mixer::Airmode>(int(in[0])));
    c.mixer_->set_thrust_factor(in[1]);
    c.saturation_.value = static_cast<uint16_t>(in[6]);
    c.rate_.setSaturationStatus(c.saturation_);
    if (in[5]) c.rate_.resetIntegral();
    const auto torque = c.rate_.update(body_roundtrip(v(in, 10)), body_roundtrip(v(in, 7)),
                                       body_roundtrip(v(in, 13)), in[2], false);
    *c.controls_ = {torque(0), torque(1), torque(2), in[16]};
    if (in[3]) c.mixer_->set_max_delta_out_once(in[4]);
    std::vector<float> out(4);
    assert(c.mixer_->mix(out.data(), 4) == 4);
    for (int i = 0; i != 4; ++i)
      out.push_back(.5f * (out[i] + 1.f));
    append(out, torque); integral(c, out);
    out.push_back(float(c.mixer_->get_saturation_status()));
    return out;
  }
};
}

using namespace xgc_lightweight;
using matrix::Vector3f;

namespace {
struct OriginalOracle {
  FlightControllerParameters params;
  AttitudeControl attitude;
  RateControl rate;
  PositionControl position;
  std::array<float, 4> controls{};
  MultirotorMixer mixer;
  MultirotorMixer::saturation_status saturation{};
  explicit OriginalOracle(const FlightControllerParameters &p)
      : params(p), mixer(&callback, reinterpret_cast<uintptr_t>(this),
                        MultirotorGeometry::QUAD_X) {
    const auto get = [&p](const char *name) { return p.get(name); };
    attitude.setProportionalGain(Vector3f(get("MC_ROLL_P"), get("MC_PITCH_P"), get("MC_YAW_P")),
                                 get("MC_YAW_WEIGHT"));
    attitude.setRateLimit(Vector3f(get("MC_ROLLRATE_MAX"), get("MC_PITCHRATE_MAX"),
                                   get("MC_YAWRATE_MAX")) * float(std::acos(-1.0) / 180.0));
    const Vector3f k(get("MC_ROLLRATE_K"), get("MC_PITCHRATE_K"), get("MC_YAWRATE_K"));
    rate.setGains(k.emult(Vector3f(get("MC_ROLLRATE_P"), get("MC_PITCHRATE_P"), get("MC_YAWRATE_P"))),
                  k.emult(Vector3f(get("MC_ROLLRATE_I"), get("MC_PITCHRATE_I"), get("MC_YAWRATE_I"))),
                  k.emult(Vector3f(get("MC_ROLLRATE_D"), get("MC_PITCHRATE_D"), get("MC_YAWRATE_D"))));
    rate.setFeedForwardGain(Vector3f(get("MC_ROLLRATE_FF"), get("MC_PITCHRATE_FF"), get("MC_YAWRATE_FF")));
    rate.setIntegratorLimit(Vector3f(get("MC_RR_INT_LIM"), get("MC_PR_INT_LIM"), get("MC_YR_INT_LIM")));
    position.setPositionGains(Vector3f(get("MPC_XY_P"), get("MPC_XY_P"), get("MPC_Z_P")));
    position.setVelocityGains(Vector3f(get("MPC_XY_VEL_P_ACC"), get("MPC_XY_VEL_P_ACC"), get("MPC_Z_VEL_P_ACC")),
                              Vector3f(get("MPC_XY_VEL_I_ACC"), get("MPC_XY_VEL_I_ACC"), get("MPC_Z_VEL_I_ACC")),
                              Vector3f(get("MPC_XY_VEL_D_ACC"), get("MPC_XY_VEL_D_ACC"), get("MPC_Z_VEL_D_ACC")));
    position.setVelocityLimits(get("MPC_XY_VEL_MAX"), get("MPC_Z_VEL_MAX_UP"), get("MPC_Z_VEL_MAX_DN"));
    position.setThrustLimits(get("MPC_THR_MIN"), get("MPC_THR_MAX"));
    position.setHoverThrust(get("MPC_THR_HOVER"));
    position.setTiltLimit(get("MPC_TILTMAX_AIR") * float(std::acos(-1.0) / 180.0));
    mixer.set_airmode(static_cast<Mixer::Airmode>(int(get("MC_AIRMODE"))));
    mixer.set_thrust_factor(get("THR_MDL_FAC"));
  }
  static int callback(uintptr_t handle, uint8_t group, uint8_t index, float &value) {
    if (group != 0 || index >= 4)
      return -1;
    value = reinterpret_cast<OriginalOracle *>(handle)->controls[index];
    return 0;
  }
  std::array<float, 4> execute(const FlightFeedback &feedback, const Vector3f &rates,
                              double thrust, double dt, bool landed,
                              const FlightControlOutput &actual) {
    rate.setSaturationStatus(saturation);
    const Vector3f measured(feedback.angular_velocity.x(), -feedback.angular_velocity.y(),
                            -feedback.angular_velocity.z());
    const Vector3f derivative(feedback.angular_acceleration.x(), -feedback.angular_acceleration.y(),
                              -feedback.angular_acceleration.z());
    const Vector3f torque = rate.update(measured, rates, derivative, float(dt), landed);
    assert((actual.normalized_torque_frd - Eigen::Vector3d(torque(0), torque(1), torque(2)))
               .norm() < 1e-7);
    controls = {torque(0), torque(1), torque(2), float(thrust)};
    const float time = params.get("MOT_SLEW_MAX");
    mixer.set_max_delta_out_once(time > 0.f ? 2.f * float(dt) / time : 0.f);
    std::array<float, 4> output{};
    assert(mixer.mix(output.data(), 4) == 4);
    saturation.value = mixer.get_saturation_status();
    assert(saturation.value == actual.allocation.saturation_status);
    double normalized_thrust = 0.0;
    for (int i = 0; i != 4; ++i) {
      const double motor = (output[i] + 1.0) / 2.0;
      assert(std::abs(motor - actual.allocation.motor_commands[i]) < 1e-12);
      assert(std::abs(100.0 + 1000.0 * motor - actual.allocation.target_rotor_speed[i]) < 1e-10);
      const double alpha = params.get("THR_MDL_FAC");
      normalized_thrust += ((1.0 - alpha) * motor + alpha * motor * motor) / 4.0;
    }
    assert(std::abs(normalized_thrust - actual.allocated_normalized_thrust) < 1e-12);
    assert(std::abs(actual.normalized_thrust - std::clamp(thrust, 0.0, 1.0)) < 1e-12);
    assert((actual.commanded_body_rate - actual.desired_body_rate).norm() == 0.0);
    rate_ctrl_status_s expected{};
    rate.getRateControlStatus(expected);
    assert(std::abs(expected.rollspeed_integ) <= params.get("MC_RR_INT_LIM") + 1e-6);
    assert(std::abs(expected.pitchspeed_integ) <= params.get("MC_PR_INT_LIM") + 1e-6);
    assert(std::abs(expected.yawspeed_integ) <= params.get("MC_YR_INT_LIM") + 1e-6);
    return output;
  }
};

FlightFeedback sample(int i) {
  const double t = i * 0.001;
  FlightFeedback f;
  f.position = {0.2 * std::sin(t), -0.3 * std::cos(t), 2.0 + 0.1 * std::sin(t)};
  f.velocity = {0.2 * std::cos(t), 0.3 * std::sin(t), 0.1 * std::cos(t)};
  f.acceleration = {-0.2 * std::sin(t), 0.3 * std::cos(t), -0.1 * std::sin(t)};
  f.orientation = Eigen::AngleAxisd(0.6 * std::sin(t), Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(0.4 * std::cos(t), Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(-0.3 * std::sin(t), Eigen::Vector3d::UnitX());
  f.angular_velocity = {0.4 * std::sin(2 * t), -0.3 * std::cos(3 * t), 0.2 * std::sin(4 * t)};
  f.angular_acceleration = {0.8 * std::cos(2 * t), 0.9 * std::sin(3 * t), 0.8 * std::cos(4 * t)};
  return f;
}

void compare_integral(FlightController &controller, OriginalOracle &oracle) {
  const auto actual = controller.rate_status();
  rate_ctrl_status_s expected{};
  oracle.rate.getRateControlStatus(expected);
  assert(actual.rollspeed_integ == expected.rollspeed_integ);
  assert(actual.pitchspeed_integ == expected.pitchspeed_integ);
  assert(actual.yawspeed_integ == expected.yawspeed_integ);
}

int replay(const char *input_path, const char *output_path) {
  std::ifstream input(input_path);
  std::ofstream output(output_path);
  assert(input && output);
  output << std::setprecision(std::numeric_limits<float>::max_digits10);
  std::string line, previous;
  std::unique_ptr<FlightController> controller;
  std::vector<float> rate_configuration;
  int count = 0;
  const auto write_vector = [&output](const std::vector<float> &values) {
    output << '[';
    for (size_t i = 0; i != values.size(); ++i) {
      if (i) output << ',';
      if (std::isfinite(values[i])) output << values[i];
      else output << (std::isnan(values[i]) ? "\"nan\"" : values[i] > 0 ? "\"inf\"" : "\"-inf\"");
    }
    output << ']';
  };
  while (std::getline(input, line)) {
    std::istringstream tokens(line);
    std::string name, token;
    int step;
    size_t size;
    tokens >> name >> step >> size;
    std::vector<float> data;
    for (size_t i = 0; i != size; ++i) {
      tokens >> token;
      data.push_back(std::stof(token));
    }
    std::string group = name;
    if (group.size() >= 7 && group.substr(group.size() - 7) == "_config")
      group.resize(group.size() - 7);
    if (group != previous) {
      controller = std::make_unique<FlightController>(fs150EquivalentParameters());
      if (name.find("mixer_") == 0)
        FlightControllerReplay::rate_config(*controller, rate_configuration);
      previous = group;
    }
    std::vector<float> result;
    if (name == "rate_history_config") {
      rate_configuration = data;
      FlightControllerReplay::rate_config(*controller, data);
    } else if (name == "position_history_config") {
      FlightControllerReplay::position_config(*controller, data);
    } else if (name.find("attitude_") == 0) {
      result = FlightControllerReplay::attitude(*controller, data);
    } else if (name == "rate_history") {
      result = FlightControllerReplay::rate(*controller, data);
    } else if (name == "position_history") {
      result = FlightControllerReplay::position(*controller, data);
    } else if (name.find("mixer_") == 0) {
      result = FlightControllerReplay::mixer(*controller, data);
    } else {
      throw std::invalid_argument("unknown source oracle case");
    }
    output << "{\"case\":\"" << name << "\",\"step\":" << step << ",\"input\":";
    write_vector(data); output << ",\"output\":"; write_vector(result); output << "}\n";
    ++count;
  }
  std::cout << "replayed " << count << " original component fixture rows on facade-owned kernel instances\n";
  return 0;
}
}

int main(int argc, char **argv) {
  if (argc == 4 && std::string(argv[1]) == "--replay")
    return replay(argv[2], argv[3]);
  assert(argc == 1);
  const auto plant = fs150EquivalentParameters();
  FlightControllerParameters params;
  assert(params.snapshot().size() == 46);
  assert(params.get("MC_ROLLRATE_P") == 0.07000000029802322f);
  assert(params.get("MPC_THR_HOVER") == 0.36000001430511475f);
  assert(!params.set("MPC_USE_HTE", 1)); // no fake subscription / second HTE
  assert(!params.set("hover_thrust_ratio", 0.5));
  assert(!params.set("offboard_timeout_ms", 500));
  assert(!params.set("MC_AIRMODE", 0.5));
  assert(!params.set("THR_MDL_FAC", -0.1));
  assert(!params.set("MC_ROLL_P", std::numeric_limits<double>::infinity()));
  assert(params.set("MC_ROLL_P", 7.0));
  assert(!params.set("MC_ROLL_P", 8.0));
  assert(params.get("MC_ROLL_P") == 7.0f);

  // Independent matrix verification of the single ENU/FLU->NED/FRD bridge,
  // including componentwise NaN preservation and antipodal quaternions.
  Eigen::Matrix3d world;
  world << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  const Eigen::Matrix3d body = Eigen::Vector3d(1, -1, -1).asDiagonal();
  for (int i = 0; i != 1000; ++i) {
    const auto f = sample(i);
    const auto q = Px4Frame::attitude(f.orientation);
    assert((Px4Frame::attitude(q).toRotationMatrix() - f.orientation.toRotationMatrix()).norm() < 1e-6);
    const Eigen::Matrix3d expected = world * f.orientation.toRotationMatrix() * body;
    matrix::Dcmf actual(q);
    for (int row = 0; row != 3; ++row)
      for (int col = 0; col != 3; ++col)
        assert(std::abs(actual(row, col) - expected(row, col)) < 5e-7);
  }
  MaskedPva masked;
  masked.position = {std::numeric_limits<double>::quiet_NaN(), 1.0, 2.0};
  const auto transformed = Px4Frame::setpoint(masked);
  assert(transformed.x == 1.f && std::isnan(transformed.y) && transformed.z == -2.f);

  // Stateful original RateControl -> original MultirotorMixer histories.
  // Saturation flags, THR_MDL_FAC, airmode and once-only slew all participate.
  int histories = 0;
  bool saw_axis_saturation = false;
  bool target_differs_from_allocation = false;
  for (int airmode = 0; airmode != 3; ++airmode)
    for (const double alpha : {0.0, 0.5, 1.0})
      for (const double slew : {0.0, 0.2}) {
        FlightControllerParameters settings;
        assert(settings.set("MC_AIRMODE", airmode));
        assert(settings.set("THR_MDL_FAC", alpha));
        assert(settings.set("MOT_SLEW_MAX", slew));
        FlightController controller(plant, settings);
        OriginalOracle oracle(settings);
        for (int i = 0; i != 600; ++i) {
          if (i == 300) {
            controller.reset();
            oracle.rate.resetIntegral();
            oracle.position.resetIntegral();
          }
          const auto f = sample(i);
          const Eigen::Vector3d requested(6.0 * std::sin(i * 0.02),
                                          5.0 * std::cos(i * 0.03), 4.0 * std::sin(i * 0.04));
          const double thrust = 0.5 + 0.7 * std::cos(i * 0.02);
          const bool landed = i < 10;
          const double dt = i % 2 ? 0.001 : 0.002;
          const auto actual = controller.command_rate(f, requested, thrust, dt, landed);
          assert(actual.type_mask == 128);
          assert((actual.desired_body_rate - requested).norm() == 0.0);
          oracle.execute(f, Vector3f(requested.x(), -requested.y(), -requested.z()),
                         thrust, dt, landed, actual);
          compare_integral(controller, oracle);
          saw_axis_saturation = saw_axis_saturation || (actual.allocation.saturation_status & 0x1f8u);
          target_differs_from_allocation = target_differs_from_allocation ||
              std::abs(actual.normalized_thrust - actual.allocated_normalized_thrust) > 0.05;
          assert((actual.allocation.allocated_wrench -
                  controller.wrench_for_motor_commands(actual.allocation.motor_commands)).norm() < 1e-12);
          ++histories;
        }
      }
  assert(saw_axis_saturation);
  assert(target_differs_from_allocation); // target survives airmode/desaturation

  // Original reduced-attitude/yaw-weight and PositionControl with full PVA,
  // velocity-only and acceleration-only commands, over a shared PID history.
  FlightController controller(plant);
  OriginalOracle oracle(FlightControllerParameters{});
  for (int i = 0; i != 2000; ++i) {
    const auto f = sample(i);
    MaskedPva command;
    command.yaw = 0.3 * std::sin(i * 0.001);
    command.yaw_rate = 0.3 * std::cos(i * 0.001);
    command.receiver_valid = true;
    if (i % 3 == 0)
      command.position = {0.5, -0.1, 2.5};
    if (i % 3 != 2)
      command.velocity = {0.1, -0.2, 0.0};
    command.acceleration = {0.2, -0.1, 0.3};
    const auto actual = controller.command_pva(f, command, 0.001, false);
    assert(actual.position_control_valid && actual.type_mask == 7);
    PositionControlStates states{};
    states.position = {float(f.position.y()), float(f.position.x()), float(-f.position.z())};
    states.velocity = {float(f.velocity.y()), float(f.velocity.x()), float(-f.velocity.z())};
    states.acceleration = {float(f.acceleration.y()), float(f.acceleration.x()), float(-f.acceleration.z())};
    const auto q = Px4Frame::attitude(f.orientation);
    states.yaw = matrix::Eulerf(q).psi();
    oracle.position.setState(states);
    oracle.position.setInputSetpoint(Px4Frame::setpoint(command));
    assert(oracle.position.update(0.001f));
    vehicle_attitude_setpoint_s attitude{};
    vehicle_local_position_setpoint_s executed{};
    oracle.position.getAttitudeSetpoint(attitude);
    oracle.position.getLocalPositionSetpoint(executed);
    const matrix::Quatf desired(attitude.q_d);
    oracle.attitude.setAttitudeSetpoint(desired, attitude.yaw_sp_move_rate);
    const auto rates = oracle.attitude.update(q);
    assert((actual.desired_body_rate - Px4Frame::body(rates)).norm() < 1e-7);
    assert((actual.commanded_body_rate - actual.desired_body_rate).norm() == 0.0);
    assert(actual.desired_body_rate.norm() > 0.01); // PVA correction is published, not zero FF
    assert(actual.desired_orientation.angularDistance(Px4Frame::attitude(desired)) < 1e-7);
    if (i % 3 == 2)
      assert((actual.acceleration_command - command.acceleration).norm() < 1e-6);
    oracle.execute(f, rates, -attitude.thrust_body[2], 0.001, false, actual);
    compare_integral(controller, oracle);
    ++histories;
  }

  // Reduced attitude also handles upside-down attitudes and q/-q correctly.
  for (const double angle : {0.4, 1.5, std::acos(-1.0)}) {
    FlightController actual_control(plant);
    OriginalOracle direct(FlightControllerParameters{});
    auto f = sample(0);
    f.orientation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
    f.angular_velocity.setZero(); f.angular_acceleration.setZero();
    const Eigen::Quaterniond desired(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));
    direct.attitude.setAttitudeSetpoint(Px4Frame::attitude(desired), 0.f);
    const auto rates = direct.attitude.update(Px4Frame::attitude(f.orientation));
    const auto actual = actual_control.command_attitude(f, desired, 0.3, Eigen::Vector3d::Zero(),
                                                        7, 0.001, false);
    assert((actual.desired_body_rate - Px4Frame::body(rates)).norm() < 1e-7);
    direct.execute(f, rates, 0.3, 0.001, false, actual);
    const auto raw = actual_control.command_rate(f, Eigen::Vector3d::Zero(), 0.3, 0.001, true);
    assert(raw.desired_body_rate.norm() == 0.0); // no hidden attitude correction
  }

  // The physical curve's inverse is unique only for an unsaturated feasible
  // motor vector. It is independent of the hover controller's expectation.
  for (const double alpha : {0.0, 0.5, 1.0}) {
    FlightControllerParameters settings;
    assert(settings.set("THR_MDL_FAC", alpha));
    FlightController physical(plant, settings);
    const Eigen::Vector4d motor(0.25, 0.32, 0.35, 0.29);
    const auto wrench = physical.wrench_for_motor_commands(motor);
    const auto controls = physical.normalized_controls_for_wrench(wrench);
    Eigen::Vector4d expected;
    for (int i = 0; i != 4; ++i) {
      const auto &r = FlightController::rotors()[i];
      expected[i] = r.roll_scale * controls[0] + r.pitch_scale * controls[1] +
                    r.yaw_scale * controls[2] + r.thrust_scale * controls[3];
    }
    assert((expected - ((1.0 - alpha) * motor.array() + alpha * motor.array().square()).matrix())
               .norm() < 1e-12);
    auto changed_plant = plant;
    changed_plant.mass *= 2.0;
    FlightController independent(changed_plant, settings);
    assert((independent.wrench_for_motor_commands(motor) - wrench).norm() == 0.0);
  }
  std::cout << "fixed PX4 adapter: " << histories
            << " original control/mixer history steps; frame, parameters, saturation, reset and physical inverse passed\n";
}
