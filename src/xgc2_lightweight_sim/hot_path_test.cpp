// The per-step path of FlightModel must not look FCU parameters up by name,
// and the values it reads instead must be the ones get() returns.
//
// 1. Lookup count: FlightControllerParameters::get() is a linear string scan.
//    Its count must not move across any step, in any mode, in any command
//    channel (disarmed, armed hold, offboard PVA, stream loss, raw attitude,
//    raw rate, land). Construction and reset may read parameters.
// 2. Hoisted values: each value the step path reads is the float get()
//    returns for its parameter, with default and with overridden parameters.
//    Step arithmetic is untouched, so the model is the same bit for bit.
//    (A recorded-trajectory comparison is not portable: the PX4 control code
//    is float32 and event times such as touchdown shift by a step with the
//    compiler, so the by-name and hoisted builds were compared bit for bit
//    with one toolchain by the change author instead.)
#include "vehicle_model.hpp"

#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace xgc_lightweight;

// Release builds define NDEBUG; these checks (some with side effects) must run.
#define REQUIRE(expr)                                                      \
  do {                                                                     \
    if (!(expr)) {                                                         \
      std::fprintf(stderr, "%s:%d: requirement failed: %s\n", __FILE__,   \
                   __LINE__, #expr);                                       \
      std::abort();                                                        \
    }                                                                      \
  } while (0)

namespace {

uint64_t lookups() { return FlightControllerParameters::lookup_count(); }

void checkpoint(std::vector<double> &out, const FlightModel &model) {
  for (int i = 0; i != 3; ++i) out.push_back(model.state().position[i]);
  for (int i = 0; i != 3; ++i) out.push_back(model.state().velocity[i]);
  for (int i = 0; i != 4; ++i) out.push_back(model.orientation().coeffs()[i]);
  for (int i = 0; i != 3; ++i) out.push_back(model.angular_velocity_body()[i]);
  for (int i = 0; i != 4; ++i) out.push_back(model.rotor_speed()[i]);
  out.push_back(model.control_output().normalized_thrust);
}

// Every step below runs under the same assertion: no parameter lookup.
FlightEvent step(FlightModel &model, double dt) {
  const uint64_t before = lookups();
  const FlightEvent event = model.step(dt);
  REQUIRE(lookups() == before); // FlightModel::step looked a parameter up by name
  return event;
}

MaskedPva pva(const Eigen::Vector3d &position) {
  MaskedPva value;
  value.position = position;
  value.receiver_valid = true;
  return value;
}

std::vector<double> scenario(const FlightControllerParameters &parameters) {
  std::vector<double> out;
  FlightModel model(Eigen::Vector3d(0, 0, 0.15), 0.2, parameters, 0.15);

  // Disarmed on the ground, then armed hold (controller.disarmed / ground gate).
  for (int i = 0; i != 100; ++i) step(model, 0.001);
  REQUIRE(model.request_arm(true));
  for (int i = 0; i != 100; ++i) step(model, 0.001);
  checkpoint(out, model);

  // Offboard position stream, 50 Hz, with an update every 10 ms round.
  REQUIRE(model.setpoint(pva({0.4, -0.3, 1.6})));
  REQUIRE(model.request_mode(FlightMode::Offboard));
  for (int round = 0; round != 300; ++round) {
    if (round % 2 == 0)
      REQUIRE(model.setpoint(pva({0.4, -0.3, 1.6 + 0.05 * std::sin(0.05 * round)})));
    step(model, 0.01);
    if (round % 100 == 99) checkpoint(out, model);
  }

  // Stream loss: COM_OF_LOSS_T elapses without a setpoint, mode falls to hold.
  FlightEvent event = FlightEvent::None;
  for (int i = 0; i != 3000 && event != FlightEvent::OffboardLost; ++i)
    event = step(model, 0.001);
  REQUIRE(event == FlightEvent::OffboardLost);
  checkpoint(out, model);

  // Raw attitude (type 7) and raw rate (type 128) channels.
  FlightAttitudeSetpoint attitude;
  attitude.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));
  attitude.thrust = 0.6;
  attitude.type_mask = 7;
  REQUIRE(model.setpoint(pva({0.4, -0.3, 1.6})));
  REQUIRE(model.request_mode(FlightMode::Offboard));
  REQUIRE(model.attitude_setpoint(attitude));
  for (int i = 0; i != 200; ++i) {
    REQUIRE(model.attitude_setpoint(attitude));
    step(model, 0.001);
  }
  checkpoint(out, model);
  attitude.type_mask = 128;
  attitude.body_rate = Eigen::Vector3d(0.05, -0.02, 0.1);
  for (int i = 0; i != 200; ++i) {
    REQUIRE(model.attitude_setpoint(attitude));
    step(model, 0.001);
  }
  checkpoint(out, model);

  // Land (MPC_LAND_SPEED, MPC_TILTMAX_LND) until touchdown disarms.
  REQUIRE(model.setpoint(pva({0.4, -0.3, 1.6})));
  REQUIRE(model.request_mode(FlightMode::Land));
  event = FlightEvent::None;
  for (int i = 0; i != 30000 && event != FlightEvent::Landed; ++i) {
    event = step(model, 0.001);
    if (i % 1000 == 999) checkpoint(out, model);
  }
  REQUIRE(event == FlightEvent::Landed);
  checkpoint(out, model);
  return out;
}

FlightControllerParameters overridden() {
  FlightControllerParameters parameters;
  REQUIRE(parameters.set("COM_OF_LOSS_T", 0.25));
  REQUIRE(parameters.set("THR_MDL_FAC", 0.3));
  REQUIRE(parameters.set("MOT_SLEW_MAX", 1.5));
  REQUIRE(parameters.set("MPC_TILTMAX_AIR", 35.0));
  REQUIRE(parameters.set("MPC_TILTMAX_LND", 15.0));
  REQUIRE(parameters.set("MPC_LAND_SPEED", 0.9));
  return parameters;
}

} // namespace

void hoisted_values_are_the_looked_up_values(const FlightControllerParameters &parameters) {
  FlightModel model(Eigen::Vector3d(0, 0, 0.15), 0.0, parameters, 0.15);
  const auto hot = model.controller_hot_parameters();
  REQUIRE(hot.tilt_max_air == parameters.get("MPC_TILTMAX_AIR"));
  REQUIRE(hot.tilt_max_land == parameters.get("MPC_TILTMAX_LND"));
  REQUIRE(hot.thrust_model_factor == parameters.get("THR_MDL_FAC"));
  REQUIRE(hot.motor_slew_max == parameters.get("MOT_SLEW_MAX"));
  REQUIRE(model.offboard_loss_timeout() == parameters.get("COM_OF_LOSS_T"));
  REQUIRE(model.land_speed() == parameters.get("MPC_LAND_SPEED"));
}

int main() {
  const FlightControllerParameters defaults;
  hoisted_values_are_the_looked_up_values(defaults);
  hoisted_values_are_the_looked_up_values(overridden());
  // The overridden values are not the defaults, so the check is not vacuous.
  REQUIRE(overridden().get("THR_MDL_FAC") != defaults.get("THR_MDL_FAC"));
  REQUIRE(overridden().get("COM_OF_LOSS_T") != defaults.get("COM_OF_LOSS_T"));
  REQUIRE(overridden().get("MPC_TILTMAX_LND") != defaults.get("MPC_TILTMAX_LND"));

  // Every mode below runs under the no-lookup requirement of step().
  const auto from_defaults = scenario(defaults);
  const auto from_overrides = scenario(overridden());
  REQUIRE(!from_defaults.empty() && !from_overrides.empty());

  // Reset reconstructs the model (a lifecycle operation): it may read
  // parameters, and the new instance steps without lookups again.
  FlightModel model(Eigen::Vector3d(0, 0, 0.15), 0.0, overridden(), 0.15);
  REQUIRE(model.request_arm(true));
  model.resetToInitial();
  const uint64_t after_reset = lookups();
  model.request_arm(true);
  for (int i = 0; i != 50; ++i) model.step(0.001);
  REQUIRE(lookups() == after_reset);
  std::puts("hot_path_test passed");
}
