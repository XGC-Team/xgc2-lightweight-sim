#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "flight_controller.hpp"
#include "ground_contact.hpp"

#include <xgc2_math/control/delayed_planar_velocity.hpp>
#include <xgc2_math/geometry/kinematics.hpp>

// The numerical plant has no ROS, transport or scheduler. Native and ROS
// boundaries supply commands and elapsed model time; flight embeds an FCU loop.
namespace xgc_lightweight {

using FlightSetpoint = MaskedPva;

struct FlightAttitudeSetpoint {
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
  double thrust{0.0};
  uint32_t type_mask{128};
};

enum class FlightMode { Hold, Offboard, Land };
enum class FlightEvent { None, OffboardLost, Landed };

// FCU service/stream facade around the fixed PX4 pure control classes.
// No EKF, GPS/RC/preflight, uORB, full Commander or firmware is simulated.
class FlightModel {
public:
  static constexpr double kLandedHeight = 0.01;
  static constexpr double kMaxControlStep = 0.001;

  explicit FlightModel(const Eigen::Vector3d &position, double yaw = 0.0,
                       const FlightControllerParameters &parameters = {},
                       double world_ground_z = 0.0)
      : body_(position, yaw), controller_(body_.parameters(), parameters),
        ground_z_(world_ground_z), initial_position_(position), initial_yaw_(yaw),
        offboard_loss_timeout_(controller_.parameters().get("COM_OF_LOSS_T")),
        land_speed_(controller_.parameters().get("MPC_LAND_SPEED")) {
    if (!std::isfinite(world_ground_z))
      throw std::invalid_argument("flight model: invalid world ground");
    constrain_ground(0.0);
    refresh_state();
  }
  // World-thread lifecycle operation: one slot only. This recreates the
  // noncopyable mixer and clears physical/control/request histories; no Host
  // world clock or other robot is represented or modified by this object.
  void resetToInitial() {
    FlightModel fresh(initial_position_, initial_yaw_, parameters(), ground_z_);
    *this = std::move(fresh);
  }
  bool setpoint(const MaskedPva &value) {
    const auto any = [](const Eigen::Vector3d &v) {
      return std::isfinite(v.x()) || std::isfinite(v.y()) || std::isfinite(v.z());
    };
    if (!value.receiver_valid ||
        !(any(value.position) || any(value.velocity) || any(value.acceleration)))
      return false;
    // The receiver heartbeat is independent of PositionControl's XY/per-axis
    // validity. An accepted incomplete setpoint is still a fresh stream.
    setpoint_ = value;
    attitude_control_ = false;
    received_since_step_ = true;
    return true;
  }
  bool attitude_setpoint(const FlightAttitudeSetpoint &value) {
    if (value.type_mask != 0 && value.type_mask != 7 && value.type_mask != 128)
      return false;
    if (!std::isfinite(value.thrust) ||
        (value.type_mask != 7 && !value.body_rate.allFinite()))
      return false;
    if (value.type_mask != 128) {
      const double norm = value.q.norm();
      if (!value.q.coeffs().allFinite() || !std::isfinite(norm) || norm < 1e-12)
        return false;
    }
    attitude_setpoint_ = value;
    if (value.type_mask != 128)
      attitude_setpoint_.q.normalize();
    attitude_control_ = true;
    received_since_step_ = true;
    return true;
  }
  bool request_arm(bool value) {
    if (!value && armed_ && !landed())
      return false;
    if (value != armed_)
      controller_.reset();
    armed_ = value;
    return true;
  }
  bool request_mode(FlightMode value) {
    if (value == FlightMode::Offboard && !stream_available())
      return false;
    mode_ = value;
    return true;
  }
  bool armed() const { return armed_; }
  FlightMode mode() const { return mode_; }
  bool landed() const { return state_.position.z() <= ground_z_ + kLandedHeight; }
  const xgc2_math::TranslationalState &state() const { return state_; }
  const Eigen::Vector3d &acceleration() const { return acceleration_; }
  const Eigen::Quaterniond &orientation() const { return body_.state().orientation; }
  Eigen::Vector3d angular_velocity_body() const { return body_.state().angular_velocity; }
  const Eigen::Vector4d &rotor_speed() const { return body_.state().rotor_speed; }
  Eigen::Vector3d specific_force_body() const {
    return orientation().conjugate() *
           (acceleration_ - Eigen::Vector3d(0.0, 0.0, -body_.parameters().gravity));
  }
  double yaw() const {
    const auto r = orientation().toRotationMatrix();
    return std::atan2(r(1, 0), r(0, 0));
  }
  double yaw_rate() const { return yaw_rate_; }
  const FlightControlOutput &control_output() const { return control_output_; }
  const FlightControllerParameters &parameters() const { return controller_.parameters(); }

  // Inactive provider physics: keep the last motor target, without running
  // FCU control/integrals/stream hysteresis or changing armed/mode state.
  // Next world-thread start reconstructs this slot via resetToInitial().
  void stepPhysicsOnly(double dt) {
    if (!std::isfinite(dt) || dt < 0.0 ||
        dt / kMaxControlStep > std::numeric_limits<int>::max())
      throw std::invalid_argument("flight model: invalid physics dt");
    if (dt == 0.0)
      return;
    const double yaw_before = yaw();
    const int count = static_cast<int>(std::ceil(dt / kMaxControlStep));
    const double h = dt / count;
    for (int i = 0; i != count; ++i) {
      body_.step(control_output_.allocation.target_rotor_speed, h);
      constrain_ground(h);
      refresh_state();
    }
    yaw_rate_ = xgc2_math::normalizeAngle(yaw() - yaw_before) / dt;
  }

  FlightEvent step(double dt) {
    if (!std::isfinite(dt) || dt < 0.0 ||
        dt / kMaxControlStep > std::numeric_limits<int>::max())
      throw std::invalid_argument("flight model: invalid dt");
    if (dt == 0.0)
      return FlightEvent::None;
    FlightEvent event = FlightEvent::None;
    const double yaw_before = yaw();
    const int count = static_cast<int>(std::ceil(dt / kMaxControlStep));
    const double h = dt / count;
    for (int i = 0; i != count; ++i) {
      // Commander v1.12.3 sets the true->false offboard-available hysteresis
      // directly to COM_OF_LOSS_T. Valid first input becomes available at once;
      // there is no one-second warmup or a second invented stream timeout.
      update_stream(h);
      if (mode_ == FlightMode::Offboard && !offboard_available_) {
        mode_ = FlightMode::Hold;
        event = FlightEvent::OffboardLost;
      }
      const auto feedback = current_feedback();
      if (!armed_) {
        control_output_ = controller_.disarmed(h);
      } else if (mode_ == FlightMode::Hold && landed()) {
        // Keep armed idle while waiting on the initial ground for a command.
        // This is the narrow service facade's ground gate, not a simulated
        // Commander/Takeoff state machine. Attitude and rate functions remain
        // the fixed source functions; no physical attitude is reset.
        control_output_ = controller_.command_attitude(
            feedback, orientation(), 0.0, Eigen::Vector3d::Zero(), 7, h, true);
      } else if (mode_ == FlightMode::Offboard && attitude_control_) {
        const auto &raw = attitude_setpoint_;
        if (raw.type_mask == 128) {
          control_output_ = controller_.command_rate(feedback, raw.body_rate,
                                                     raw.thrust, h, landed());
          control_output_.desired_orientation = raw.q; // ignored target payload
        } else {
          control_output_ = controller_.command_attitude(
              feedback, raw.q, raw.thrust, raw.body_rate, raw.type_mask, h, landed());
        }
      } else {
        MaskedPva command;
        if (mode_ == FlightMode::Offboard) {
          command = setpoint_;
        } else {
          command.velocity.setZero();
          if (mode_ == FlightMode::Land)
            command.velocity.z() = -land_speed_;
        }
        control_output_ = controller_.command_pva(feedback, command, h,
                                                  landed(), mode_ == FlightMode::Land);
      }
      body_.step(control_output_.allocation.target_rotor_speed, h);
      if (constrain_ground(h)) {
        if (armed_ && mode_ == FlightMode::Land) {
          armed_ = false;
          controller_.reset();
          event = FlightEvent::Landed;
        }
      }
      refresh_state();
    }
    yaw_rate_ = xgc2_math::normalizeAngle(yaw() - yaw_before) / dt;
    return event;
  }

private:
  bool constrain_ground(double dt) {
    const Eigen::Vector3d omega_before = body_.state().angular_velocity;
    const auto result = resolveFlatGroundContact(body_, ground_z_, dt);
    acceleration_ = result.base_acceleration_world;
    angular_acceleration_body_ = body_.angular_acceleration_body();
    if (result.valid && dt > 0.0)
      angular_acceleration_body_ += (body_.state().angular_velocity - omega_before) / dt;
    return result.valid && result.constrained;
  }
  FlightFeedback current_feedback() const {
    return {state_.position, state_.velocity, acceleration_, orientation(),
            angular_velocity_body(), angular_acceleration_body_};
  }
  void refresh_state() {
    state_.position = body_.base_position();
    state_.velocity = body_.base_velocity();
  }
  bool stream_available() const {
    return received_since_step_ || offboard_available_;
  }
  void update_stream(double dt) {
    if (received_since_step_) {
      // Same true->false-only hysteresis as Commander::offboard_control_update.
      // No warmup on first valid receiver event. Loss delay begins at the
      // first FCU iteration without a new valid event, not at its last stamp.
      offboard_available_ = true;
      received_since_step_ = false;
      loss_pending_ = false;
      loss_age_ = 0.0;
    } else if (offboard_available_) {
      if (!loss_pending_) {
        loss_pending_ = true;
        loss_age_ = 0.0;
      }
      if (loss_age_ >= offboard_loss_timeout_ - 1e-9)
        offboard_available_ = false;
      loss_age_ += dt;
    }
  }

  RigidBodyModel body_;
  FlightController controller_;
  xgc2_math::TranslationalState state_;
  double ground_z_;
  Eigen::Vector3d initial_position_;
  double initial_yaw_;
  // Read once from the immutable parameter block; the step loop never looks
  // parameters up by name.
  float offboard_loss_timeout_, land_speed_;
  MaskedPva setpoint_;
  FlightAttitudeSetpoint attitude_setpoint_;
  FlightControlOutput control_output_;
  Eigen::Vector3d acceleration_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_body_{Eigen::Vector3d::Zero()};
  double loss_age_{0.0};
  double yaw_rate_{0.0};
  FlightMode mode_{FlightMode::Hold};
  bool received_since_step_{false};
  bool offboard_available_{false};
  bool loss_pending_{false};
  bool attitude_control_{false};
  bool armed_{false};
};

class ScoutModel {
public:
  explicit ScoutModel(
      const xgc2_math::Pose2 &pose,
      const xgc2_math::DelayedPlanarVelocityParameters &response = {})
      : pose_(pose), response_(response) {}

  void command(double time, double forward, double yaw_rate) {
    // Same defaults as the accepted Gazebo unicycle plant; not wheel physics.
    response_.command(time, {std::clamp(forward, -1.5, 1.5),
                             std::clamp(yaw_rate, -1.0, 1.0)});
  }
  void advance(double end) {
    const double dt = end - response_.time();
    const auto middle = response_.advance(response_.time() + 0.5 * dt);
    pose_ = xgc2_math::stepBodyVelocity(pose_, {middle.linear_m_s, 0.0},
                                        middle.yaw_rad_s, dt);
    response_.advance(end);
  }
  const xgc2_math::Pose2 &pose() const { return pose_; }
  xgc2_math::PlanarVelocity velocity() const { return response_.velocity(); }

private:
  xgc2_math::Pose2 pose_;
  xgc2_math::DelayedPlanarVelocity response_;
};

class MecanumModel {
public:
  explicit MecanumModel(const xgc2_math::Pose2 &pose) : pose_(pose) {}

  void command(double forward, double left, double yaw_rate) {
    // Defaults of ugv_sim_single.launch, not unverified MCU/TEB limits.
    body_velocity_ = {std::clamp(forward, -1.5, 1.5),
                      std::clamp(left, -1.5, 1.5)};
    yaw_rate_ = std::clamp(yaw_rate, -1.5707963267948966, 1.5707963267948966);
  }
  void step(double dt) {
    pose_ = xgc2_math::stepBodyVelocity(pose_, body_velocity_, yaw_rate_, dt);
  }
  const xgc2_math::Pose2 &pose() const { return pose_; }
  const Eigen::Vector2d &body_velocity() const { return body_velocity_; }
  double yaw_rate() const { return yaw_rate_; }

private:
  xgc2_math::Pose2 pose_;
  Eigen::Vector2d body_velocity_{Eigen::Vector2d::Zero()};
  double yaw_rate_{0.0};
};

} // namespace xgc_lightweight
