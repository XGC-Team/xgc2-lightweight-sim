#pragma once

#include "rigid_body.hpp"
#include <algorithm>
#include <cmath>

namespace xgc_lightweight {

struct GroundContactResult {
  bool valid{true};
  bool constrained{false};
  double normal_impulse{0.0}; // N s, unilateral, perfectly inelastic
  Eigen::Vector3d base_acceleration_world{Eigen::Vector3d::Zero()};
};

// One frictionless world-horizontal plane at the body-origin measurement point.
// world_ground_z is environment data, never derived from a spawn/reset pose.
// The point lies at r=-body_origin_to_com relative to COM. A normal impulse
// must change BOTH COM velocity and body angular momentum:
// K=1/m+(r x n_body)' J^-1 (r x n_body), lambda=-v_point.normal/K.
// Its kinetic-energy change is -.5*v_normal^2/K <= 0. A COM-only correction
// lacks that property when a tilted/off-centre body is rotating into the plane.
// No friction, attitude snapping, restitution or multi-point contact engine.
inline GroundContactResult resolveFlatGroundContact(RigidBodyModel &body,
                                                    double world_ground_z,
                                                    double dt = 0.0) {
  GroundContactResult result;
  result.base_acceleration_world = body.base_acceleration();
  const Eigen::Vector3d position = body.base_position();
  const Eigen::Vector3d velocity = body.base_velocity();
  if (!std::isfinite(world_ground_z) || !std::isfinite(dt) || dt < 0.0 ||
      !position.allFinite() || !velocity.allFinite()) {
    result.valid = false;
    return result;
  }
  if (position.z() > world_ground_z)
    return result;
  const auto &p = body.parameters();
  auto state = body.state();
  const Eigen::Vector3d normal_world = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d normal_body = state.orientation.conjugate() * normal_world;
  const Eigen::Vector3d r = -p.body_origin_to_com;
  const Eigen::Vector3d lever = r.cross(normal_body);
  const Eigen::Vector3d inverse_j_lever = p.inertia.llt().solve(lever);
  const double inverse_effective_mass = 1.0 / p.mass + lever.dot(inverse_j_lever);
  if (!std::isfinite(inverse_effective_mass) || inverse_effective_mass <= 0.0) {
    result.valid = false;
    return result;
  }
  state.position.z() += world_ground_z - position.z();
  result.normal_impulse = std::max(0.0, -velocity.z() / inverse_effective_mass);
  state.velocity += normal_world * (result.normal_impulse / p.mass);
  state.angular_velocity += inverse_j_lever * result.normal_impulse;
  if (!state.position.allFinite() || !state.velocity.allFinite() ||
      !state.angular_velocity.allFinite()) {
    result.valid = false;
    return result;
  }
  body.set_state(state);
  result.constrained = true;
  result.base_acceleration_world = body.base_acceleration();
  if (dt > 0.0) {
    // Export the point's observed impulse acceleration, including angular
    // impulse. At settled contact this cancels free-fall acceleration, making
    // specific force +g; at impact it preserves the finite-step impact pulse.
    result.base_acceleration_world += (body.base_velocity() - velocity) / dt;
  } else if (velocity.z() <= 0.0 && result.base_acceleration_world.z() < 0.0) {
    // Initial/resting observation without an elapsed integration interval.
    const Eigen::Vector3d response = normal_world / p.mass + state.orientation *
        inverse_j_lever.cross(r);
    result.base_acceleration_world += response *
        (-result.base_acceleration_world.z() / inverse_effective_mass);
  }
  return result;
}

} // namespace xgc_lightweight
