#include "ground_contact.hpp"
#include <iostream>
#include <stdexcept>

using namespace xgc_lightweight;
namespace {
void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}
double energy(const RigidBodyState &s, const RigidBodyParameters &p) {
  return .5 * p.mass * s.velocity.squaredNorm() +
         .5 * s.angular_velocity.dot(p.inertia * s.angular_velocity);
}
}
int main() {
  try {
    auto p = fs150EquivalentParameters();
    p.body_origin_to_com = {.2, 0, .3}; // deliberately visible off-centre lever
    RigidBodyModel tilted({0, 0, 0}, 0, p);
    auto s = tilted.state();
    s.orientation = Eigen::AngleAxisd(.6, Eigen::Vector3d::UnitY());
    s.position = s.orientation * p.body_origin_to_com + Eigen::Vector3d(0, 0, -.02);
    s.angular_velocity = {0, -20, 0};
    tilted.set_state(s);
    const double before = energy(s, p);
    const double vn = tilted.base_velocity().z();
    require(vn < 0, "negative control must rotate the point into the plane");
    auto old = s;
    old.velocity.z() -= vn; // the previous COM-only clamp, no angular impulse
    const double wrong = energy(old, p);
    require(wrong > before, "negative control must expose energy creation");
    const auto contact = resolveFlatGroundContact(tilted, 0, .001);
    const double after = energy(tilted.state(), p);
    require(contact.valid && contact.constrained && contact.normal_impulse > 0,
            "off-centre contact impulse");
    require(std::abs(tilted.base_position().z()) < 1e-12, "base plane, not COM plane");
    require(std::abs(tilted.base_velocity().z()) < 1e-12, "point normal velocity zero");
    require((tilted.state().angular_velocity - s.angular_velocity).norm() > 0,
            "contact must change angular momentum");
    require(after <= before + 1e-12, "inelastic point impulse must not add kinetic energy");
    require(tilted.state().orientation.angularDistance(s.orientation) < 1e-12,
            "contact cannot snap attitude");
    std::cout << "energy before=" << before << " old COM-only=" << wrong
              << " coupled=" << after << " normal impulse=" << contact.normal_impulse << '\n';

    RigidBodyModel low({0, 0, 1}), high({0, 0, 2});
    for (int i = 0; i < 200; ++i) {
      low.step(Eigen::Vector4d::Zero(), .01);
      high.step(Eigen::Vector4d::Zero(), .01);
      resolveFlatGroundContact(low, 0, .01);
      resolveFlatGroundContact(high, 0, .01);
    }
    require(std::abs(low.base_position().z()) < 1e-12 &&
            std::abs(high.base_position().z()) < 1e-12, "common world plane independent of spawn");
    const auto rest = resolveFlatGroundContact(high, 0);
    require(std::abs(rest.base_acceleration_world.z()) < 1e-10, "resting acceleration zero");
    require(std::abs(rest.base_acceleration_world.z() + high.parameters().gravity -
                     high.parameters().gravity) < 1e-10, "contact specific force gravity reaction");
    RigidBodyModel leaving({0, 0, 0});
    auto up = leaving.state(); up.velocity.z() = 1; leaving.set_state(up);
    const auto released = resolveFlatGroundContact(leaving, 0);
    require(released.normal_impulse == 0 && released.base_acceleration_world.z() < 0,
            "separating point is not glued to ground");
    std::cout << "PASS coupled impulse energy negative control, COM/base, shared plane, support and release\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL " << e.what() << '\n'; return 1;
  }
}
