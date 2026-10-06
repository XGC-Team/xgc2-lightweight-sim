#include "robots.hpp"
namespace xsim {
Model prepare_model(const Config &c) {
  if (c.kind == Kind::FS150)
    return Flight(c);
  if (c.kind == Kind::Scout)
    return Scout(c);
  return Mecanum(c);
}
void step_robot(Flight &f, bool enabled, double h) {
  if (enabled) {
    if (f.model.step(h) == FlightEvent::OffboardLost)
      f.mode = "AUTO.LOITER";
  } else if (f.ever_started)
    f.model.stepPhysicsOnly(h);
}
void step_robot(Scout &s, int64_t dt) {
  ++s.steps;
  s.age = double(int64_t(s.steps) * dt) * 1e-9;
  s.model.advance(s.age);
}
void step_robot(Mecanum &m, double h) { m.model.step(h); }
} // namespace xsim
