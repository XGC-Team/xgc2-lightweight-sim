#pragma once
#include "core/components.hpp"
#include <variant>

namespace xsim {
struct Flight {
  FlightModel model;
  std::string mode = "POSCTL";
  bool ever_started = false;
  explicit Flight(const Config &c)
      : model(c.initial, c.yaw, c.fcu, c.ground_z) {}
};
struct Scout {
  ScoutModel model;
  double age = 0;
  int64_t age_ns = 0;
  explicit Scout(const Config &c) : model({c.initial.head<2>(), c.yaw}) {}
};
struct Mecanum {
  MecanumModel model;
  explicit Mecanum(const Config &c) : model({c.initial.head<2>(), c.yaw}) {}
};
using Model = std::variant<Flight, Scout, Mecanum>;
Model prepare_model(const Config &);
void step_robot(Flight &, bool enabled, double h);
void step_robot(Scout &, int64_t dt);
void step_robot(Mecanum &, double h);
} // namespace xsim
