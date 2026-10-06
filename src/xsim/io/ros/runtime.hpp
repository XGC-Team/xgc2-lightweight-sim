#pragma once
#include "io/runtime_io.hpp"
#include "systems/sensors.hpp"
namespace xsim {
RuntimeIO make_ros_io(const Json &config, World &, Sensors &);
} // namespace xsim
