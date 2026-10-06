#pragma once
#include "config.hpp"
#include "core/world.hpp"
#include <functional>

namespace xsim {
// One optional IO attachment per existing Entity. Its callbacks own the concrete
// transport resources; there is no second entity roster or lifecycle authority.
struct EntityIO {
  std::function<void()> reconcile;
  std::function<void(const State &)> publish;
  std::function<bool()> publish_sensor;
};

// Main composes the optional ROS boundary. Empty hooks give the same native
// server and model/sensor systems a transport-free, headless runtime.
struct RuntimeIO {
  std::function<void(const std::shared_ptr<Entity> &, const Json &)> attach_entity;
  std::function<void()> poll_inputs, poll_services;
  std::function<bool()> okay;
  std::function<void(const Frame &)> publish_frame;
};
} // namespace xsim
