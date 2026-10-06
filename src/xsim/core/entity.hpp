#pragma once
#include "components.hpp"
#include <utility>

namespace xsim {
struct EntityIO;
struct Sensor;
struct Entity {
  explicit Entity(Config c) : config(std::move(c)) {}
  const Config config;
  uint64_t id = 0;
  // Written only at world boundaries; callback/output projections use atomics.
  std::atomic<uint64_t> generation{0};
  std::atomic<bool> alive{false}, enabled{false};
  std::atomic<int64_t> generation_stamp{0};
  std::shared_ptr<EntityIO> io;
  std::shared_ptr<Sensor> sensor;
};
} // namespace xsim
