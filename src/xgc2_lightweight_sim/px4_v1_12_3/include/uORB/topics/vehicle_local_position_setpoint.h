#pragma once
// Data-only shim generated from the pinned upstream .msg.
// No uORB transport, metadata, publication or firmware scheduling.
#include <cstdint>
struct vehicle_local_position_setpoint_s {
  uint64_t timestamp{};
  float x{};
  float y{};
  float z{};
  float yaw{};
  float yawspeed{};
  float vx{};
  float vy{};
  float vz{};
  float acceleration[3]{};
  float jerk[3]{};
  float thrust[3]{};
};
