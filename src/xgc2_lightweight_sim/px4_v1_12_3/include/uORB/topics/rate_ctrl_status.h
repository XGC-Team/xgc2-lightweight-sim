#pragma once
// Data-only shim generated from the pinned upstream .msg.
// No uORB transport, metadata, publication or firmware scheduling.
#include <cstdint>
struct rate_ctrl_status_s {
  uint64_t timestamp{};
  float rollspeed_integ{};
  float pitchspeed_integ{};
  float yawspeed_integ{};
  float additional_integ1{};
};
