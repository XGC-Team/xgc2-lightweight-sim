#pragma once
// Data-only shim generated from the pinned upstream .msg.
// No uORB transport, metadata, publication or firmware scheduling.
#include <cstdint>
struct vehicle_attitude_setpoint_s {
  uint64_t timestamp{};
  float roll_body{};
  float pitch_body{};
  float yaw_body{};
  float yaw_sp_move_rate{};
  float q_d[4]{};
  float thrust_body[3]{};
  bool roll_reset_integral{};
  bool pitch_reset_integral{};
  bool yaw_reset_integral{};
  bool fw_control_yaw{};
  uint8_t apply_flaps{};
  static constexpr uint8_t FLAPS_OFF = 0;
  static constexpr uint8_t FLAPS_LAND = 1;
  static constexpr uint8_t FLAPS_TAKEOFF = 2;
};
