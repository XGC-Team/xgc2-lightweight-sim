# Assigned lightweight pure-simulation 3D authority: physics pose truth.
# This intentionally replaces fused display pose; it is not transport-only equivalence.
# Map numbers are interpreted directly in the existing world selector, no spawn offset.
# No noisy VRPN/AR substitution.
string body
uint32 robot_index
uint64 generation
uint32 validity
bool provider_enabled
std_msgs/Header header
geometry_msgs/Pose pose
