# XGC2 lightweight simulator

Independent native simulation product: fixed PX4 v1.12.3 control core,
generated FS150 asset parameters, four motors and a 6DoF rigid body.
Runtime supplies only XgcRuntime::SDK and a generic Host; the core does not
require ROS, Gazebo or SSS. The optional ROS interface is the minimal
xgc2_lightweight_sim_msgs/SetProvider service package. The owning ROS facade
consumes that contract without another simulator or clock implementation.

Build/install details and behavior boundaries: src/xgc2_lightweight_sim/README.md.
Migration source/blob provenance: docs/source-migration.json.
