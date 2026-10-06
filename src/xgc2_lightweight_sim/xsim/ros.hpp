#pragma once
#include "sensors.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <xgc2_lightweight_sim_msgs/SetProvider.h>
namespace xsim {
struct RosEntity {
  RosEntity(const std::shared_ptr<Entity> &, World &, ros::CallbackQueue &,
            ros::CallbackQueue &);
  void reconcile(); // event thread only; bind immutable-generation callbacks
  void publish(const State &); // output thread only
  bool publish_sensor(); // at most one completed scan; output thread only
  std::weak_ptr<Entity> entity;
  World &world;
  ros::NodeHandle input, services;
  ros::Publisher truth, mocap, mocap_velocity, pose, velocity, imu, raw_imu,
      state, extended, target, odom, cloud;
  ros::Subscriber pva_sub, attitude_sub, velocity_sub;
  ros::ServiceServer provider, reset, arm, mode, command;
  uint64_t bound_generation = UINT64_MAX;
  bool bound_enabled = false, bound_alive = false;
  std::string frame, body_frame;
  std::mt19937 rng;
  std::normal_distribution<double> normal{0, 1};
  Eigen::Vector3d noise{Eigen::Vector3d::Zero()};
  bool last_enabled = false, last_armed = false;
  std::string last_mode;
  int64_t last_stamp = -1;
  uint64_t last_generation = UINT64_MAX;
  sensor_msgs::PointCloud2 cloud_message;
  std::string topic(const std::string &, const std::string &) const;
};
} // namespace xsim
