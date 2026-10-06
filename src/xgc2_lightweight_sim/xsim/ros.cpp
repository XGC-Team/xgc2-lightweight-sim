#include "ros.hpp"
namespace xsim {
namespace {
Ticket ticket(const std::shared_ptr<Entity> &e, Op op, uint64_t generation) {
  auto t = std::make_shared<Command>();
  t->op = op;
  t->key = {e->id, generation};
  return t;
}
void vector(geometry_msgs::Vector3 &o, const Eigen::Vector3d &v) {
  o.x = v.x();
  o.y = v.y();
  o.z = v.z();
}
void orientation(geometry_msgs::Quaternion &o, const Eigen::Quaterniond &q) {
  o.w = q.w();
  o.x = q.x();
  o.y = q.y();
  o.z = q.z();
}
bool current(const std::shared_ptr<Entity> &e, uint64_t generation) {
  return e->alive && e->enabled && e->generation == generation;
}
} // namespace
std::string RosEntity::topic(const std::string &key,
                             const std::string &suffix) const {
  auto e = entity.lock();
  return e->config.ros.value(key, "/" + e->config.name + suffix);
}
RosEntity::RosEntity(const std::shared_ptr<Entity> &e, World &w,
                     ros::CallbackQueue &iq, ros::CallbackQueue &sq)
    : entity(e), world(w), rng(e->config.ros.value("mocap_seed", 1u)) {
  input.setCallbackQueue(&iq);
  services.setCallbackQueue(&sq);
  frame = e->config.ros.value(
      "frame", std::string(e->config.kind == Kind::FS150 ? "map" : "world"));
  body_frame = e->config.ros.value("body_frame", std::string("base_link"));
  if (e->config.ros.contains("mocap_noise")) {
    const auto &n = e->config.ros.at("mocap_noise");
    if (n.size() != 3)
      throw std::invalid_argument("mocap_noise needs 3 standard deviations");
    for (int a = 0; a < 3; ++a)
      noise[a] = n[a].get<double>();
    if (!noise.allFinite() || (noise.array() < 0).any())
      throw std::invalid_argument("invalid mocap noise");
  }
  truth = input.advertise<geometry_msgs::PoseStamped>(
      topic("truth_topic", "/simulation/body_pose"), 1);
  mocap = input.advertise<geometry_msgs::PoseStamped>(
      e->config.ros.value("mocap_topic",
                          "/vrpn_client_node/" + e->config.name + "/pose"),
      1);
  mocap_velocity = input.advertise<geometry_msgs::TwistStamped>(
      e->config.ros.value("mocap_velocity_topic",
                          "/vrpn_client_node/" + e->config.name + "/twist"),
      1);
  const bool flight = e->config.kind == Kind::FS150;
  pose = input.advertise<geometry_msgs::PoseStamped>(
      topic("pose_topic",
            flight ? "/mavros/local_position/pose" : "/simulation/pose"),
      1);
  velocity = input.advertise<geometry_msgs::TwistStamped>(
      topic("velocity_topic", flight ? "/mavros/local_position/velocity_local"
                                     : "/simulation/velocity"),
      1);
  odom = input.advertise<nav_msgs::Odometry>(
      topic("odometry_topic", flight ? "/mavros/local_position/odom" : "/odom"),
      1);
  if (flight) {
    imu = input.advertise<sensor_msgs::Imu>(
        topic("imu_topic", "/mavros/imu/data"), 1);
    raw_imu = input.advertise<sensor_msgs::Imu>(
        topic("raw_imu_topic", "/mavros/imu/data_raw"), 1);
    state = input.advertise<mavros_msgs::State>(
        topic("state_topic", "/mavros/state"), 1);
    extended = input.advertise<mavros_msgs::ExtendedState>(
        topic("extended_state_topic", "/mavros/extended_state"), 1);
    target = input.advertise<mavros_msgs::AttitudeTarget>(
        topic("target_attitude_topic", "/mavros/setpoint_raw/target_attitude"),
        1);
  }
  if (e->sensor) {
    cloud = input.advertise<sensor_msgs::PointCloud2>(e->sensor->topic, 1);
    cloud_message.fields.resize(e->sensor->gpu ? 4 : 3);
    for (unsigned a = 0; a < cloud_message.fields.size(); ++a) {
      auto &f = cloud_message.fields[a];
      f.name = a == 3 ? "intensity" : std::string(1, "xyz"[a]);
      f.offset = a * 4;
      f.datatype = sensor_msgs::PointField::FLOAT32;
      f.count = 1;
    }
    cloud_message.point_step = e->sensor->gpu ? 16 : 12;
    cloud_message.height = 1;
    cloud_message.is_dense = true;
    cloud_message.is_bigendian = false;
  }
  boost::function<bool(xgc2_lightweight_sim_msgs::SetProvider::Request &,
                       xgc2_lightweight_sim_msgs::SetProvider::Response &)>
      provider_fn = [weak = entity, &w](auto &req, auto &res) {
        auto e = weak.lock();
        if (!e || !e->alive)
          return false;
        auto t = ticket(e, Op::Provider, req.generation);
        t->action = req.action;
        if (req.action == 1)
          t->prepared =
              std::make_unique<Prepared>(Prepared{e, prepare_model(e->config)});
        w.submit(t);
        if (!World::wait(t))
          return false;
        res.accepted = t->result.success;
        res.enabled = t->result.enabled;
        res.generation = t->result.key.generation;
        res.reason = t->result.reason;
        res.message = res.accepted ? "accepted"
                                   : res.reason == 1 ? "stale generation"
                                                     : "invalid action or body";
        return true;
      };
  provider = services.advertiseService(
      topic("provider_service", "/simulation/provider"), provider_fn);
  reconcile();
}
void RosEntity::reconcile() {
  auto e = entity.lock();
  if (!e)
    return;
  const uint64_t gen = e->generation;
  const bool enabled = e->alive && e->enabled;
  if (bound_generation == gen && bound_enabled == enabled &&
      bound_alive == e->alive)
    return;
  bound_generation = gen;
  bound_enabled = enabled;
  bound_alive = e->alive;
  pva_sub.shutdown();
  attitude_sub.shutdown();
  velocity_sub.shutdown();
  arm.shutdown();
  mode.shutdown();
  command.shutdown();
  reset.shutdown();
  if (!e->alive)
    return;
  boost::function<bool(std_srvs::Trigger::Request &,
                       std_srvs::Trigger::Response &)>
      reset_fn = [weak = entity, gen, this](auto &, auto &res) {
        auto e = weak.lock();
        if (!e || !e->alive)
          return false;
        auto t = ticket(e, Op::Reset, gen);
        t->prepared =
            std::make_unique<Prepared>(Prepared{e, prepare_model(e->config)});
        world.submit(t);
        if (!World::wait(t))
          return false;
        res.success = t->result.success;
        res.message = res.success ? "reset applied" : "stale generation";
        return true;
      };
  reset = services.advertiseService(topic("reset_service", "/simulation/reset"),
                                    reset_fn);
  if (e->config.kind != Kind::FS150) {
    if (!enabled)
      return;
    velocity_sub = input.subscribe<geometry_msgs::Twist>(
        topic("cmd_vel_topic", "/cmd_vel"), 1,
        [weak = entity, gen, this](const geometry_msgs::Twist::ConstPtr &m) {
          auto e = weak.lock();
          if (!e || !current(e, gen))
            return;
          auto t = ticket(e, Op::Velocity, gen);
          t->velocity = {m->linear.x, m->linear.y, m->angular.z};
          world.submit(t);
        });
    return;
  }
  if (enabled) {
    pva_sub = input.subscribe<mavros_msgs::PositionTarget>(
        topic("setpoint_topic", "/mavros/setpoint_raw/local"), 1,
        [weak = entity, gen,
         this](const mavros_msgs::PositionTarget::ConstPtr &m) {
          auto e = weak.lock();
          if (!e || !current(e, gen))
            return;
          const int64_t stamp = m->header.stamp.toNSec();
          if ((stamp && stamp < e->generation_stamp) ||
              stamp > world.metrics.sim_ns + 1000000000)
            return;
          auto t = ticket(e, Op::Pva, gen);
          t->at = stamp;
          auto &p = t->pva;
          p.stamp = m->header.stamp.toSec();
          p.coordinate_frame = m->coordinate_frame;
          p.type_mask = m->type_mask;
          p.position[0] = m->position.x;
          p.position[1] = m->position.y;
          p.position[2] = m->position.z;
          p.velocity[0] = m->velocity.x;
          p.velocity[1] = m->velocity.y;
          p.velocity[2] = m->velocity.z;
          p.acceleration[0] = m->acceleration_or_force.x;
          p.acceleration[1] = m->acceleration_or_force.y;
          p.acceleration[2] = m->acceleration_or_force.z;
          p.yaw = m->yaw;
          p.yaw_rate = m->yaw_rate;
          world.submit(t);
        });
    attitude_sub = input.subscribe<mavros_msgs::AttitudeTarget>(
        topic("attitude_topic", "/mavros/setpoint_raw/attitude"), 1,
        [weak = entity, gen,
         this](const mavros_msgs::AttitudeTarget::ConstPtr &m) {
          auto e = weak.lock();
          if (!e || !current(e, gen))
            return;
          const int64_t stamp = m->header.stamp.toNSec();
          if ((stamp && stamp < e->generation_stamp) ||
              stamp > world.metrics.sim_ns + 1000000000)
            return;
          auto t = ticket(e, Op::Attitude, gen);
          t->at = stamp;
          t->attitude.q = {m->orientation.w, m->orientation.x, m->orientation.y,
                           m->orientation.z};
          t->attitude.body_rate = {m->body_rate.x, m->body_rate.y,
                                   m->body_rate.z};
          t->attitude.thrust = m->thrust;
          t->attitude.type_mask = m->type_mask;
          world.submit(t);
        });
  }
  boost::function<bool(mavros_msgs::CommandBool::Request &,
                       mavros_msgs::CommandBool::Response &)>
      arm_fn = [weak = entity, gen, this](auto &req, auto &res) {
        auto e = weak.lock();
        res.success = false;
        res.result = 4;
        if (!e || !current(e, gen))
          return true;
        auto t = ticket(e, Op::Arm, gen);
        t->arm = req.value;
        world.submit(t);
        if (World::wait(t)) {
          res.success = t->result.success;
          res.result =
              res.success ? 0 : t->result.reason == 1 ? 4 : t->result.reason;
        }
        return true;
      };
  arm = services.advertiseService(topic("arming_service", "/mavros/cmd/arming"),
                                  arm_fn);
  boost::function<bool(mavros_msgs::SetMode::Request &,
                       mavros_msgs::SetMode::Response &)>
      mode_fn = [weak = entity, gen, this](auto &req, auto &res) {
        auto e = weak.lock();
        res.mode_sent = false;
        if (!e || !current(e, gen) || req.base_mode ||
            req.custom_mode.empty() || req.custom_mode.size() >= 32 ||
            req.custom_mode.find('\0') != std::string::npos)
          return true;
        auto t = ticket(e, Op::Mode, gen);
        t->mode = req.custom_mode;
        world.submit(t);
        // MAVROS mode_sent is transmission, not Commander mode acceptance. The
        // response waits for execution; rejected mode is observable in state.
        if (World::wait(t))
          res.mode_sent =
              t->result.applied && t->result.enabled && t->result.reason != 1;
        return true;
      };
  mode = services.advertiseService(topic("mode_service", "/mavros/set_mode"),
                                   mode_fn);
  boost::function<bool(mavros_msgs::CommandLong::Request &,
                       mavros_msgs::CommandLong::Response &)>
      command_fn = [weak = entity, gen, this](auto &req, auto &res) {
        res.success = false;
        res.result = 3;
        if (req.command != 400)
          return true;
        if (req.param1 != 0 && req.param1 != 1) {
          res.result = 2;
          return true;
        }
        if (req.broadcast || req.confirmation ||
            (req.param2 != 0 && req.param2 != 21196) || req.param3 ||
            req.param4 || req.param5 || req.param6 || req.param7)
          return true;
        auto e = weak.lock();
        res.result = 4;
        if (!e || !current(e, gen))
          return true;
        auto t = ticket(e, Op::Arm, gen);
        t->arm = req.param1 == 1;
        t->action = req.param2 == 21196 ? 1 : 0;
        world.submit(t);
        if (World::wait(t)) {
          res.success = t->result.success;
          res.result =
              res.success ? 0 : t->result.reason == 1 ? 4 : t->result.reason;
        }
        return true;
      };
  command = services.advertiseService(
      topic("command_service", "/mavros/cmd/command"), command_fn);
}
void RosEntity::publish(const State &s) {
  auto e = entity.lock();
  if (!e || !e->alive || s.key.generation != e->generation)
    return;
  const bool fresh =
      s.stamp != last_stamp || s.key.generation != last_generation;
  ros::Time stamp;
  stamp.fromNSec(s.stamp);
  if (e->config.kind == Kind::FS150 &&
      (fresh || last_enabled != s.enabled || last_armed != s.armed ||
       last_mode != s.mode.data())) {
    mavros_msgs::State m;
    m.header.stamp = stamp;
    m.connected = s.enabled;
    m.armed = s.enabled && s.armed;
    m.guided = s.enabled;
    m.system_status = !s.enabled ? 0 : s.armed ? 4 : 3;
    m.mode = s.mode.data();
    state.publish(m);
    mavros_msgs::ExtendedState x;
    x.header.stamp = stamp;
    x.landed_state = s.landed
                         ? (s.armed && s.velocity.z() > 0 ? 3 : 1)
                         : (std::string(s.mode.data()) == "AUTO.LAND" ? 4 : 2);
    extended.publish(x);
  }
  last_enabled = s.enabled;
  last_armed = s.armed;
  last_mode = s.mode.data();
  if (fresh) {
    last_stamp = s.stamp;
    last_generation = s.key.generation;
    if (s.enabled) {
      geometry_msgs::PoseStamped p;
      p.header.stamp = stamp;
      p.header.frame_id = "world";
      p.pose.position.x = s.position.x();
      p.pose.position.y = s.position.y();
      p.pose.position.z = s.position.z();
      orientation(p.pose.orientation, s.orientation);
      truth.publish(p);
      auto measured = p;
      double *axes[3] = {&measured.pose.position.x, &measured.pose.position.y,
                         &measured.pose.position.z};
      for (int a = 0; a < 3; ++a)
        if (noise[a] > 0)
          *axes[a] += noise[a] * normal(rng);
      mocap.publish(measured);
      p.header.frame_id = frame;
      p.pose.position.x -= e->config.local_origin.x();
      p.pose.position.y -= e->config.local_origin.y();
      p.pose.position.z -= e->config.local_origin.z();
      pose.publish(p);
      geometry_msgs::TwistStamped v;
      v.header = p.header;
      vector(v.twist.linear, s.velocity);
      vector(v.twist.angular, s.orientation * s.omega);
      velocity.publish(v);
      auto mocap_v = v;
      mocap_v.header.frame_id = "world";
      mocap_velocity.publish(mocap_v);
      nav_msgs::Odometry o;
      o.header = p.header;
      o.child_frame_id = body_frame;
      o.pose.pose = p.pose;
      vector(o.twist.twist.linear, s.orientation.conjugate() * s.velocity);
      vector(o.twist.twist.angular, s.omega);
      odom.publish(o);
      if (e->config.kind == Kind::FS150) {
        sensor_msgs::Imu m;
        m.header.stamp = stamp;
        m.header.frame_id = body_frame;
        m.orientation = p.pose.orientation;
        vector(m.angular_velocity, s.omega);
        vector(m.linear_acceleration, s.specific_force);
        imu.publish(m);
        m.orientation = geometry_msgs::Quaternion{};
        m.orientation_covariance[0] = -1.0;
        raw_imu.publish(m);
        mavros_msgs::AttitudeTarget a;
        a.header = p.header;
        orientation(a.orientation, s.control.desired_orientation);
        vector(a.body_rate, s.control.desired_body_rate);
        a.thrust = s.control.normalized_thrust;
        a.type_mask = s.control.type_mask;
        target.publish(a);
      }
    }
  }
  }
bool RosEntity::publish_sensor() {
  auto e=entity.lock();
  if(!e || !e->alive || !e->enabled || !e->sensor)return false;
  Sample sample;
  if(!Sensors::take(e->sensor,sample,cloud_message.data))return false;
  cloud_message.header.stamp.fromNSec(sample.stamp);
  cloud_message.header.frame_id=e->sensor->frame;
  cloud_message.width=cloud_message.data.size()/cloud_message.point_step;
  cloud_message.row_step=cloud_message.width*cloud_message.point_step;
  cloud.publish(cloud_message);
  return true;
}
} // namespace xsim
