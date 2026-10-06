#include "runtime.hpp"
#include "entity.hpp"
#include <rosgraph_msgs/Clock.h>
namespace xsim {
namespace {
struct RosRuntime {
  ros::CallbackQueue inputs, services;
  ros::NodeHandle node;
  ros::Publisher clock, reference;
  sensor_msgs::PointCloud2 reference_message;
  bool reference_sent = false;
  int64_t clock_stamp = -1;

  RosRuntime(const Json &config, Sensors &sensors) {
    const auto topic = config.value("reference_cloud_topic", std::string{});
    if (!topic.empty()) {
      reference = node.advertise<sensor_msgs::PointCloud2>(topic, 1, true);
      reference_message.height=1;reference_message.point_step=12;reference_message.is_dense=true;reference_message.header.frame_id="world";
      for(unsigned a=0;a<3;++a){sensor_msgs::PointField f;f.name=std::string(1,"xyz"[a]);f.offset=a*4;f.datatype=sensor_msgs::PointField::FLOAT32;f.count=1;reference_message.fields.push_back(f);}
      reference_message.data=sensors.reference_cloud();reference_message.width=reference_message.data.size()/12;reference_message.row_step=reference_message.data.size();
    }
    if (config.value("publish_clock", false))
      clock = node.advertise<rosgraph_msgs::Clock>("/clock", 1);
  }
  void publish(const Frame &frame) {
    if (clock && frame.stamp != clock_stamp) {
      rosgraph_msgs::Clock c;
      c.clock.fromNSec(frame.stamp);
      clock.publish(c);
      clock_stamp = frame.stamp;
    }
    if(reference && !reference_sent){reference_message.header.stamp.fromNSec(frame.stamp);reference.publish(reference_message);reference_sent=true;}
  }
};
} // namespace
RuntimeIO make_ros_io(const Json &config, World &world, Sensors &sensors) {
  auto runtime = std::make_shared<RosRuntime>(config, sensors);
  RuntimeIO io;
  io.attach_entity = [runtime, &world](const std::shared_ptr<Entity> &entity, const Json &settings) {
    auto ros = std::make_shared<RosEntity>(entity, world, runtime->inputs, runtime->services, settings);
    // Each attachment retains the same concrete queue owner until its Entity retires.
    entity->io = std::make_shared<EntityIO>(EntityIO{
      [ros, runtime] { ros->reconcile(); },
      [ros, runtime](const State &state) { ros->publish(state); },
      [ros, runtime] { return ros->publish_sensor(); },
    });
  };
  io.poll_inputs = [runtime] { runtime->inputs.callAvailable(ros::WallDuration(0)); };
  io.poll_services = [runtime] { runtime->services.callOne(ros::WallDuration(.01)); };
  io.okay = [] { return ros::ok(); };
  io.publish_frame = [runtime](const Frame &frame) { runtime->publish(frame); };
  return io;
}
} // namespace xsim
