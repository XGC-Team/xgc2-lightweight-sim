// Private ROS fixture consuming the original Adapter projection implementation.
// This is a projection-boundary test, not a replacement production Adapter.
#include <array>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <xgc2_ros1_robot_adapter/localization_projection.hpp>
int main(int argc, char **argv) {
  ros::init(argc, argv, "xsim_canonical_fixture");
  ros::NodeHandle n;
  std::array<ros::Publisher, 3> poses, twists;
  std::array<ros::Subscriber, 3> ps, ts;
  const std::array<std::string, 3> names{{"fs150", "scout", "mecanum"}};
  for (size_t i = 0; i < names.size(); ++i) {
    poses[i] =
        n.advertise<geometry_msgs::PoseStamped>("/" + names[i] + "/pose", 100);
    twists[i] = n.advertise<geometry_msgs::TwistStamped>(
        "/" + names[i] + "/twist", 100);
    ps[i] = n.subscribe<geometry_msgs::PoseStamped>(
        "/vrpn_client_node/" + names[i] + "/pose", 100,
        [&, i](const geometry_msgs::PoseStamped::ConstPtr &source) {
          xgc2_ros1_robot_adapter::LocalizationProjectionConfig c;
          c.source_root = "/vrpn_client_node";
          c.offset_x = .4;
          c.offset_y = -.2;
          c.offset_z = .3;
          geometry_msgs::PoseStamped projected;
          if (xgc2_ros1_robot_adapter::projectLocalizationPose(*source, c,
                                                               &projected))
            poses[i].publish(projected);
        });
    ts[i] = n.subscribe<geometry_msgs::TwistStamped>(
        "/vrpn_client_node/" + names[i] + "/twist", 100,
        [&, i](const geometry_msgs::TwistStamped::ConstPtr &source) {
          if (xgc2_ros1_robot_adapter::validLocalizationTwist(*source))
            twists[i].publish(*source);
        });
  }
  ros::spin();
  return 0;
}
