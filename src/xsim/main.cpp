#include "io/config.hpp"
#include "io/native_rpc/server.hpp"
#ifdef XSIM_ROS
#include "io/ros/runtime.hpp"
#include <ros/ros.h>
#endif
#include <csignal>
#include <iostream>
namespace {
volatile sig_atomic_t stopping = 0;
void signal_stop(int) { stopping = 1; }
#ifdef XSIM_ROS
struct RosShutdown {
  ~RosShutdown() { ros::shutdown(); }
};
#endif
} // namespace
int main(int argc, char **argv) {
  try {
    std::string config_path, socket;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--config" && i + 1 < argc) config_path = argv[++i];
      else if (arg == "--socket" && i + 1 < argc) socket = argv[++i];
      else if (arg == "--help") {
        std::cout << "xsim --config WORLD.json --socket /private/world.sock\n";
        return 0;
      } else throw std::invalid_argument("unknown argument: " + arg);
    }
    if (config_path.empty() || socket.empty())
      throw std::invalid_argument("--config and --socket required");
    const auto config = xsim::load_config(config_path);
#ifdef XSIM_ROS
    ros::init(argc, argv, "xsim", ros::init_options::NoSigintHandler);
    RosShutdown ros_shutdown;
#endif
    signal(SIGINT, signal_stop);
    signal(SIGTERM, signal_stop);
    xsim::World world(config.at("epoch_ns").get<int64_t>(),
                     config.value("model_step_ns", int64_t(1000000)),
                     config.value("output_period_ns", int64_t(10000000)),
                     config.value("catchup_batch", 8u));
    xsim::Sensors sensors(config.value("scene", xsim::Json::object()),
                          config.value("sensor_workers", 2u));
    xsim::RuntimeIO io;
#ifdef XSIM_ROS
    io = xsim::make_ros_io(config, world, sensors);
#endif
    xsim::Server server(config, socket, world, sensors, std::move(io));
    server.run(stopping);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "xsim: " << error.what() << '\n';
    return 1;
  }
}
