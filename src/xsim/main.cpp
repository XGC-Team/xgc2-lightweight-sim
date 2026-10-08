#include "io/config.hpp"
#include "io/native_rpc/server.hpp"
#ifdef XSIM_ROS
#include "io/ros/runtime.hpp"
#include <ros/ros.h>
#endif
#include <csignal>
#include <iostream>
#include <limits>
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
    std::string config_path, experiment_path, socket, scene_file;
    bool has_config = false, has_experiment = false, has_socket = false, has_scene = false;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "xsim (--config WORLD.json | --experiment-file EXPERIMENT.json) --socket /private/world.sock [--scene-file SCENE.yaml]\n";
        return 0;
      }
      std::string *value = nullptr;
      bool *present = nullptr;
      if (arg == "--config") { value = &config_path; present = &has_config; }
      else if (arg == "--experiment-file") { value = &experiment_path; present = &has_experiment; }
      else if (arg == "--socket") { value = &socket; present = &has_socket; }
      else if (arg == "--scene-file") { value = &scene_file; present = &has_scene; }
      else throw std::invalid_argument("unknown argument: " + arg);
      if (*present) throw std::invalid_argument("argument occurs more than once: " + arg);
      if (i + 1 >= argc || std::string(argv[i + 1]).compare(0, 2, "--") == 0)
        throw std::invalid_argument("missing value for " + arg);
      *present = true;
      *value = argv[++i];
    }
    if (has_config == has_experiment || socket.empty() || (has_config && config_path.empty()) || (has_experiment && experiment_path.empty()))
      throw std::invalid_argument("exactly one of --config or --experiment-file and --socket required");
    const auto config = has_experiment ? xsim::load_experiment_config(experiment_path, scene_file) : xsim::load_config(config_path, scene_file);
    const auto &epoch = config.at("epoch_ns");
    if (!epoch.is_number_integer() ||
        (epoch.is_number_unsigned() && epoch.get<uint64_t>() > uint64_t(std::numeric_limits<int64_t>::max())))
      throw std::invalid_argument("epoch_ns must be an explicit int64 integer from the frozen configuration");
#ifdef XSIM_ROS
    ros::init(argc, argv, "xsim", ros::init_options::NoSigintHandler);
    RosShutdown ros_shutdown;
#endif
    signal(SIGINT, signal_stop);
    signal(SIGTERM, signal_stop);
    xsim::World world(config.at("epoch_ns").get<int64_t>(),
                     config.value("model_step_ns", int64_t(2000000)),
                     config.value("output_period_ns", int64_t(8000000)),
                     config.value("catchup_batch", 8u),
                     config.value("max_model_step_ns", int64_t(10000000)));
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
