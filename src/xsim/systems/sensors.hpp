#pragma once
#include "core/world.hpp"
#include "io/config.hpp"
#include <xgc2_world_lidar/world_lidar.h>
#ifdef XSIM_GPU
#include <xgc2_world_lidar/shared_cloud_gpu.hpp>
#endif
namespace xsim {
struct Sample {
  Key key;
  int64_t stamp = 0;
  uint64_t scene_version = 1;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Clock::time_point submitted;
  std::shared_ptr<const Frame> bodies;
};
struct Sensor {
  std::weak_ptr<Entity> entity;
  std::unique_ptr<xgc2_world_lidar::WorldLidar> cpu;
#ifdef XSIM_GPU
  xgc2_world_lidar::SensorMetadata gpu_config;
#endif
  bool gpu = false, with_bodies=false, publish_beams=false;
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  int64_t period = 100000000, next = 0;
  std::string topic, frame = "world";
  std::mutex mutex;
  Sample pending, completed;
  bool has_pending = false, busy = false, has_completed = false;
  std::vector<uint8_t> data, scratch, beam_data, beam_scratch;
  std::vector<xgc2_world_lidar::VehicleBody> others;
  std::atomic<uint64_t> misses{0}, scans{0}, errors{0};
  std::atomic<int64_t> latency_ns{0}, max_latency_ns{0};
  std::string error;
};
class Sensors {
public:
  explicit Sensors(const Json &scene, unsigned workers = 2);
  ~Sensors();
  std::shared_ptr<Sensor> prepare(const std::shared_ptr<Entity> &, const Json &config);
  void submit(const State &, const World *world=nullptr);
  std::vector<uint8_t> reference_cloud() const;
  void stop();
  static bool take(const std::shared_ptr<Sensor> &, Sample &,
                   std::vector<uint8_t> &, std::vector<uint8_t> *beams=nullptr);
  Json status() const;

private:
  std::shared_ptr<const xgc2_world_lidar::LidarScene> scene_;
  std::vector<xgc2_world_lidar::Obstacle> obstacles_;
  double spacing_ = 0.1;
  bool buried_ = false;
  unsigned workers_;
  std::shared_ptr<const Frame> body_sample_;
  std::map<std::pair<double,bool>,std::shared_ptr<const xgc2_world_lidar::LidarScene>> sampled_scenes_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::vector<std::weak_ptr<Sensor>> sensors_;
  std::vector<std::thread> cpu_threads_;
  std::thread gpu_thread_;
  bool stopping_ = false;
#ifdef XSIM_GPU
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  xgc2_world_lidar::SensorMetadata gpu_initial_config_;
  bool gpu_ready_ = false;
  std::string gpu_error_;
#endif
  void work(bool gpu);
};
} // namespace xsim
