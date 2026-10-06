#include "sensors.hpp"
#include <xgc2_world_lidar/scene_conversion.h>
#include <cstring>
#include <pthread.h>
#ifdef XSIM_GPU
#include <omp.h>
#endif
namespace xsim {
namespace {
Eigen::Vector3d v3(const Json &j) {
  if (j.size() != 3)
    throw std::invalid_argument("sensor vector needs 3 values");
  return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()};
}
Eigen::Quaterniond quat(const Json &j) {
  if (j.size() != 4)
    throw std::invalid_argument("sensor quaternion needs wxyz");
  Eigen::Quaterniond q(j[0].get<double>(), j[1].get<double>(),
                       j[2].get<double>(), j[3].get<double>());
  if (!q.coeffs().allFinite() || q.norm() < 1e-12)
    throw std::invalid_argument("sensor quaternion invalid");
  return q.normalized();
}
} // namespace
Sensors::Sensors(const Json &j, unsigned workers) : workers_(workers) {
  if (!workers)
    throw std::invalid_argument("sensor_workers must be positive");
  spacing_ = j.value("surface_spacing", 0.1);
  buried_ = j.value("keep_buried", false);
  if(j.contains("document")) {
    std::vector<xgc2_world_lidar::SceneObstacleDescription> descriptions;
    const auto pose=[](const Json& p){xgc2_world_lidar::Pose r;
      r.position=v3(p.value("position",Json::array({0,0,0})));
      auto q=p.value("orientation",Json::array({0,0,0,1}));
      r.orientation=Eigen::Quaterniond(q[3].get<double>(),q[0].get<double>(),q[1].get<double>(),q[2].get<double>()).normalized();return r;};
    for(const auto& o:j.at("document").at("obstacles")) {
      xgc2_world_lidar::SceneObstacleDescription d;d.id=o.at("id");d.pose=pose(o.value("pose",Json::object()));
      for(const auto& part:o.at("parts")) {
        const auto& g=part.at("geometry");xgc2_world_lidar::ScenePartDescription p;p.type=g.at("type");p.pose=pose(part.value("pose",Json::object()));
        if(g.contains("size"))p.size=v3(g.at("size"));p.radius=g.value("radius",0.0);p.height=g.value("height",0.0);
        if(g.contains("vertices"))for(const auto& v:g.at("vertices"))p.vertices.push_back(v3(v));d.parts.push_back(p);
      }descriptions.push_back(std::move(d));
    }obstacles_=xgc2_world_lidar::toObstacles(descriptions);
  }
  for (const auto &o : j.value("obstacles", Json::array())) {
    const auto type = o.at("type").get<std::string>();
    auto pos = v3(o.at("position"));
    auto q = o.contains("orientation") ? quat(o.at("orientation"))
                                       : Eigen::Quaterniond::Identity();
    using O = xgc2_world_lidar::Obstacle;
    if (type == "box")
      obstacles_.push_back(O::box(pos, v3(o.at("size")), q));
    else if (type == "sphere")
      obstacles_.push_back(O::sphere(pos, o.at("radius")));
    else if (type == "cylinder")
      obstacles_.push_back(O::cylinder(pos, o.at("radius"), o.at("height"), q));
    else if (type == "capsule")
      obstacles_.push_back(O::capsule(pos, o.at("radius"), o.at("height"), q));
    else if (type == "convex") {
      std::vector<Eigen::Vector3d> v;
      for (auto &x : o.at("vertices"))
        v.push_back(v3(x));
      obstacles_.push_back(O::convexHull(v, pos, q));
    } else
      throw std::invalid_argument("unknown obstacle type");
  }
  // One immutable compiled geometry/index for this world, never one map per
  // robot.
  scene_ = std::make_shared<xgc2_world_lidar::LidarScene>(obstacles_, spacing_,
                                                          buried_);
}
std::vector<uint8_t> Sensors::reference_cloud() const {
  xgc2_world_lidar::SensorConfig c;c.surface_spacing=spacing_;c.penetrating_keep_buried=buried_;
  xgc2_world_lidar::WorldLidar source(c);source.setScene(scene_);const auto points=source.globalMap(spacing_);
  std::vector<uint8_t> data(points.size()*12);size_t at=0;for(const auto& p:points){float xyz[3]={float(p.x()),float(p.y()),float(p.z())};std::memcpy(data.data()+at,xyz,12);at+=12;}return data;
}
Sensors::~Sensors() { stop(); }
std::shared_ptr<Sensor> Sensors::prepare(const std::shared_ptr<Entity> &e) {
  const auto &j = e->config.sensor;
  if (j.empty())
    return {};
  auto s = std::make_shared<Sensor>();
  s->entity = e;
  s->topic = j.value("topic", "/" + e->config.name + "/cloud");
  s->frame = j.value("frame", "world");
  if (s->frame != "world" && s->frame != "map")
    throw std::invalid_argument("existing sensor emits world/map points");
  const auto hz = j.value("rate_hz", 10.0);
  if (!std::isfinite(hz) || hz <= 0 || hz > 1e9)
    throw std::invalid_argument("invalid sensor rate");
  s->period = std::llround(1e9 / hz);
  if (j.contains("translation"))
    s->translation = v3(j.at("translation"));
  if (!s->translation.allFinite())
    throw std::invalid_argument("invalid sensor translation");
  if (j.contains("rotation"))
    s->rotation = quat(j.at("rotation"));
  const auto backend = j.value("backend", std::string("cpu"));
  if (backend != "cpu" && backend != "gpu")
    throw std::invalid_argument("backend must be cpu or gpu");
  s->gpu = backend == "gpu";
  s->with_bodies=j.value("world_bodies",false);s->publish_beams=j.value("publish_beams",false);
  if(s->gpu && (s->with_bodies || s->publish_beams)) throw std::invalid_argument("original GPU has static map points and no beams");
  xgc2_world_lidar::SensorConfig c;
  c.range = j.value("range", 20.0);
  c.min_range = j.value("min_range", 0.0);
  c.h_res = j.value("h_res", 360);
  c.v_res = j.value("v_res", 32);
  c.h_fov_deg = j.value("h_fov_deg", 360.0);
  c.v_fov_deg = j.value("v_fov_deg", 30.0);
  c.noise_std = j.value("noise_std", 0.0);
  c.seed = j.value("seed", 0u);
  c.surface_spacing = spacing_;
  c.penetrating_keep_buried = j.value("keep_buried",buried_);
  c.surface_spacing=j.value("surface_spacing",spacing_);
  c.penetrating_heading_crop=j.value("heading_crop",false);
  c.heading_cos_min=j.value("heading_cos_min",0.0);
  c.vertical_slab_tan=j.value("vertical_slab_tan",0.5773502691896258);
  const auto mode = j.value("mode", std::string("raycast"));
  if (mode == "raycast")
    c.mode = xgc2_world_lidar::SensorConfig::kRaycast;
  else if (mode == "penetrating")
    c.mode = xgc2_world_lidar::SensorConfig::kPenetrating;
  else if (mode == "depth") {
    c.mode = xgc2_world_lidar::SensorConfig::kDepthFrustum;
    c.width = j.value("width", 160);
    c.height = j.value("height", 120);
    c.fx = j.value("fx", 0.0);
    c.fy = j.value("fy", 0.0);
    c.cx = j.value("cx", 0.0);
    c.cy = j.value("cy", 0.0);
  } else if (!(s->gpu && mode == "lidar_scan"))
    throw std::invalid_argument("unknown sensor model");
  if(s->publish_beams && c.mode==xgc2_world_lidar::SensorConfig::kPenetrating)throw std::invalid_argument("penetrating model has no beams");
  if (!s->gpu) {
    s->cpu = std::make_unique<xgc2_world_lidar::WorldLidar>(c);
    if(c.mode==xgc2_world_lidar::SensorConfig::kPenetrating && (c.surface_spacing!=spacing_ || c.penetrating_keep_buried!=buried_)) {
      const auto key=std::make_pair(c.surface_spacing,c.penetrating_keep_buried);
      std::lock_guard<std::mutex> l(mutex_);auto& scene=sampled_scenes_[key];
      if(!scene)scene=std::make_shared<xgc2_world_lidar::LidarScene>(obstacles_,key.first,key.second);
      s->cpu->setScene(scene);
    } else s->cpu->setScene(scene_);
  } else {
#ifndef XSIM_GPU
    throw std::invalid_argument("GPU backend not compiled; no CPU fallback");
#else
    if (mode != "lidar_scan" || c.noise_std != 0)
      throw std::invalid_argument(
          "original GPU supports lidar_scan without range noise only");
    auto &m = s->gpu_config;
    m.backend = "gpu";
    m.observation_model = "lidar_scan";
    m.range_m = c.range;
    m.min_range_m = c.min_range;
    m.h_fov_deg = c.h_fov_deg;
    m.v_fov_deg = c.v_fov_deg;
    m.h_res = c.h_res;
    m.v_res = c.v_res;
    m.point_cover_spacing_m = j.value("point_cover_spacing", spacing_);
    m.publish_rate_hz = hz;
    m.frame_id = s->frame;
    m.stamp_policy = "pose";
    m.pose_type = "geometry_msgs/PoseStamped";
    xgc2_world_lidar::validateGpuSensorMetadata(m);
#endif
  }
  std::unique_lock<std::mutex> lock(mutex_);
#ifdef XSIM_GPU
  if (s->gpu && !cloud_) {
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    xgc2_world_lidar::WorldLidar source(c);
    source.setScene(scene_);
    for (const auto &p : source.globalMap(spacing_))
      cloud_->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
  }
#endif
  sensors_.erase(
      std::remove_if(sensors_.begin(), sensors_.end(),
                     [](const auto &weak) { return weak.expired(); }),
      sensors_.end());
  sensors_.push_back(s);
  if (s->gpu) {
    if (!gpu_thread_.joinable()) {
#ifdef XSIM_GPU
      gpu_initial_config_ = s->gpu_config;
#endif
      gpu_thread_ = std::thread([this] { work(true); });
    }
#ifdef XSIM_GPU
    wake_.wait(lock,
               [&] { return gpu_ready_ || !gpu_error_.empty() || stopping_; });
    if (!gpu_ready_)
      throw std::runtime_error(gpu_error_.empty() ? "sensor owner stopped"
                                                  : gpu_error_);
#endif
  } else if (cpu_threads_.empty())
    for (unsigned i = 0; i < workers_; ++i)
      cpu_threads_.emplace_back([this] { work(false); });
  wake_.notify_all();
  return s;
}
void Sensors::submit(const State &v, const World *world) {
  auto owner = v.entity.lock();
  if (!owner)
    return;
  auto s = owner->sensor;
  if (!s || v.stamp < s->next)
    return;
  // Absolute simulation schedule; missed samples counted, never change
  // requested Hz.
  if (s->next == 0)
    s->next = v.stamp;
  const auto due = uint64_t((v.stamp - s->next) / s->period) + 1;
  s->next += int64_t(due) * s->period;
  if (due > 1)
    s->misses += due - 1;
  std::unique_lock<std::mutex> lock(s->mutex, std::try_to_lock);
  if (!lock.owns_lock()) {
    ++s->misses;
    return;
  }
  if (s->has_pending)
    ++s->misses;
  s->pending = {v.key,
                v.stamp,
                1,
                v.position + v.orientation * s->translation,
                v.orientation * s->rotation,
                Clock::now()};
  if(s->with_bodies && world) {
    // One immutable whole-world pose snapshot shared by all sensors due on
    // this step. No provider/ROS roster supplies or owns these bodies.
    if(!body_sample_ || body_sample_->stamp!=v.stamp)body_sample_=std::make_shared<const Frame>(world->capture());
    s->pending.bodies=body_sample_;
  }
  s->has_pending = true;
  wake_.notify_all();
}
void Sensors::work(bool gpu) {
  pthread_setname_np(pthread_self(), gpu ? "xsim-gpu" : "xsim-sensor");
#ifdef XSIM_GPU
  if (gpu) {
    omp_set_dynamic(0);
    omp_set_num_threads(1);
  }
  std::unique_ptr<xgc2_world_lidar::SharedCloudGpu> renderer;
  if (gpu) {
    try {
      renderer = std::make_unique<xgc2_world_lidar::SharedCloudGpu>();
      renderer->load(cloud_, gpu_initial_config_);
      {
        std::lock_guard<std::mutex> l(mutex_);
        gpu_ready_ = true;
      }
      wake_.notify_all();
    } catch (const std::exception &e) {
      {
        std::lock_guard<std::mutex> l(mutex_);
        gpu_error_ = e.what();
      }
      wake_.notify_all();
      return;
    }
  }
#endif
  size_t cursor = 0;
  for (;;) {
    std::shared_ptr<Sensor> selected;
    Sample sample;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (stopping_)
        break;
      for (size_t n = 0; n < sensors_.size(); ++n) {
        auto s = sensors_[(cursor + n) % sensors_.size()].lock();
        if (!s || s->gpu != gpu)
          continue;
        auto e = s->entity.lock();
        if (!e || !e->alive || !e->enabled)
          continue;
        std::lock_guard<std::mutex> sl(s->mutex);
        if (!s->busy && s->has_pending) {
          s->busy = true;
          s->has_pending = false;
          sample = s->pending;
          selected = s;
          cursor = (cursor + n + 1) % sensors_.size();
          break;
        }
      }
      if (!selected) {
        wake_.wait_for(lock, std::chrono::milliseconds(2));
        continue;
      }
    }
    auto &s = *selected;
    bool ok = true;
    try {
      if (!gpu) {
        s.others.clear();
        if(sample.bodies)for(const auto& b:sample.bodies->states)if(b.key.id!=sample.key.id)s.others.push_back({int(b.key.id),b.position,0.3});
        if(!s.publish_beams)s.cpu->scanInto(sample.position,sample.orientation,s.others,s.with_bodies,&s.scratch);
        else {
          const auto beams=s.cpu->scanWithBeams(sample.position,sample.orientation,s.others);
          const size_t point_stride=s.with_bodies?16:12,beam_stride=s.with_bodies?36:32;
          s.scratch.clear();s.beam_scratch.resize(beams.size()*beam_stride);size_t at=0;
          for(const auto& b:beams) {
            float record[8];for(int k=0;k<3;++k){record[k]=b.origin[k];record[3+k]=b.direction[k];}record[6]=b.range;record[7]=b.hit?1.f:0.f;
            std::memcpy(s.beam_scratch.data()+at,record,32);if(s.with_bodies)std::memcpy(s.beam_scratch.data()+at+32,&b.vehicle_id,4);at+=beam_stride;
            if(b.hit){const auto p=b.origin+b.range*b.direction;float xyz[3]={float(p.x()),float(p.y()),float(p.z())};const auto old=s.scratch.size();s.scratch.resize(old+point_stride);std::memcpy(s.scratch.data()+old,xyz,12);if(s.with_bodies)std::memcpy(s.scratch.data()+old+12,&b.vehicle_id,4);}
          }
        }
      }
#ifdef XSIM_GPU
      else {
        const auto &points =
            renderer->scan(sample.position, sample.orientation,
                           double(sample.stamp) * 1e-9, s.gpu_config);
        s.scratch.resize(points.size() * 16);
        size_t at = 0;
        for (const auto &p : points) {
          float xyzi[4] = {p.x, p.y, p.z, p.intensity};
          std::memcpy(s.scratch.data() + at, xyzi, 16);
          at += 16;
        }
      }
#endif
    } catch (const std::exception &error) {
      ok = false;
      std::lock_guard<std::mutex> l(s.mutex);
      s.error = error.what();
      ++s.errors;
    }
    {
      std::lock_guard<std::mutex> l(s.mutex);
      s.busy = false;
      auto e = s.entity.lock();
      if (ok && e && e->alive && e->enabled &&
          e->generation == sample.key.generation) {
        if (s.has_completed)
          ++s.misses;
        s.completed = sample;
        s.data.swap(s.scratch);
        s.beam_data.swap(s.beam_scratch);
        s.has_completed = true;
        ++s.scans;
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - sample.submitted)
                .count();
        s.latency_ns = elapsed;
        if (elapsed > s.max_latency_ns)
          s.max_latency_ns = elapsed;
      } else
        ++s.misses;
    }
  }
}
bool Sensors::take(const std::shared_ptr<Sensor> &s, Sample &p,
                   std::vector<uint8_t> &out, std::vector<uint8_t> *beams) {
  std::lock_guard<std::mutex> l(s->mutex);
  if (!s->has_completed)
    return false;
  auto e = s->entity.lock();
  s->has_completed = false;
  if (!e || !e->alive || !e->enabled ||
      e->generation != s->completed.key.generation) {
    ++s->misses;
    return false;
  }
  p = s->completed;
  out.swap(s->data);
  if(beams)beams->swap(s->beam_data);
  return true;
}
Json Sensors::status() const {
  Json j = Json::array();
  std::lock_guard<std::mutex> l(mutex_);
  for (const auto &weak : sensors_)
    if (auto s = weak.lock()) {
      auto e = s->entity.lock();
      if (!e || !e->alive)
        continue;
      std::lock_guard<std::mutex> sl(s->mutex);
      j.push_back({{"id", e->id},
                   {"generation", e->generation.load()},
                   {"scans", s->scans.load()},
                   {"misses", s->misses.load()},
                   {"latency_ns", s->latency_ns.load()},
                   {"max_latency_ns", s->max_latency_ns.load()},
                   {"errors", s->errors.load()},
                   {"error", s->error},
                   {"backend", s->gpu ? "gpu" : "cpu"}});
    }
  return j;
}
void Sensors::stop() {
  {
    std::lock_guard<std::mutex> l(mutex_);
    stopping_ = true;
  }
  wake_.notify_all();
  for (auto &t : cpu_threads_)
    if (t.joinable())
      t.join();
  if (gpu_thread_.joinable())
    gpu_thread_.join();
}
} // namespace xsim
