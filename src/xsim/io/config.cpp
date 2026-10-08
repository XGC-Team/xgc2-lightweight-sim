#include "config.hpp"
#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <set>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
#include <yaml-cpp/yaml.h>
namespace xsim {
namespace {
Eigen::Vector3d vec3(const Json &v) {
  if (!v.is_array() || v.size() != 3)
    throw std::invalid_argument("expected three numbers");
  Eigen::Vector3d r(v[0].get<double>(), v[1].get<double>(), v[2].get<double>());
  if (!r.allFinite())
    throw std::invalid_argument("nonfinite vector");
  return r;
}
Json yaml_json(const YAML::Node &n) {
  if(n.IsNull())return nullptr;
  if(n.IsSequence()){Json j=Json::array();for(const auto& x:n)j.push_back(yaml_json(x));return j;}
  if(n.IsMap()){Json j=Json::object();for(const auto& x:n)j[x.first.as<std::string>()]=yaml_json(x.second);return j;}
  auto text=n.as<std::string>();
  if(text=="true")return true;if(text=="false")return false;
  try {size_t end=0;double value=std::stod(text,&end);if(end==text.size())return value;}catch(const std::exception&){}
  return text;
}
} // namespace
Config parse_entity(const Json &j) {
  Config c;
  c.name = j.at("name").get<std::string>();
  if (c.name.empty() ||
      c.name.find_first_not_of(
          "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") !=
          std::string::npos)
    throw std::invalid_argument("name must be a ROS namespace segment");
  const auto kind = j.at("kind").get<std::string>();
  if (kind == "fs150")
    c.kind = Kind::FS150;
  else if (kind == "scout")
    c.kind = Kind::Scout;
  else if (kind == "mecanum")
    c.kind = Kind::Mecanum;
  else
    throw std::invalid_argument("unknown kind");
  if (j.contains("position"))
    c.initial = vec3(j.at("position"));
  c.yaw = j.value("yaw", 0.0);
  c.ground_z = j.value("ground_z", 0.0);
  if (!std::isfinite(c.yaw) || !std::isfinite(c.ground_z))
    throw std::invalid_argument("nonfinite pose");
  if (j.contains("local_origin"))
    c.local_origin = vec3(j.at("local_origin"));
  if (j.contains("fcu_parameters")) {
    if (c.kind != Kind::FS150)
      throw std::invalid_argument("FCU parameters require fs150");
    for (auto i = j.at("fcu_parameters").begin();
         i != j.at("fcu_parameters").end(); ++i)
      if (!c.fcu.set(i.key(), i.value().get<double>()))
        throw std::invalid_argument("invalid FCU parameter: " + i.key());
  }
  const auto ros = j.value("ros", Json::object());
  for (const char *key : {"provider_service", "truth_topic", "reset_service",
                          "mocap_topic", "mocap_velocity_topic"})
    if (ros.contains(key))
      throw std::invalid_argument("unsupported ROS configuration: " + std::string(key));
  if (c.kind != Kind::FS150)
    for (const char *key : {"pose_topic", "velocity_topic", "odometry_topic"})
      if (ros.contains(key))
        throw std::invalid_argument("UGV ROS configuration is not supported: " + std::string(key));
  return c;
}
Json load_config(const std::string &path, const std::string &scene_file) {
  constexpr std::size_t limit = 1024 * 1024;
  struct File {
    int fd;
    ~File() { if (fd >= 0) close(fd); }
  } file{open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK)};
  if (file.fd < 0) throw std::runtime_error("cannot read configuration without symbolic links");
  struct stat metadata{};
  if (fstat(file.fd, &metadata) != 0 || !S_ISREG(metadata.st_mode) ||
      metadata.st_size <= 0 || metadata.st_size > static_cast<off_t>(limit))
    throw std::invalid_argument("configuration must be a nonempty regular file <=1 MiB");
  std::string contents;
  contents.reserve(static_cast<std::size_t>(metadata.st_size));
  char buffer[8192];
  for (;;) {
    const auto count = read(file.fd, buffer, std::min(sizeof(buffer), limit + 1 - contents.size()));
    if (count < 0 && errno == EINTR) continue;
    if (count < 0) throw std::runtime_error("cannot read configuration");
    if (count == 0) break;
    contents.append(buffer, static_cast<std::size_t>(count));
    if (contents.size() > limit) throw std::invalid_argument("configuration exceeds 1 MiB");
  }
  std::vector<std::set<std::string>> object_keys;
  const auto unique_keys = [&object_keys](int, Json::parse_event_t event, Json &value) {
    if (event == Json::parse_event_t::object_start) object_keys.emplace_back();
    else if (event == Json::parse_event_t::object_end) object_keys.pop_back();
    else if (event == Json::parse_event_t::key &&
             !object_keys.back().insert(value.get<std::string>()).second)
      throw std::invalid_argument("duplicate configuration key: " + value.get<std::string>());
    return true;
  };
  Json j = Json::parse(contents, unique_keys);
  if (!j.is_object()) throw std::invalid_argument("configuration must be one JSON object");
  if (!scene_file.empty()) {
    const auto configured = j.value("scene_file", std::string{});
    if (!configured.empty() && configured != scene_file)
      throw std::invalid_argument("--scene-file conflicts with configuration scene_file");
    j["scene_file"] = scene_file;
  }
  if(j.contains("scene_file") && !j.at("scene_file").get<std::string>().empty())
    j["scene"]["document"]=yaml_json(YAML::LoadFile(j.at("scene_file").get<std::string>()));
  return j;
}
} // namespace xsim
