#include "config.hpp"
#include <fstream>
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
  for (const char *key : {"provider_service", "truth_topic", "reset_service"})
    if (ros.contains(key))
      throw std::invalid_argument("unsupported ROS configuration: " + std::string(key));
  if (c.kind != Kind::FS150)
    for (const char *key : {"pose_topic", "velocity_topic", "odometry_topic"})
      if (ros.contains(key))
        throw std::invalid_argument("UGV ROS configuration is not supported: " + std::string(key));
  return c;
}
Json load_config(const std::string &path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot read configuration");
  Json j;
  in >> j;
  if(j.contains("scene_file") && !j.at("scene_file").get<std::string>().empty())
    j["scene"]["document"]=yaml_json(YAML::LoadFile(j.at("scene_file").get<std::string>()));
  return j;
}
} // namespace xsim
