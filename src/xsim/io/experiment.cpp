#include "config.hpp"
#include <charconv>
#include <limits>
#include <regex>
#include <set>

namespace xsim {
namespace {
void require(bool valid, const std::string &message) {
  if (!valid) throw std::invalid_argument("experiment: " + message);
}
std::string text(const Json &j, const char *key) { return j.at(key).get<std::string>(); }
double number(const Json &v) {
  require(v.is_number(), "expected a finite number");
  const auto n = v.get<double>();
  require(std::isfinite(n), "expected a finite number");
  return n;
}
int64_t epoch(const std::string &value) {
  int64_t result{};
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  require(parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size() &&
          result > 0 && std::to_string(result) == value, "epoch must be a positive canonical int64 decimal string");
  return result;
}
int64_t timing(const Json &parameters, const char *key, int64_t fallback) {
  if (!parameters.contains(key)) return fallback * 1000000;
  const auto &v = parameters.at(key);
  require(v.is_number_integer(), std::string(key) + " must be whole milliseconds");
  require(!v.is_number_unsigned() || v.get<uint64_t>() <= uint64_t(INT64_MAX), "timing overflow");
  const auto ms = v.get<int64_t>();
  require(ms > 0 && ms <= INT64_MAX / 1000000, "timing must be positive and representable in nanoseconds");
  require(std::string(key) != "modelStepMs" || ms <= 10, "modelStepMs exceeds 10ms");
  return ms * 1000000;
}
bool reference_requested(const Json &groups) {
  require(groups.is_array(), "visualizationTopics declarations are required");
  for (const auto &group : groups) {
    // The historical Go reader skipped the entire preset if any typed field
    // could not decode. Keep that boundary while interpreting only declarations.
    if (group.is_null()) continue;
    if (!group.is_array()) continue;
    bool valid = true, requested = false;
    for (const auto &topic : group) {
      if (topic.is_null()) continue;
      if (!topic.is_object()) { valid = false; break; }
      for (const char *key : {"topic", "messageType", "role"})
        if (topic.contains(key) && !topic.at(key).is_null() && !topic.at(key).is_string()) valid = false;
      for (const char *key : {"visible3d", "visibleAr"})
        if (topic.contains(key) && !topic.at(key).is_null() && !topic.at(key).is_boolean()) valid = false;
      if (!valid) break;
      const auto string_value = [&](const char *key) {
        return topic.contains(key) && topic.at(key).is_string() ? topic.at(key).get<std::string>() : std::string{};
      };
      const auto bool_value = [&](const char *key) { return topic.contains(key) && topic.at(key).is_boolean() && topic.at(key).get<bool>(); };
      requested = requested || (string_value("topic") == "/xgc/scene/reference_cloud" &&
        string_value("messageType") == "sensor_msgs/PointCloud2" && string_value("role") == "semantic" &&
        (bool_value("visible3d") || bool_value("visibleAr")));
    }
    if (valid && requested) return true;
  }
  return false;
}
Json sensor(const Json &authored, const std::string &ns, const std::string &backend) {
  require(authored.is_object() && authored.contains("simpleLidar"), "authoredSimulationSensors.simpleLidar is required");
  require(authored.size() == 1, "unknown authored simulation sensor field");
  const auto &raw = authored.at("simpleLidar");
  Json lidar = raw;
  if (raw.is_boolean()) lidar = Json{{"enabled", raw.get<bool>()}};
  require(lidar.is_object(), "authored simpleLidar must be boolean or object");
  const std::set<std::string> allowed{"enabled", "mode", "preset", "acceleration", "rateHz", "rangeMeters", "hFovDeg", "vFovDeg", "hRes", "vRes", "publishBeams"};
  for (auto i = lidar.begin(); i != lidar.end(); ++i) require(allowed.count(i.key()) != 0, "unknown authored lidar field: " + i.key());
  const auto acceleration = lidar.value("acceleration", std::string{});
  require(acceleration.empty() || acceleration == "cpu" || acceleration == "gpu", "unsupported authored acceleration");
  const auto enabled = lidar.value("enabled", false);
  (void)lidar.value("publishBeams", false);
  for (const auto &bound : {std::pair<const char *, double>{"rateHz", 100}, {"rangeMeters", 200}, {"hFovDeg", 360}, {"vFovDeg", 180}})
    if (lidar.contains(bound.first)) {
      const auto value = number(lidar.at(bound.first));
      require(value >= 0 && value <= bound.second, "authored lidar numeric bound exceeded");
    }
  for (const char *key : {"hRes", "vRes"})
    if (lidar.contains(key)) {
      require(lidar.at(key).is_number_integer(), "authored lidar resolution must be integer");
      require(!lidar.at(key).is_number_unsigned() || lidar.at(key).get<uint64_t>() <= 4096, "lidar resolution bound exceeded");
      const auto value = lidar.at(key).get<int64_t>();
      require(value >= 0 && value <= 4096 && (lidar.value("mode", std::string{}) == "penetrating" || value != 1), "authored lidar resolution bound exceeded");
    }
  const auto mode_authored = lidar.value("mode", std::string{});
  auto preset = lidar.value("preset", std::string{});
  require(mode_authored.empty() || mode_authored == "raycast" || mode_authored == "penetrating" || mode_authored == "depth_frustum", "unsupported lidar mode");
  require(preset.empty() || preset == "bridge_equivalent" || preset == "zju_cpu_crop" || preset == "zju_cpu_crop_ego_v2", "unsupported lidar preset");
  if (!enabled) return nullptr;
  if (mode_authored.empty() && preset.empty()) preset = "bridge_equivalent";
  std::string mode = "raycast";
  double rate = 10, range = 20, hfov = 360, vfov = 30, heading_cos = 0, slab = 0;
  int hres = 360, vres = 32;
  bool buried = false, crop = false;
  if (!preset.empty()) {
    mode = "penetrating"; vfov = 180; buried = true; range = 5;
    if (preset == "bridge_equivalent") range = 8;
    else { crop = true; slab = 0.5773502691896258; if (preset == "zju_cpu_crop") heading_cos = 0.5; }
  }
  if (!mode_authored.empty()) mode = mode_authored;
  if (mode == "depth_frustum") { hfov = 90; vfov = 60; }
  for (auto pair : {std::pair<const char *, double *>{"rateHz", &rate}, {"rangeMeters", &range}, {"hFovDeg", &hfov}, {"vFovDeg", &vfov}})
    if (lidar.contains(pair.first)) { const auto v = number(lidar.at(pair.first)); if (v > 0) *pair.second = v; }
  for (auto pair : {std::pair<const char *, int *>{"hRes", &hres}, {"vRes", &vres}})
    if (lidar.contains(pair.first)) {
      require(lidar.at(pair.first).is_number_integer(), "lidar resolution must be integer");
      const auto v = lidar.at(pair.first).get<int64_t>();
      require(v >= 0 && v <= 4096, "lidar resolution exceeds authored bounds");
      if (v > 0) *pair.second = static_cast<int>(v);
    }
  const bool beams = lidar.value("publishBeams", false);
  require(!(beams && backend == "cpu" && mode == "penetrating"), "penetrating has no rays for beams");
  require(!(beams && backend == "gpu"), "GPU lidar_scan does not produce beams");
  if (mode == "depth_frustum") mode = "depth";
  if (backend == "gpu") mode = "lidar_scan";
  Json result{{"backend", backend}, {"mode", mode}, {"topic", ns + "/simple_lidar/points"}, {"frame", "world"},
    {"rate_hz", rate}, {"range", range}, {"h_fov_deg", hfov}, {"v_fov_deg", vfov}, {"h_res", hres}, {"v_res", vres},
    {"surface_spacing", 0.1}, {"keep_buried", buried}, {"heading_crop", crop}, {"heading_cos_min", heading_cos},
    {"vertical_slab_tan", slab}, {"publish_beams", beams}, {"world_bodies", backend == "cpu" && (mode == "raycast" || mode == "depth")}};
  if (backend == "gpu") { result["min_range"] = 0.1; result["point_cover_spacing"] = 0.1; }
  return result;
}
} // namespace
Json experiment_config(const Json &input) {
  require(input.is_object(), "bootstrap must be one object");
  const auto &context = input.at("context");
  const auto instance = text(input, "instanceId");
  const auto epoch_text = text(input, "epochNs");
  const auto epoch_ns = epoch(epoch_text);
  require(!instance.empty() && instance == text(context, "openingRunId") && epoch_text == text(context, "openingAcceptedAtEpochNs"), "bootstrap does not match the accepted opening Run");
  require(context.at("containerizedDeployment").is_boolean() && !context.at("containerizedDeployment").get<bool>(), "containerized deployments are not supported");
  const auto run_mode = text(context, "runMode");
  require(run_mode == "simulation" || run_mode == "hybrid", "unsupported runMode");
  require(text(context.at("scene"), "simulator") == "xsim", "scene simulator must be xsim");
  require(text(context.at("simulation"), "placement") == "core", "xsim must be centralized on Core");
  const auto &robots = input.at("robots");
  require(robots.is_array() && robots.size() <= 256, "robots must be the frozen roster");
  const auto &settings = input.at("settings");
  require(settings.is_object() && settings.value("autoStartGazeboServer", false), "simulated world start must be enabled");
  const auto parameters = context.at("scene").value("parameters", Json::object());
  require(parameters.is_object(), "scene parameters must be an object");
  for (auto i = parameters.begin(); i != parameters.end(); ++i)
    require(i.key() == "modelStepMs" || i.key() == "outputPeriodMs" || i.key() == "commandPollMs" ||
      i.key() == "pointCloudBackend" || i.key() == "fcuParameters" || i.key() == "generation", "unknown scene parameter: " + i.key());
  if (parameters.contains("generation")) {
    require(parameters.at("generation").is_object(), "generation parameters must be a numeric object");
    for (const auto &v : parameters.at("generation")) (void)number(v);
  }
  const auto step = timing(parameters, "modelStepMs", 2);
  const auto output = timing(parameters, "outputPeriodMs", 8);
  const auto poll = timing(parameters, "commandPollMs", 5);
  require(step <= INT64_MAX - epoch_ns && output <= INT64_MAX - epoch_ns, "Session epoch plus timing overflows");
  const auto backend = parameters.value("pointCloudBackend", std::string("cpu"));
  require(backend == "cpu" || backend == "gpu", "pointCloudBackend must be cpu or gpu");
  const auto fcu = parameters.value("fcuParameters", Json::object());
  require(fcu.is_object(), "fcuParameters must be a numeric object");
  FlightControllerParameters fcu_validator;
  for (auto i = fcu.begin(); i != fcu.end(); ++i) require(fcu_validator.set(i.key(), number(i.value())), "invalid FCU parameter: " + i.key());
  if (fcu.contains("MPC_THR_MIN") && fcu.contains("MPC_THR_MAX"))
    require(number(fcu.at("MPC_THR_MIN")) <= number(fcu.at("MPC_THR_MAX")), "FCU MPC_THR_MIN exceeds MPC_THR_MAX");
  Json config{{"instance_id", instance}, {"epoch_ns", epoch_ns}, {"model_step_ns", step}, {"output_period_ns", output},
    {"input_poll_ns", poll}, {"publish_clock", true}, {"scene", Json::object()}, {"entities", Json::array()}};
  std::set<std::string> names;
  const std::regex body_pattern("^/[A-Za-z][A-Za-z0-9_]*$");
  const std::regex topic_pattern("^/[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*$");
  for (const auto &robot : robots) {
    if (run_mode == "hybrid") {
      const auto source = text(robot, "hybridSource");
      require(source == "simulation" || source == "physical", "invalid Hybrid source");
      if (source != "simulation") continue;
    }
    const auto ns = text(robot, "namespace");
    require(std::regex_match(ns, body_pattern), "namespace must be one absolute ROS segment");
    const auto body = ns.substr(1);
    require(names.insert(body).second, "duplicate entity namespace");
    const auto kind = text(robot, "kind");
    std::string model;
    if (kind == "px4_multirotor") {
      const auto &px4 = robot.at("px4");
      require(text(px4, "modelId") == "fs150", "xsim models FS150 only");
      require(!px4.contains("simulationSetup") || px4.at("simulationSetup").is_null(), "robot container simulation is unsupported");
      if (px4.contains("simulationRuntime") && !px4.at("simulationRuntime").is_null())
        require(text(px4.at("simulationRuntime"), "placement") == "core", "onboard simulation is unsupported");
      require(!px4.value("imageSimulationEnabled", false), "xsim has no camera producer");
      model = "fs150";
    } else if (kind == "scout_mini") {
      const auto &scout = robot.at("scout");
      require(!scout.value("lidarSimulationEnabled", false), "xsim has no 2D scan producer");
      require(!scout.value("imageSimulationEnabled", false), "xsim has no camera/depth producer");
      model = "scout";
    } else if (kind == "mecanum_ugv") model = "mecanum";
    else throw std::invalid_argument("experiment: unsupported robot kind: " + kind);
    const auto &pose = robot.at("initialPose");
    Json position = Json::array({number(pose.at("x")), number(pose.at("y")), number(pose.at("z"))});
    const auto yaw = number(pose.at("yaw"));
    uint32_t seed = 2166136261u;
    for (unsigned char byte : body) { seed ^= byte; seed *= 16777619u; }
    const auto pose_topic = robot.value("simulationPoseTopic", std::string{});
    const auto twist_topic = robot.value("simulationTwistTopic", std::string{});
    const auto resolved_pose = pose_topic.empty() ? ns + "/pose" : pose_topic;
    const auto resolved_twist = twist_topic.empty() ? ns + "/twist" : twist_topic;
    require(std::regex_match(resolved_pose, topic_pattern) && std::regex_match(resolved_twist, topic_pattern), "invalid localization topic");
    Json entity{{"name", body}, {"kind", model}, {"position", position}, {"yaw", yaw}, {"ros", Json{
      {"mocap_noise", Json::array({1e-7, 1e-7, 1e-7})}, {"mocap_seed", seed & 0x7fffffffu},
      {"localization_pose_topic", resolved_pose}, {"localization_twist_topic", resolved_twist}}}};
    if (model == "fs150" && !fcu.empty()) entity["fcu_parameters"] = fcu;
    const auto desired_sensor = sensor(robot.at("authoredSimulationSensors"), ns, backend);
    if (!desired_sensor.is_null()) entity["sensor"] = desired_sensor;
    (void)parse_entity(entity);
    config["entities"].push_back(std::move(entity));
  }
  if (reference_requested(context.at("visualizationTopics"))) config["reference_cloud_topic"] = "/xgc/scene/reference_cloud";
  return config;
}
Json load_experiment_config(const std::string &path, const std::string &scene_file) {
  return resolve_scene_file(experiment_config(load_json_object(path)), scene_file);
}
} // namespace xsim
