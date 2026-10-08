#include "io/config.hpp"
#include <cassert>
#include <iostream>
#include <fstream>

int main(int argc, char **argv) {
  assert(argc == 2);
  const std::string root = argv[1];
  const auto cases = xsim::load_json_object(root + "/manifest.json").at("cases");
  for (const auto &entry : cases) {
    const auto name = entry.at("name").get<std::string>();
    xsim::Json input;
    const auto input_path = root + "/" + entry.at("input").get<std::string>();
    // A missing fixture is a test setup failure, never an expected rejection.
    if (!std::ifstream(input_path).good()) {
      std::cerr << name << " missing fixture: " << input_path << '\n';
      return 1;
    }
    try { input = xsim::load_json_object(input_path); }
    catch (const std::exception &) { if (entry.contains("expectedError")) continue; throw; }
    if (entry.contains("expectedConfig")) {
      const auto expected = xsim::load_json_object(root + "/" + entry.at("expectedConfig").get<std::string>());
      const auto actual = xsim::experiment_config(input);
      if (expected != actual) {
        std::cerr << name << "\nexpected " << expected.dump(2) << "\nactual " << actual.dump(2) << '\n';
        return 1;
      }
    } else {
      bool rejected = false;
      try { (void)xsim::experiment_config(input); } catch (const std::exception &) { rejected = true; }
      if (!rejected) { std::cerr << name << " was accepted\n"; return 1; }
    }
  }
  auto original = xsim::load_json_object(root + "/default-mixed.input.json");
  for (const char *key : {"openingRunId", "openingAcceptedAtEpochNs", "containerizedDeployment", "visualizationTopics"}) {
    auto modified = original; modified["context"].erase(key);
    bool rejected = false;
    try { (void)xsim::experiment_config(modified); } catch (const std::exception &) { rejected = true; }
    assert(rejected);
  }
  for (const char *value : {"0", "01", "-1", "1.0", "9223372036854775808"}) {
    auto modified = original; modified["epochNs"] = value; modified["context"]["openingAcceptedAtEpochNs"] = value;
    bool rejected = false;
    try { (void)xsim::experiment_config(modified); } catch (const std::exception &) { rejected = true; }
    assert(rejected);
  }
  auto missing_raw = original; missing_raw["robots"][0].erase("authoredSimulationSensors");
  bool rejected = false;
  try { (void)xsim::experiment_config(missing_raw); } catch (const std::exception &) { rejected = true; }
  assert(rejected);
  auto omitted_selection = original;
  omitted_selection["robots"][0]["authoredSimulationSensors"] = xsim::Json::object();
  const auto no_sensor = xsim::experiment_config(omitted_selection);
  assert(!no_sensor["entities"][0].contains("sensor"));
  auto long_names = original;
  const std::string large_segment(100000, 'a');
  long_names["robots"][0]["namespace"] = "/" + large_segment;
  long_names["robots"][0]["simulationPoseTopic"] = "/" + large_segment + "/pose";
  const auto large_config = xsim::experiment_config(long_names);
  assert(large_config["entities"][0]["name"] == large_segment);
  for (const char *invalid : {"/", "/1bad", "/valid/", "/valid//pose", "/valid/1pose", "/valid/pose-1"}) {
    auto modified = original; modified["robots"][0]["simulationPoseTopic"] = invalid;
    bool refused = false;
    try { (void)xsim::experiment_config(modified); } catch (const std::exception &) { refused = true; }
    assert(refused);
  }
  const auto frozen_before = original.dump();
  const auto config_before = xsim::experiment_config(original).dump();
  const xsim::Json expected_bindings{{"ugv1", "scout-1"}, {"uav1", "px4-1"},
    {"uav2", "px4-2"}, {"mecanum1", "mecanum-1"}};
  assert(xsim::experiment_robot_bindings(original) == expected_bindings);
  assert(original.dump() == frozen_before);
  assert(xsim::experiment_config(original).dump() == config_before);

  auto explicit_identity = original;
  explicit_identity["robots"][0]["id"] = "Robot:Case_01.rev-2";
  const auto identities = xsim::experiment_robot_bindings(explicit_identity);
  assert(identities.at("ugv1") == "Robot:Case_01.rev-2");
  assert(!identities.contains("Robot:Case_01.rev-2"));
  assert(xsim::experiment_config(explicit_identity).dump() == config_before);
  explicit_identity["robots"][0]["id"] = std::string(128, 'a');
  assert(xsim::experiment_robot_bindings(explicit_identity).at("ugv1") == std::string(128, 'a'));

  const auto reject_binding = [](const xsim::Json &input) {
    bool refused = false;
    try { (void)xsim::experiment_robot_bindings(input); }
    catch (const std::exception &) { refused = true; }
    assert(refused);
  };
  for (const auto &invalid : xsim::Json::array({nullptr, 42, "", "robot/name", "robot name", "robot\nname", "机器人", std::string(129, 'a')})) {
    auto modified = original; modified["robots"][0]["id"] = invalid;
    reject_binding(modified);
  }
  auto missing_id = original; missing_id["robots"][0].erase("id");
  reject_binding(missing_id);
  auto duplicate_id = original; duplicate_id["robots"][1]["id"] = duplicate_id["robots"][0]["id"];
  reject_binding(duplicate_id);
  auto duplicate_name = original; duplicate_name["robots"][1]["namespace"] = duplicate_name["robots"][0]["namespace"];
  reject_binding(duplicate_name);
  for (const char *invalid : {"", "ugv1", "/1bad", "/two/segments"}) {
    auto modified = original; modified["robots"][0]["namespace"] = invalid;
    reject_binding(modified);
  }
  auto hybrid = xsim::load_json_object(root + "/hybrid-filter-physical-sensor.input.json");
  const xsim::Json hybrid_bindings{{"ugv1", "scout-1"}, {"mecanum1", "mecanum-1"}};
  assert(xsim::experiment_robot_bindings(hybrid) == hybrid_bindings);
  // Physical members are outside this world's identity map and are not repaired.
  hybrid["robots"][1].erase("id");
  hybrid["robots"][1].erase("namespace");
  assert(xsim::experiment_robot_bindings(hybrid) == hybrid_bindings);
  auto invalid_source = hybrid; invalid_source["robots"][0]["hybridSource"] = "unknown";
  reject_binding(invalid_source);
  auto invalid_mode = original; invalid_mode["context"]["runMode"] = "unknown";
  reject_binding(invalid_mode);
  const auto simulation_physical = xsim::load_json_object(root + "/simulation-retains-physical-hybrid-flags.input.json");
  assert(xsim::experiment_robot_bindings(simulation_physical) == expected_bindings);
  auto empty = original; empty["robots"] = xsim::Json::array();
  assert(xsim::experiment_robot_bindings(empty) == xsim::Json::object());
  std::cout << cases.size() << " original Go fixtures (noWorld retained as a direct-bootstrap refusal) and strict frozen-fact negative controls passed\n";
  std::cout << "frozen public robot identities, hybrid filtering and unchanged scientific config passed\n";
}
