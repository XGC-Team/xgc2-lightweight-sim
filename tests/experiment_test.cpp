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
  std::cout << cases.size() << " original Go fixtures (noWorld retained as a direct-bootstrap refusal) and strict frozen-fact negative controls passed\n";
}
