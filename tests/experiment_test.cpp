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
    try { input = xsim::load_json_object(root + "/" + entry.at("input").get<std::string>()); }
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
  std::cout << cases.size() << " original Go fixtures (noWorld retained as a direct-bootstrap refusal) and strict frozen-fact negative controls passed\n";
}
