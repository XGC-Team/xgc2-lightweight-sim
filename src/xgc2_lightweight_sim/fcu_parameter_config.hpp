#pragma once

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <string>

namespace xgc_lightweight {

// The host serializes the one startup override map as a TOML table. This is
// deliberately limited to numeric FCU parameters, not a second TOML or PX4
// parameter system. The control kernel owns names, bounds and effective values.
template <class Parameters>
void apply_fcu_parameter_config(const std::string& configuration, Parameters& parameters) {
  const auto trim = [](std::string value) {
    const auto first = value.find_first_not_of(" \t\r");
    if (first == std::string::npos) return std::string();
    return value.substr(first, value.find_last_not_of(" \t\r") - first + 1);
  };
  std::istringstream input(configuration);
  std::string line;
  bool inside = false;
  while (std::getline(input, line)) {
    line = trim(line);
    if (line.empty() || line.front() == '#') continue;
    if (line.front() == '[') {
      inside = line == "[fcu_parameters]";
      if (!inside && line.find("[fcu_parameters") == 0)
        throw std::invalid_argument("fcu_parameters must be one flat numeric table");
      continue;
    }
    if (!inside) {
      if (line.find("fcu_parameters") == 0)
        throw std::invalid_argument("fcu_parameters must use a numeric TOML table");
      continue;
    }
    const auto separator = line.find('=');
    if (separator == std::string::npos)
      throw std::invalid_argument("invalid FCU parameter assignment");
    const std::string name = trim(line.substr(0, separator));
    std::string text = trim(line.substr(separator + 1));
    if (const auto comment = text.find('#'); comment != std::string::npos)
      text = trim(text.substr(0, comment));
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || errno == ERANGE ||
        !std::isfinite(value) || !parameters.set(name, value))
      throw std::invalid_argument("unsupported, repeated or invalid FCU parameter: " + name);
  }
}

} // namespace xgc_lightweight
