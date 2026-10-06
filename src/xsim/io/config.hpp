#pragma once
#include "core/entity.hpp"
#include <nlohmann/json.hpp>
namespace xsim {
using Json = nlohmann::json;
Config parse_entity(const Json &);
Json load_config(const std::string &path);
} // namespace xsim
