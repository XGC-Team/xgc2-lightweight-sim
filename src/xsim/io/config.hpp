#pragma once
#include "core/entity.hpp"
#include <nlohmann/json.hpp>
namespace xsim {
using Json = nlohmann::json;
Config parse_entity(const Json &);
Json load_json_object(const std::string &path);
Json resolve_scene_file(Json config, const std::string &scene_file);
Json experiment_config(const Json &input);
Json load_experiment_config(const std::string &path, const std::string &scene_file = "");
Json load_config(const std::string &path, const std::string &scene_file = "");
} // namespace xsim
