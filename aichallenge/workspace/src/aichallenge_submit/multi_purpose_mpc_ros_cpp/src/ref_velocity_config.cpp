#include "multi_purpose_mpc_ros_cpp/ref_velocity_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <stdexcept>

namespace multi_purpose_mpc_ros_cpp
{

RefVelocityConfig::RefVelocityConfig(const std::string & path)
{
  if (path.empty()) {
    return;
  }

  const auto root = YAML::LoadFile(path);
  const auto sections = root["ref_vel_configulator"];
  if (!sections || !sections.IsMap()) {
    return;
  }

  for (const auto & item : sections) {
    const auto config = item.second;
    sections_.emplace_back(
      config["wp_id"].as<std::size_t>(),
      config["ref_vel"].as<double>());
  }

  std::sort(
    sections_.begin(), sections_.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.first < rhs.first; });
}

bool RefVelocityConfig::empty() const
{
  return sections_.empty();
}

double RefVelocityConfig::get_ref_vel_kmph(std::size_t current_wp_id) const
{
  if (sections_.empty()) {
    throw std::runtime_error("ref velocity config is empty");
  }

  for (std::size_t i = 0; i < sections_.size(); ++i) {
    const auto start = sections_[i].first;
    const auto end = sections_[(i + 1) % sections_.size()].first;
    const auto target_speed = sections_[i].second;

    if (start <= end) {
      if (start <= current_wp_id && current_wp_id < end) {
        return target_speed;
      }
    } else {
      if (current_wp_id >= start || current_wp_id < end) {
        return target_speed;
      }
    }
  }

  return sections_.front().second;
}

}  // namespace multi_purpose_mpc_ros_cpp
