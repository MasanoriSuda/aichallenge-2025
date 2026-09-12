#pragma once
#include "multi_purpose_mpc_ros/mpcc_vehicle_model.hpp"
#include <cstddef>
#include <cstdint>
#include <memory>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model {
namespace numerical {
struct CompiledInputMapData;
struct CompiledInputMapAccess;
} // namespace numerical

/// Immutable local mean-value equations from an independently propagated
/// starting domain. These are numerical acceleration data, never a trajectory,
/// current-world proof or command authority. Only contained native state/input
/// branches may use them; every other branch is calculated afresh.
class CompiledInputMaps {
public:
  std::size_t size() const noexcept;
  std::size_t storage_bytes() const noexcept;
  std::uint64_t source_domain_fingerprint() const noexcept;
  bool matches(const Parameters &parameters) const noexcept;

private:
  explicit CompiledInputMaps(
      std::shared_ptr<const numerical::CompiledInputMapData> data)
      : data_(std::move(data)) {}
  std::shared_ptr<const numerical::CompiledInputMapData> data_;
  friend struct numerical::CompiledInputMapAccess;
};
} // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
