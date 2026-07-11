#ifndef MULTI_PURPOSE_MPC_ROS__PATH_CORE_HPP_
#define MULTI_PURPOSE_MPC_ROS__PATH_CORE_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::path_core
{

/// Segments at or below this length are degenerate for path validation.
inline constexpr double kMinimumSegmentLengthM = 1e-6;

/// One row of the strict reference-path trajectory CSV contract.
struct ReferencePathPoint
{
  double s_m{};
  double x_m{};
  double y_m{};
  double psi_rad{};
  double kappa_radpm{};
  double vx_mps{};
  double ax_mps2{};
};

/// Normalize a finite angle to the equivalent value in [-pi, pi].
double wrap_to_pi(double angle);

/// Load the strict seven-column trajectory CSV contract.
///
/// Throws std::runtime_error when the file or any row violates the contract.
/// CSV diagnostics include the file, one-based row, column, value, and reason.
std::vector<ReferencePathPoint> load_reference_path_csv(const std::string & path);

/// Remove a circular path's closure duplicate as one whole record.
///
/// Returns true when the final record was within tolerance of the first record
/// and was removed. Throws when tolerance is invalid, coordinates are not
/// finite, a non-closure consecutive degenerate edge exists, or fewer than
/// three unique points would remain.
bool normalize_circular_endpoint(
  std::vector<ReferencePathPoint> & points, double tolerance_m);

/// Number of equal subdivisions needed to keep each interval <= resolution.
///
/// The result is max(1, ceil(distance / resolution)); zero distance therefore
/// has one subdivision. Throws for non-finite/negative distance, non-finite or
/// non-positive resolution, and a result that does not fit in std::size_t.
std::size_t subdivision_count(double distance, double resolution);

}  // namespace multi_purpose_mpc_ros::path_core

#endif  // MULTI_PURPOSE_MPC_ROS__PATH_CORE_HPP_
