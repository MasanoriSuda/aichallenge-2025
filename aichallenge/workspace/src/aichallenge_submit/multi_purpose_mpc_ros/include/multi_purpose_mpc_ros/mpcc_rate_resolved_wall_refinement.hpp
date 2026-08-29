#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_WALL_REFINEMENT_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_WALL_REFINEMENT_HPP_

#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
#include "multi_purpose_mpc_ros/recovery_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_wall_refinement
{

struct Request;
struct Result;

/// Bounded numerical cache for immutable static-wall scan evidence.
///
/// The cache is deliberately owned by one SolverContext.  It contains no
/// trajectory, Mission or authority state: a hit only returns the exact clear
/// runs produced by a previous query with an identical grid and geometry key.
class Cache
{
public:
  explicit Cache(std::size_t maximum_entries = 4096U);
  ~Cache();

  Cache(const Cache &) = delete;
  Cache & operator=(const Cache &) = delete;
  Cache(Cache &&) = delete;
  Cache & operator=(Cache &&) = delete;

  void clear() noexcept;
  std::size_t size() const noexcept;
  std::size_t maximum_entries() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  friend Result resolve(const Request & request, Cache * cache) noexcept;
};

enum class PreRefinementLateralSupportReason
{
  NotRequested,
  InvalidInput,
  Accepted,
};

const char * to_string(PreRefinementLateralSupportReason reason) noexcept;

struct PreRefinementLateralSupportRequest
{
  bool active{false};
  double initial_lateral_m{};
  std::vector<double> wall_lower_m;
  std::vector<double> wall_upper_m;
};

struct PreRefinementLateralSupport
{
  bool valid{false};
  bool applied{false};
  PreRefinementLateralSupportReason reason{
    PreRefinementLateralSupportReason::NotRequested};
  double lower_m{};
  double upper_m{};
};

/// Resolve the union support used only by the first sequential-convexification
/// solve. A stage-indexed wall interval belongs to its progress sample and
/// must not be imposed as a state box before progress itself has been solved.
/// The subsequent progress-aligned and physical refinements remain the sole
/// executable wall constraints.
PreRefinementLateralSupport resolve_pre_refinement_lateral_support(
  const PreRefinementLateralSupportRequest & request) noexcept;

/// One first-solve state together with the convex bounds that may be tightened
/// by the physical wall refinement. Progress is local to course_progress_origin_m.
struct StageRequest
{
  double solved_progress_m{};
  double solved_lateral_m{};
  double solved_lag_m{};
  double solved_heading_offset_rad{};
  double lateral_lower_m{};
  double lateral_upper_m{};
  double lag_lower_m{};
  double lag_upper_m{};
  double heading_lower_rad{};
  double heading_upper_rad{};
  double progress_lower_m{};
  double progress_upper_m{};
};

struct Request
{
  bool active{false};
  const recovery_footprint::OccupancyGrid * wall_grid{};
  /// Immutable content identity for wall_grid.  Zero asks resolve() to derive
  /// it from the grid before using the cache.
  std::uint64_t wall_grid_fingerprint{};
  recovery_footprint::FootprintExtents footprint;
  std::vector<mpc_stage_geometry::CourseFrameKnot> course_frame_knots;
  double course_progress_origin_m{};
  StageRequest initial_stage;
  std::vector<StageRequest> stages;
  double heading_bucket_width_rad{0.025};
  double translation_bucket_width_m{};
  double lateral_sample_step_m{};
  double boundary_guard_m{0.001};
};

enum class Reason
{
  NotRequested,
  InvalidInput,
  CourseFrameUnavailable,
  WallIntervalUnavailable,
  EmptyTrustRegion,
  Accepted,
};

const char * to_string(Reason reason) noexcept;

struct StageBounds
{
  double lateral_lower_m{};
  double lateral_upper_m{};
  double lag_lower_m{};
  double lag_upper_m{};
  double heading_lower_rad{};
  double heading_upper_rad{};
  double progress_lower_m{};
  double progress_upper_m{};
  double heading_bucket_center_rad{};
  double lag_bucket_center_m{};
  double progress_bucket_center_m{};
};

struct SweptLateralConstraint
{
  int transition_stage{-1};
  double destination_ratio{};
  double lateral_lower_m{};
  double lateral_upper_m{};
};

struct Result
{
  bool valid{false};
  bool feasible{false};
  bool applied{false};
  Reason reason{Reason::NotRequested};
  int first_failure_stage{-1};
  std::size_t cache_hit_count{};
  std::size_t cache_miss_count{};
  /// New footprint poses evaluated in this call. checked_pose_count retains
  /// the proof-equivalent count even when the clear runs came from cache.
  std::size_t cache_scanned_pose_count{};
  std::size_t checked_pose_count{};
  std::vector<StageBounds> stages;
  std::vector<SweptLateralConstraint> swept_lateral_constraints;
  std::string detail{"not-requested"};
};

/// Resolve a conservative convex trust region around a first MPCC solution.
///
/// The wall scan is evaluated at quantized progress/lag/heading centres. The
/// footprint is expanded by the maximum translation and corner motion within
/// those buckets, so every second-solve state inside the returned bounds uses
/// the same physical wall evidence. This closes the former contract gap where
/// a lateral interval certified at a nominal heading was used by an optimizer
/// whose heading and lag states remained unconstrained.
Result resolve(const Request & request, Cache * cache = nullptr) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_wall_refinement

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_WALL_REFINEMENT_HPP_
