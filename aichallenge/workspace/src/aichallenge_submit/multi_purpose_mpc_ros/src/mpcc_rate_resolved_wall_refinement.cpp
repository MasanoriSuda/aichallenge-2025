#include "multi_purpose_mpc_ros/mpcc_rate_resolved_wall_refinement.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <list>
#include <limits>
#include <mutex>
#include <sstream>
#include <unordered_map>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_wall_refinement
{

struct Cache::Impl
{
  struct Key
  {
    std::uint64_t wall_grid_fingerprint{};
    // Lateral scan bounds deliberately do not belong to the geometric key.
    // One entry owns clear-run evidence for the widest range evaluated so
    // far; a later subset query is proof-equivalent and an expansion causes
    // a scan of the union.  This matches the controller-side static-wall
    // cache instead of turning every receding bound change into a miss.
    std::array<double, 11U> geometry{};

    bool operator==(const Key & other) const noexcept
    {
      return
        wall_grid_fingerprint == other.wall_grid_fingerprint &&
        geometry == other.geometry;
    }
  };

  struct KeyHash
  {
    std::size_t operator()(const Key & key) const noexcept
    {
      std::uint64_t hash = 1469598103934665603ULL;
      const auto append = [&hash](const std::uint64_t value) {
          constexpr std::uint64_t prime = 1099511628211ULL;
          for (std::size_t byte = 0U; byte < sizeof(value); ++byte) {
            hash ^= (value >> (8U * byte)) & 0xffU;
            hash *= prime;
          }
        };
      append(key.wall_grid_fingerprint);
      for (const double value : key.geometry) {
        std::uint64_t bits{};
        static_assert(sizeof(bits) == sizeof(value));
        std::memcpy(&bits, &value, sizeof(bits));
        append(bits);
      }
      return static_cast<std::size_t>(hash);
    }
  };

  struct Entry
  {
    double searched_lower_m{};
    double searched_upper_m{};
    recovery_footprint::LateralClearRunsResult runs;
    std::list<Key>::iterator lru;
  };

  struct Lookup
  {
    std::optional<recovery_footprint::LateralClearRunsResult> runs;
    double evaluation_lower_m{};
    double evaluation_upper_m{};
  };

  explicit Impl(const std::size_t requested_maximum_entries)
  : maximum_entries(std::max<std::size_t>(1U, requested_maximum_entries))
  {
  }

  Lookup lookup(
    const Key & key, const double requested_lower_m,
    const double requested_upper_m)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto found = entries.find(key);
    if (found == entries.end()) {
      return Lookup{std::nullopt, requested_lower_m, requested_upper_m};
    }
    constexpr double tolerance_m = 1e-12;
    const double evaluation_lower_m = std::min(
      requested_lower_m, found->second.searched_lower_m);
    const double evaluation_upper_m = std::max(
      requested_upper_m, found->second.searched_upper_m);
    if (
      found->second.searched_lower_m <= requested_lower_m + tolerance_m &&
      found->second.searched_upper_m + tolerance_m >= requested_upper_m)
    {
      lru.splice(lru.end(), lru, found->second.lru);
      return Lookup{
        found->second.runs, evaluation_lower_m, evaluation_upper_m};
    }
    return Lookup{std::nullopt, evaluation_lower_m, evaluation_upper_m};
  }

  void store(
    const Key & key, const double searched_lower_m,
    const double searched_upper_m,
    const recovery_footprint::LateralClearRunsResult & runs)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto existing = entries.find(key);
    if (existing != entries.end()) {
      // A concurrent wider result, if one ever exists, must not be narrowed.
      if (
        existing->second.searched_lower_m <= searched_lower_m &&
        existing->second.searched_upper_m >= searched_upper_m)
      {
        lru.splice(lru.end(), lru, existing->second.lru);
        return;
      }
      existing->second.searched_lower_m = searched_lower_m;
      existing->second.searched_upper_m = searched_upper_m;
      existing->second.runs = runs;
      lru.splice(lru.end(), lru, existing->second.lru);
      return;
    }
    if (entries.size() >= maximum_entries && !lru.empty()) {
      entries.erase(lru.front());
      lru.pop_front();
    }
    lru.push_back(key);
    auto position = lru.end();
    --position;
    entries.emplace(
      key, Entry{searched_lower_m, searched_upper_m, runs, position});
  }

  mutable std::mutex mutex;
  std::size_t maximum_entries{};
  std::list<Key> lru;
  std::unordered_map<Key, Entry, KeyHash> entries;
};

Cache::Cache(const std::size_t maximum_entries)
: impl_(std::make_unique<Impl>(maximum_entries))
{
}

Cache::~Cache() = default;

void Cache::clear() noexcept
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->entries.clear();
  impl_->lru.clear();
}

std::size_t Cache::size() const noexcept
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->entries.size();
}

std::size_t Cache::maximum_entries() const noexcept
{
  return impl_->maximum_entries;
}

namespace
{

constexpr double kNumericalTolerance = 1e-9;

bool finite_stage(const StageRequest & stage) noexcept
{
  const auto ordered_extended_bounds = [](const double lower,
      const double upper) noexcept {
      return !std::isnan(lower) && !std::isnan(upper) && lower <= upper;
    };
  return
    std::isfinite(stage.solved_progress_m) &&
    std::isfinite(stage.solved_lateral_m) &&
    std::isfinite(stage.solved_lag_m) &&
    std::isfinite(stage.solved_heading_offset_rad) &&
    std::isfinite(stage.lateral_lower_m) &&
    std::isfinite(stage.lateral_upper_m) &&
    // The canonical semantic adapter intentionally represents an unbounded
    // lag or heading state as +/-infinity. The physical refinement owns the
    // finite trust region, so extended-real source bounds are valid here.
    ordered_extended_bounds(stage.lag_lower_m, stage.lag_upper_m) &&
    ordered_extended_bounds(stage.heading_lower_rad, stage.heading_upper_rad) &&
    std::isfinite(stage.progress_lower_m) &&
    std::isfinite(stage.progress_upper_m) &&
    stage.lateral_lower_m <= stage.lateral_upper_m &&
    stage.progress_lower_m <= stage.progress_upper_m;
}

double bucket_center(const double value, const double width) noexcept
{
  return std::round(value / width) * width;
}

double clamped_bucket_lower(
  const double value, const double width, const double original_lower) noexcept
{
  return std::max(original_lower, bucket_center(value, width) - 0.5 * width);
}

double clamped_bucket_upper(
  const double value, const double width, const double original_upper) noexcept
{
  return std::min(original_upper, bucket_center(value, width) + 0.5 * width);
}

}  // namespace

const char * to_string(const PreRefinementLateralSupportReason reason) noexcept
{
  switch (reason) {
    case PreRefinementLateralSupportReason::NotRequested:
      return "not-requested";
    case PreRefinementLateralSupportReason::InvalidInput:
      return "invalid-input";
    case PreRefinementLateralSupportReason::Accepted:
      return "accepted";
  }
  return "unknown";
}

PreRefinementLateralSupport resolve_pre_refinement_lateral_support(
  const PreRefinementLateralSupportRequest & request) noexcept
{
  PreRefinementLateralSupport result;
  if (!request.active) {
    result.valid = true;
    return result;
  }
  result.reason = PreRefinementLateralSupportReason::InvalidInput;
  if (
    !std::isfinite(request.initial_lateral_m) ||
    request.wall_lower_m.size() < 2U ||
    request.wall_lower_m.size() != request.wall_upper_m.size())
  {
    return result;
  }

  double lower_m = request.initial_lateral_m;
  double upper_m = request.initial_lateral_m;
  for (std::size_t index = 0U; index < request.wall_lower_m.size(); ++index) {
    const double wall_lower_m = request.wall_lower_m[index];
    const double wall_upper_m = request.wall_upper_m[index];
    if (
      !std::isfinite(wall_lower_m) || !std::isfinite(wall_upper_m) ||
      wall_lower_m > wall_upper_m)
    {
      return result;
    }
    lower_m = std::min(lower_m, wall_lower_m);
    upper_m = std::max(upper_m, wall_upper_m);
  }
  if (!std::isfinite(lower_m) || !std::isfinite(upper_m) || lower_m > upper_m) {
    return result;
  }
  result.valid = true;
  result.applied = true;
  result.reason = PreRefinementLateralSupportReason::Accepted;
  result.lower_m = lower_m;
  result.upper_m = upper_m;
  return result;
}

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::NotRequested: return "not-requested";
    case Reason::InvalidInput: return "invalid-input";
    case Reason::CourseFrameUnavailable: return "course-frame-unavailable";
    case Reason::WallIntervalUnavailable: return "wall-interval-unavailable";
    case Reason::EmptyTrustRegion: return "empty-trust-region";
    case Reason::Accepted: return "accepted";
  }
  return "unknown";
}

Result resolve(const Request & request, Cache * cache) noexcept
{
  Result result;
  if (!request.active) {
    return result;
  }
  result.reason = Reason::InvalidInput;
  result.detail = to_string(result.reason);
  if (
    request.wall_grid == nullptr || !request.wall_grid->valid() ||
    !request.footprint.valid() || request.course_frame_knots.size() < 2U ||
    request.stages.empty() || !finite_stage(request.initial_stage) ||
    !std::isfinite(request.course_progress_origin_m) ||
    !std::isfinite(request.heading_bucket_width_rad) ||
    request.heading_bucket_width_rad <= 0.0 ||
    !std::isfinite(request.translation_bucket_width_m) ||
    request.translation_bucket_width_m <= 0.0 ||
    !std::isfinite(request.lateral_sample_step_m) ||
    request.lateral_sample_step_m <= 0.0 ||
    !std::isfinite(request.boundary_guard_m) ||
    request.boundary_guard_m < 0.0)
  {
    return result;
  }
  result.valid = true;
  result.stages.reserve(request.stages.size());
  std::uint64_t wall_grid_fingerprint = request.wall_grid_fingerprint;
  if (cache != nullptr && wall_grid_fingerprint == 0U) {
    wall_grid_fingerprint =
      recovery_footprint::occupancy_grid_fingerprint(*request.wall_grid);
  }
  for (std::size_t index = 0U; index < request.stages.size(); ++index) {
    const auto & stage = request.stages[index];
    if (!finite_stage(stage)) {
      result.first_failure_stage = static_cast<int>(index);
      result.detail = "non-finite or inverted stage bounds";
      return result;
    }

    StageBounds bounds;
    bounds.progress_bucket_center_m = bucket_center(
      stage.solved_progress_m, request.translation_bucket_width_m);
    bounds.lag_bucket_center_m = bucket_center(
      stage.solved_lag_m, request.translation_bucket_width_m);
    bounds.heading_bucket_center_rad = bucket_center(
      stage.solved_heading_offset_rad, request.heading_bucket_width_rad);
    bounds.progress_lower_m = clamped_bucket_lower(
      stage.solved_progress_m, request.translation_bucket_width_m,
      stage.progress_lower_m);
    bounds.progress_upper_m = clamped_bucket_upper(
      stage.solved_progress_m, request.translation_bucket_width_m,
      stage.progress_upper_m);
    bounds.lag_lower_m = clamped_bucket_lower(
      stage.solved_lag_m, request.translation_bucket_width_m,
      stage.lag_lower_m);
    bounds.lag_upper_m = clamped_bucket_upper(
      stage.solved_lag_m, request.translation_bucket_width_m,
      stage.lag_upper_m);
    bounds.heading_lower_rad = clamped_bucket_lower(
      stage.solved_heading_offset_rad, request.heading_bucket_width_rad,
      stage.heading_lower_rad);
    bounds.heading_upper_rad = clamped_bucket_upper(
      stage.solved_heading_offset_rad, request.heading_bucket_width_rad,
      stage.heading_upper_rad);
    if (
      bounds.progress_lower_m > bounds.progress_upper_m + kNumericalTolerance ||
      bounds.lag_lower_m > bounds.lag_upper_m + kNumericalTolerance ||
      bounds.heading_lower_rad >
      bounds.heading_upper_rad + kNumericalTolerance)
    {
      result.reason = Reason::EmptyTrustRegion;
      result.first_failure_stage = static_cast<int>(index);
      result.detail = to_string(result.reason);
      return result;
    }

    const double global_progress_m = request.course_progress_origin_m +
      bounds.progress_bucket_center_m;
    const auto course = mpc_stage_geometry::sample_course_frame(
      request.course_frame_knots, global_progress_m,
      0.5 * request.translation_bucket_width_m + kNumericalTolerance);
    if (!course.has_value()) {
      result.reason = Reason::CourseFrameUnavailable;
      result.first_failure_stage = static_cast<int>(index);
      result.detail = to_string(result.reason);
      return result;
    }

    const double cos_heading = std::cos(course->heading_rad);
    const double sin_heading = std::sin(course->heading_rad);
    const recovery_footprint::Pose2D reference_pose{
      course->x_m + bounds.lag_bucket_center_m * cos_heading,
      course->y_m + bounds.lag_bucket_center_m * sin_heading,
      course->heading_rad};
    const double maximum_longitudinal_extent = std::max(
      request.footprint.front_extent_m,
      request.footprint.rear_extent_m) + request.footprint.margin_m;
    const double maximum_lateral_extent = std::max(
      request.footprint.left_extent_m,
      request.footprint.right_extent_m) + request.footprint.margin_m;
    const double footprint_radius_m = std::hypot(
      maximum_longitudinal_extent, maximum_lateral_extent);
    const double heading_guard_m = 2.0 * footprint_radius_m * std::sin(
      0.25 * request.heading_bucket_width_rad);
    // Progress and lag may each move by half a translation bucket. Their
    // worst-case world translations can align, so both halves are retained.
    recovery_footprint::FootprintExtents guarded_footprint = request.footprint;
    guarded_footprint.margin_m +=
      request.translation_bucket_width_m + heading_guard_m;

    Cache::Impl::Lookup cache_lookup{
      std::nullopt, stage.lateral_lower_m, stage.lateral_upper_m};
    Cache::Impl::Key cache_key;
    if (cache != nullptr && wall_grid_fingerprint != 0U) {
      cache_key.wall_grid_fingerprint = wall_grid_fingerprint;
      cache_key.geometry = {
        guarded_footprint.front_extent_m,
        guarded_footprint.rear_extent_m,
        guarded_footprint.left_extent_m,
        guarded_footprint.right_extent_m,
        guarded_footprint.margin_m,
        reference_pose.x_m,
        reference_pose.y_m,
        reference_pose.yaw_rad,
        bounds.heading_bucket_center_rad,
        0.0,
        request.lateral_sample_step_m};
      cache_lookup = cache->impl_->lookup(
        cache_key, stage.lateral_lower_m, stage.lateral_upper_m);
    }
    recovery_footprint::LateralClearRunsResult runs;
    if (cache_lookup.runs.has_value()) {
      runs = std::move(cache_lookup.runs.value());
      ++result.cache_hit_count;
    } else {
      runs = recovery_footprint::find_clear_lateral_runs_with_heading(
        *request.wall_grid, guarded_footprint, reference_pose,
        cache_lookup.evaluation_lower_m, cache_lookup.evaluation_upper_m,
        bounds.heading_bucket_center_rad, 0.0,
        request.lateral_sample_step_m);
      ++result.cache_miss_count;
      result.cache_scanned_pose_count += runs.checked_pose_count;
      if (cache != nullptr && wall_grid_fingerprint != 0U && runs.valid) {
        cache->impl_->store(
          cache_key, cache_lookup.evaluation_lower_m,
          cache_lookup.evaluation_upper_m, runs);
      }
    }
    const auto interval = recovery_footprint::select_lateral_clear_interval(
      runs, stage.lateral_lower_m, stage.lateral_upper_m,
      stage.solved_lateral_m, request.boundary_guard_m);
    result.checked_pose_count += interval.checked_pose_count;
    if (!interval.valid || !interval.feasible) {
      result.reason = Reason::WallIntervalUnavailable;
      result.first_failure_stage = static_cast<int>(index);
      std::ostringstream detail;
      detail << to_string(result.reason) << "/stage=" << index
             << "/progress=" << stage.solved_progress_m
             << "/lateral=" << stage.solved_lateral_m
             << "/lag=" << stage.solved_lag_m
             << "/heading=" << stage.solved_heading_offset_rad;
      result.detail = detail.str();
      return result;
    }
    bounds.lateral_lower_m = std::max(
      stage.lateral_lower_m, interval.lower_lateral_offset_m);
    bounds.lateral_upper_m = std::min(
      stage.lateral_upper_m, interval.upper_lateral_offset_m);
    if (bounds.lateral_lower_m > bounds.lateral_upper_m + kNumericalTolerance) {
      result.reason = Reason::EmptyTrustRegion;
      result.first_failure_stage = static_cast<int>(index);
      result.detail = "empty lateral trust region";
      return result;
    }
    result.stages.push_back(bounds);
  }

  const auto interpolate = [](const double source, const double destination,
      const double ratio) noexcept {
      return source + ratio * (destination - source);
    };
  // Keep the row structure constant across cycles. Four interior samples make
  // five convex subsegments per controller stage; the independent nonlinear
  // swept-footprint proof remains the final authority for geometry below this
  // discretization. Re-scanning the whole raster corridor at every interior
  // point made the result stale before publication and duplicated static work.
  constexpr std::size_t kInteriorSamplesPerTransition = 4U;
  double source_physical_lower_m = request.initial_stage.solved_lateral_m;
  double source_physical_upper_m = request.initial_stage.solved_lateral_m;
  const StageRequest * source_nominal = &request.initial_stage;
  for (std::size_t transition = 0U;
    transition < request.stages.size(); ++transition)
  {
    const auto & destination = request.stages[transition];
    const auto & destination_physical = result.stages[transition];
    for (std::size_t sample = 1U;
      sample <= kInteriorSamplesPerTransition; ++sample)
    {
      const double ratio = static_cast<double>(sample) /
        static_cast<double>(kInteriorSamplesPerTransition + 1U);
      const double physical_lower_m = interpolate(
        source_physical_lower_m,
        destination_physical.lateral_lower_m, ratio);
      const double physical_upper_m = interpolate(
        source_physical_upper_m,
        destination_physical.lateral_upper_m, ratio);
      const double nominal_lower_m = interpolate(
        source_nominal->lateral_lower_m,
        destination.lateral_lower_m, ratio);
      const double nominal_upper_m = interpolate(
        source_nominal->lateral_upper_m,
        destination.lateral_upper_m, ratio);
      const double lower_m = std::max(physical_lower_m, nominal_lower_m);
      const double upper_m = std::min(physical_upper_m, nominal_upper_m);
      if (lower_m > upper_m + kNumericalTolerance) {
        result.reason = Reason::EmptyTrustRegion;
        result.first_failure_stage = static_cast<int>(transition);
        result.detail = "empty swept lateral trust region";
        result.applied = false;
        result.feasible = false;
        return result;
      }
      result.swept_lateral_constraints.push_back(
        SweptLateralConstraint{
          static_cast<int>(transition), ratio,
          lower_m, upper_m});
    }
    source_physical_lower_m = destination_physical.lateral_lower_m;
    source_physical_upper_m = destination_physical.lateral_upper_m;
    source_nominal = &destination;
  }

  result.feasible = true;
  result.applied = true;
  result.reason = Reason::Accepted;
  result.detail = to_string(result.reason);
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_wall_refinement
