#include "multi_purpose_mpc_ros/recovery_footprint.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace multi_purpose_mpc_ros::recovery_footprint
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kNumericalEpsilon = 1e-12;
constexpr double kClearanceComparisonToleranceM = 1e-6;
constexpr double kMinimumEscapeImprovementM = 1e-3;
constexpr std::size_t kMaximumSamples = 1000000U;
constexpr std::uint64_t kFingerprintOffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFingerprintPrime = 1099511628211ULL;

void append_fingerprint_byte(std::uint64_t & value, const std::uint8_t byte) noexcept
{
  value ^= static_cast<std::uint64_t>(byte);
  value *= kFingerprintPrime;
}

void append_fingerprint_u64(std::uint64_t & value, const std::uint64_t field) noexcept
{
  for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
    append_fingerprint_byte(
      value, static_cast<std::uint8_t>((field >> shift) & 0xffU));
  }
}

void append_fingerprint_double(std::uint64_t & value, const double field) noexcept
{
  static_assert(sizeof(double) == sizeof(std::uint64_t));
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &field, sizeof(bits));
  append_fingerprint_u64(value, bits);
}

bool finite(const double value) noexcept
{
  return std::isfinite(value);
}

bool valid_pose(const Pose2D & pose) noexcept
{
  return finite(pose.x_m) && finite(pose.y_m) && finite(pose.yaw_rad);
}

bool valid_circle_obstacle(const CircleObstacle & obstacle) noexcept
{
  return
    finite(obstacle.x_m) && finite(obstacle.y_m) &&
    finite(obstacle.velocity_x_mps) && finite(obstacle.velocity_y_mps) &&
    finite(obstacle.radius_m) && obstacle.radius_m >= 0.0;
}

double circle_to_footprint_clearance(
  const FootprintExtents & footprint, const Pose2D & pose,
  const double circle_x_m, const double circle_y_m, const double radius_m) noexcept
{
  const double dx = circle_x_m - pose.x_m;
  const double dy = circle_y_m - pose.y_m;
  const double cos_yaw = std::cos(pose.yaw_rad);
  const double sin_yaw = std::sin(pose.yaw_rad);
  const double longitudinal = cos_yaw * dx + sin_yaw * dy;
  const double lateral = -sin_yaw * dx + cos_yaw * dy;
  const double minimum_longitudinal = -footprint.rear_extent_m - footprint.margin_m;
  const double maximum_longitudinal = footprint.front_extent_m + footprint.margin_m;
  const double minimum_lateral = -footprint.right_extent_m - footprint.margin_m;
  const double maximum_lateral = footprint.left_extent_m + footprint.margin_m;

  const double outside_longitudinal = std::max(
    {minimum_longitudinal - longitudinal, 0.0, longitudinal - maximum_longitudinal});
  const double outside_lateral = std::max(
    {minimum_lateral - lateral, 0.0, lateral - maximum_lateral});
  double signed_point_clearance = std::hypot(outside_longitudinal, outside_lateral);
  if (
    outside_longitudinal <= kNumericalEpsilon &&
    outside_lateral <= kNumericalEpsilon)
  {
    const double distance_to_boundary = std::min(
      {longitudinal - minimum_longitudinal, maximum_longitudinal - longitudinal,
        lateral - minimum_lateral, maximum_lateral - lateral});
    signed_point_clearance = -distance_to_boundary;
  }
  return signed_point_clearance - radius_m;
}

double wrap_to_pi(const double angle) noexcept
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool valid_primitive(const ReversePrimitive primitive) noexcept
{
  return primitive == ReversePrimitive::Straight || primitive == ReversePrimitive::Left ||
         primitive == ReversePrimitive::Right || primitive == ReversePrimitive::ForwardStraight ||
         primitive == ReversePrimitive::ForwardLeft ||
         primitive == ReversePrimitive::ForwardRight;
}

bool valid_y_axis(const YAxisConvention convention) noexcept
{
  return convention == YAxisConvention::RowZeroAtMinimumY ||
         convention == YAxisConvention::RowZeroAtMaximumY;
}

bool checked_cell_count(
  const std::size_t width, const std::size_t height, std::size_t & count) noexcept
{
  if (width == 0U || height == 0U || width > std::numeric_limits<std::size_t>::max() / height) {
    return false;
  }
  count = width * height;
  return true;
}

std::optional<std::size_t> subdivision_count(
  const double distance, const double step) noexcept
{
  if (!finite(distance) || !finite(step) || distance < 0.0 || step <= 0.0) {
    return std::nullopt;
  }
  const long double count = std::max(
    1.0L, std::ceil(
      static_cast<long double>(distance) / static_cast<long double>(step)));
  if (!std::isfinite(count) || count > static_cast<long double>(kMaximumSamples)) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(count);
}

double steering_for(
  const ReversePrimitive primitive, const ReverseRolloutParameters & parameters) noexcept
{
  if (primitive == ReversePrimitive::Left || primitive == ReversePrimitive::ForwardLeft) {
    return parameters.steering_angle_rad;
  }
  if (primitive == ReversePrimitive::Right || primitive == ReversePrimitive::ForwardRight) {
    return -parameters.steering_angle_rad;
  }
  return 0.0;
}

bool valid_rollout_parameters(
  const ReversePrimitive primitive, const ReverseRolloutParameters & parameters) noexcept
{
  if (
    !valid_primitive(primitive) || !finite(parameters.reverse_distance_m) ||
    !finite(parameters.rollout_step_m) || !finite(parameters.swept_step_m) ||
    !finite(parameters.wheelbase_m) || !finite(parameters.steering_angle_rad) ||
    parameters.reverse_distance_m <= 0.0 || parameters.rollout_step_m <= 0.0 ||
    parameters.swept_step_m <= 0.0 || parameters.wheelbase_m <= 0.0 ||
    parameters.steering_angle_rad < 0.0 ||
    parameters.steering_angle_rad >= kPi / 2.0 - 1e-6)
  {
    return false;
  }
  if (
    (primitive == ReversePrimitive::Left || primitive == ReversePrimitive::Right ||
    primitive == ReversePrimitive::ForwardLeft ||
    primitive == ReversePrimitive::ForwardRight) &&
    parameters.steering_angle_rad <= kNumericalEpsilon)
  {
    return false;
  }
  return true;
}

Pose2D pose_at_reverse_distance(
  const Pose2D & initial_pose, const ReversePrimitive primitive,
  const ReverseRolloutParameters & parameters, const double distance_m) noexcept
{
  const double steering = steering_for(primitive, parameters);
  const double curvature = std::tan(steering) / parameters.wheelbase_m;
  const double signed_distance = primitive_is_forward(primitive) ?
    distance_m : -distance_m;

  Pose2D pose = initial_pose;
  if (std::abs(curvature) <= kNumericalEpsilon) {
    pose.x_m += signed_distance * std::cos(initial_pose.yaw_rad);
    pose.y_m += signed_distance * std::sin(initial_pose.yaw_rad);
    return pose;
  }

  const double final_yaw = initial_pose.yaw_rad + signed_distance * curvature;
  pose.x_m +=
    (std::sin(final_yaw) - std::sin(initial_pose.yaw_rad)) / curvature;
  pose.y_m +=
    (-std::cos(final_yaw) + std::cos(initial_pose.yaw_rad)) / curvature;
  pose.yaw_rad = wrap_to_pi(final_yaw);
  return pose;
}

struct OrientedBox
{
  double center_x{};
  double center_y{};
  double forward_x{};
  double forward_y{};
  double left_x{};
  double left_y{};
  double half_forward{};
  double half_left{};
  std::array<Point2D, 4> corners;
};

OrientedBox make_box(const FootprintExtents & footprint, const Pose2D & pose) noexcept
{
  const double front = footprint.front_extent_m + footprint.margin_m;
  const double rear = footprint.rear_extent_m + footprint.margin_m;
  const double left = footprint.left_extent_m + footprint.margin_m;
  const double right = footprint.right_extent_m + footprint.margin_m;
  const double local_center_forward = 0.5 * (front - rear);
  const double local_center_left = 0.5 * (left - right);

  OrientedBox box;
  box.forward_x = std::cos(pose.yaw_rad);
  box.forward_y = std::sin(pose.yaw_rad);
  box.left_x = -box.forward_y;
  box.left_y = box.forward_x;
  box.half_forward = 0.5 * (front + rear);
  box.half_left = 0.5 * (left + right);
  box.center_x =
    pose.x_m + local_center_forward * box.forward_x + local_center_left * box.left_x;
  box.center_y =
    pose.y_m + local_center_forward * box.forward_y + local_center_left * box.left_y;

  std::size_t index = 0U;
  for (const double forward_sign : {-1.0, 1.0}) {
    for (const double left_sign : {-1.0, 1.0}) {
      box.corners[index++] = Point2D{
        box.center_x + forward_sign * box.half_forward * box.forward_x +
        left_sign * box.half_left * box.left_x,
        box.center_y + forward_sign * box.half_forward * box.forward_y +
        left_sign * box.half_left * box.left_y};
    }
  }
  return box;
}

bool box_intersects_cell(
  const OrientedBox & box, const double cell_x, const double cell_y,
  const double cell_half) noexcept
{
  const double dx = cell_x - box.center_x;
  const double dy = cell_y - box.center_y;
  const double abs_forward_x = std::abs(box.forward_x);
  const double abs_forward_y = std::abs(box.forward_y);
  const double world_half_x =
    box.half_forward * abs_forward_x + box.half_left * abs_forward_y;
  const double world_half_y =
    box.half_forward * abs_forward_y + box.half_left * abs_forward_x;
  if (
    std::abs(dx) > world_half_x + cell_half + kNumericalEpsilon ||
    std::abs(dy) > world_half_y + cell_half + kNumericalEpsilon)
  {
    return false;
  }

  const double local_forward = dx * box.forward_x + dy * box.forward_y;
  const double local_left = dx * box.left_x + dy * box.left_y;
  const double cell_projection = cell_half * (abs_forward_x + abs_forward_y);
  return
    std::abs(local_forward) <= box.half_forward + cell_projection + kNumericalEpsilon &&
    std::abs(local_left) <= box.half_left + cell_projection + kNumericalEpsilon;
}

double footprint_corner_radius(const FootprintExtents & footprint) noexcept
{
  const double longitudinal = std::max(
    footprint.front_extent_m + footprint.margin_m,
    footprint.rear_extent_m + footprint.margin_m);
  const double lateral = std::max(
    footprint.left_extent_m + footprint.margin_m,
    footprint.right_extent_m + footprint.margin_m);
  return std::hypot(longitudinal, lateral);
}

bool initial_contact_patch_is_forward(
  const OccupancyGrid & grid, const Pose2D & initial_pose,
  const std::vector<std::size_t> & initial_contact_cells) noexcept
{
  const double forward_x = std::cos(initial_pose.yaw_rad);
  const double forward_y = std::sin(initial_pose.yaw_rad);
  const double cell_half_projection = 0.5 * grid.resolution_m *
    (std::abs(forward_x) + std::abs(forward_y));
  for (const std::size_t cell_index : initial_contact_cells) {
    if (cell_index >= grid.cells.size()) {
      return false;
    }
    const auto cell_center = grid.grid_to_world(
      cell_index / grid.width, cell_index % grid.width);
    if (!cell_center.has_value()) {
      return false;
    }
    const double local_forward =
      (cell_center->x_m - initial_pose.x_m) * forward_x +
      (cell_center->y_m - initial_pose.y_m) * forward_y;
    if (local_forward - cell_half_projection < -kNumericalEpsilon) {
      return false;
    }
  }
  return true;
}

bool initial_contact_patch_is_rear(
  const OccupancyGrid & grid, const Pose2D & initial_pose,
  const std::vector<std::size_t> & initial_contact_cells) noexcept
{
  const double forward_x = std::cos(initial_pose.yaw_rad);
  const double forward_y = std::sin(initial_pose.yaw_rad);
  const double cell_half_projection = 0.5 * grid.resolution_m *
    (std::abs(forward_x) + std::abs(forward_y));
  for (const std::size_t cell_index : initial_contact_cells) {
    if (cell_index >= grid.cells.size()) {
      return false;
    }
    const auto cell_center = grid.grid_to_world(
      cell_index / grid.width, cell_index % grid.width);
    if (!cell_center.has_value()) {
      return false;
    }
    const double local_forward =
      (cell_center->x_m - initial_pose.x_m) * forward_x +
      (cell_center->y_m - initial_pose.y_m) * forward_y;
    if (local_forward + cell_half_projection > kNumericalEpsilon) {
      return false;
    }
  }
  return true;
}

FeasibilityResult rejected(
  FeasibilityResult result, const RejectReason reason, const double distance_m,
  const std::size_t final_contact_count = 0U)
{
  result.feasible = false;
  result.reason = reason;
  result.rejected_at_distance_m = distance_m;
  result.final_contact_count = final_contact_count;
  return result;
}

}  // namespace

bool OccupancyGrid::valid() const noexcept
{
  std::size_t expected_size = 0U;
  return
    checked_cell_count(width, height, expected_size) && cells.size() == expected_size &&
    finite(resolution_m) && resolution_m > 0.0 && finite(origin_x_m) &&
    finite(origin_y_m) && valid_y_axis(y_axis);
}

bool OccupancyGrid::build_non_free_integral_index()
{
  non_free_integral_index.clear();
  if (!valid() || width == std::numeric_limits<std::size_t>::max() ||
    height == std::numeric_limits<std::size_t>::max())
  {
    return false;
  }
  const std::size_t stride = width + 1U;
  std::size_t index_size{};
  if (!checked_cell_count(stride, height + 1U, index_size)) {
    return false;
  }
  std::vector<std::size_t> index(index_size, 0U);
  for (std::size_t row = 0U; row < height; ++row) {
    std::size_t row_non_free{};
    for (std::size_t column = 0U; column < width; ++column) {
      row_non_free += cell(row, column) == CellState::Free ? 0U : 1U;
      index[(row + 1U) * stride + column + 1U] =
        index[row * stride + column + 1U] + row_non_free;
    }
  }
  non_free_integral_index = std::move(index);
  return true;
}

bool OccupancyGrid::has_non_free_integral_index() const noexcept
{
  if (!valid() || width == std::numeric_limits<std::size_t>::max() ||
    height == std::numeric_limits<std::size_t>::max())
  {
    return false;
  }
  std::size_t expected_size{};
  return checked_cell_count(width + 1U, height + 1U, expected_size) &&
         non_free_integral_index.size() == expected_size;
}

std::optional<bool> OccupancyGrid::contains_non_free_cell(
  const std::size_t minimum_row, const std::size_t maximum_row,
  const std::size_t minimum_column, const std::size_t maximum_column) const noexcept
{
  if (!has_non_free_integral_index() || minimum_row > maximum_row ||
    minimum_column > maximum_column || maximum_row >= height ||
    maximum_column >= width)
  {
    return std::nullopt;
  }
  const std::size_t stride = width + 1U;
  const std::size_t top = minimum_row;
  const std::size_t bottom = maximum_row + 1U;
  const std::size_t left = minimum_column;
  const std::size_t right = maximum_column + 1U;
  const std::size_t count =
    non_free_integral_index[bottom * stride + right] -
    non_free_integral_index[top * stride + right] -
    non_free_integral_index[bottom * stride + left] +
    non_free_integral_index[top * stride + left];
  return count != 0U;
}

std::uint64_t occupancy_grid_fingerprint(const OccupancyGrid & grid) noexcept
{
  if (!grid.valid()) {
    return 0U;
  }

  std::uint64_t fingerprint = kFingerprintOffsetBasis;
  // Version the byte stream so future schema changes cannot silently compare
  // equal to the current occupancy-grid identity.
  append_fingerprint_u64(fingerprint, 1U);
  append_fingerprint_u64(fingerprint, static_cast<std::uint64_t>(grid.width));
  append_fingerprint_u64(fingerprint, static_cast<std::uint64_t>(grid.height));
  append_fingerprint_double(fingerprint, grid.resolution_m);
  append_fingerprint_double(fingerprint, grid.origin_x_m);
  append_fingerprint_double(fingerprint, grid.origin_y_m);
  append_fingerprint_u64(
    fingerprint, static_cast<std::uint64_t>(grid.y_axis));
  for (const CellState cell : grid.cells) {
    append_fingerprint_byte(
      fingerprint,
      static_cast<std::uint8_t>(static_cast<std::int8_t>(cell)));
  }
  // Zero is reserved for invalid/unavailable identity.
  return fingerprint == 0U ? 1U : fingerprint;
}

std::optional<GridIndex> OccupancyGrid::world_to_grid(
  const double x_m, const double y_m) const noexcept
{
  if (!valid() || !finite(x_m) || !finite(y_m)) {
    return std::nullopt;
  }

  const long double x_index = std::floor(
    (static_cast<long double>(x_m) - static_cast<long double>(origin_x_m)) /
    static_cast<long double>(resolution_m) + 0.5L);
  const long double y_index = std::floor(
    (static_cast<long double>(y_m) - static_cast<long double>(origin_y_m)) /
    static_cast<long double>(resolution_m) + 0.5L);
  if (
    !std::isfinite(x_index) || !std::isfinite(y_index) || x_index < 0.0L ||
    y_index < 0.0L || x_index >= static_cast<long double>(width) ||
    y_index >= static_cast<long double>(height))
  {
    return std::nullopt;
  }

  const auto column = static_cast<std::size_t>(x_index);
  const auto world_y_index = static_cast<std::size_t>(y_index);
  const std::size_t row = y_axis == YAxisConvention::RowZeroAtMaximumY ?
    height - 1U - world_y_index : world_y_index;
  return GridIndex{row, column};
}

std::optional<Point2D> OccupancyGrid::grid_to_world(
  const std::size_t row, const std::size_t column) const noexcept
{
  if (!valid() || row >= height || column >= width) {
    return std::nullopt;
  }
  const std::size_t world_y_index = y_axis == YAxisConvention::RowZeroAtMaximumY ?
    height - 1U - row : row;
  return Point2D{
    origin_x_m + static_cast<double>(column) * resolution_m,
    origin_y_m + static_cast<double>(world_y_index) * resolution_m};
}

CellState OccupancyGrid::cell(const std::size_t row, const std::size_t column) const noexcept
{
  if (!valid() || row >= height || column >= width) {
    return CellState::Unknown;
  }
  return cells[row * width + column];
}

bool FootprintExtents::valid() const noexcept
{
  if (
    !finite(front_extent_m) || !finite(rear_extent_m) || !finite(left_extent_m) ||
    !finite(right_extent_m) || !finite(margin_m) || front_extent_m < 0.0 ||
    rear_extent_m < 0.0 || left_extent_m < 0.0 || right_extent_m < 0.0 ||
    margin_m < 0.0)
  {
    return false;
  }
  return
    front_extent_m + rear_extent_m + 2.0 * margin_m > kNumericalEpsilon &&
    left_extent_m + right_extent_m + 2.0 * margin_m > kNumericalEpsilon;
}

bool primitive_is_forward(const ReversePrimitive primitive) noexcept
{
  return primitive == ReversePrimitive::ForwardStraight ||
         primitive == ReversePrimitive::ForwardLeft ||
         primitive == ReversePrimitive::ForwardRight;
}

double primitive_steering_sign(const ReversePrimitive primitive) noexcept
{
  if (primitive == ReversePrimitive::Left || primitive == ReversePrimitive::ForwardLeft) {
    return 1.0;
  }
  if (primitive == ReversePrimitive::Right || primitive == ReversePrimitive::ForwardRight) {
    return -1.0;
  }
  return 0.0;
}

std::vector<double> steering_magnitude_samples(
  const double maximum_steering_angle_rad, const std::size_t sample_count)
{
  if (
    !finite(maximum_steering_angle_rad) || maximum_steering_angle_rad <= 0.0 ||
    sample_count == 0U || sample_count > kMaximumSamples)
  {
    return {};
  }
  std::vector<double> samples;
  samples.reserve(sample_count);
  for (std::size_t index = 1U; index <= sample_count; ++index) {
    samples.push_back(
      maximum_steering_angle_rad * static_cast<double>(index) /
      static_cast<double>(sample_count));
  }
  return samples;
}

const char * to_string(const RejectReason reason) noexcept
{
  switch (reason) {
    case RejectReason::None:
      return "none";
    case RejectReason::NotEvaluated:
      return "not_evaluated";
    case RejectReason::InvalidGrid:
      return "invalid_grid";
    case RejectReason::InvalidFootprint:
      return "invalid_footprint";
    case RejectReason::InvalidInitialPose:
      return "invalid_initial_pose";
    case RejectReason::InvalidRollout:
      return "invalid_rollout";
    case RejectReason::SampleLimitExceeded:
      return "sample_limit_exceeded";
    case RejectReason::InitialOutOfMap:
      return "initial_out_of_map";
    case RejectReason::InitialContactNotForward:
      return "initial_contact_not_forward";
    case RejectReason::InitialContactNotRear:
      return "initial_contact_not_rear";
    case RejectReason::OutOfMap:
      return "out_of_map";
    case RejectReason::Collision:
      return "collision";
    case RejectReason::NewContact:
      return "new_contact";
    case RejectReason::ContactWorsened:
      return "contact_worsened";
    case RejectReason::ContactNotImproved:
      return "contact_not_improved";
    case RejectReason::InitialContactNotCleared:
      return "initial_contact_not_cleared";
  }
  return "unknown";
}

const char * to_string(const MarginEscapePathReason reason) noexcept
{
  switch (reason) {
    case MarginEscapePathReason::None:
      return "none";
    case MarginEscapePathReason::InvalidInput:
      return "invalid_input";
    case MarginEscapePathReason::PhysicalPathBlocked:
      return "physical_path_blocked";
    case MarginEscapePathReason::MarginPathBlocked:
      return "margin_path_blocked";
    case MarginEscapePathReason::MarginEscapeNotAllowed:
      return "margin_escape_not_allowed";
    case MarginEscapePathReason::MarginNotCleared:
      return "margin_not_cleared";
    case MarginEscapePathReason::MarginRecontact:
      return "margin_recontact";
  }
  return "unknown";
}

const char * to_string(const ReversePrimitive primitive) noexcept
{
  switch (primitive) {
    case ReversePrimitive::Straight:
      return "reverse_straight";
    case ReversePrimitive::Left:
      return "reverse_left";
    case ReversePrimitive::Right:
      return "reverse_right";
    case ReversePrimitive::ForwardStraight:
      return "forward_straight";
    case ReversePrimitive::ForwardLeft:
      return "forward_left";
    case ReversePrimitive::ForwardRight:
      return "forward_right";
  }
  return "invalid";
}

const char * to_string(const WallRegion region) noexcept
{
  switch (region) {
    case WallRegion::None:
      return "none";
    case WallRegion::Front:
      return "front";
    case WallRegion::Rear:
      return "rear";
    case WallRegion::Left:
      return "left";
    case WallRegion::Right:
      return "right";
    case WallRegion::Mixed:
      return "mixed";
    case WallRegion::Unknown:
      return "unknown";
  }
  return "unknown";
}

bool use_stepwise_escape_mode(
  const bool side_escape_enabled, const bool current_footprint_clear,
  const std::size_t current_contact_count, const WallRegion wall_region,
  const std::size_t completed_escape_steps,
  const bool continuous_clear_escape_enabled) noexcept
{
  if (!side_escape_enabled) {
    return false;
  }
  const bool side_wall =
    wall_region == WallRegion::Left || wall_region == WallRegion::Right ||
    wall_region == WallRegion::Mixed;
  if (current_footprint_clear) {
    if (continuous_clear_escape_enabled) {
      return false;
    }
    return wall_region == WallRegion::None || side_wall;
  }
  return
    current_contact_count > 0U &&
    (completed_escape_steps > 0U || side_wall);
}

const char * to_string(const DynamicClearanceRejectReason reason) noexcept
{
  switch (reason) {
    case DynamicClearanceRejectReason::None:
      return "none";
    case DynamicClearanceRejectReason::InvalidFootprint:
      return "invalid_footprint";
    case DynamicClearanceRejectReason::InvalidRollout:
      return "invalid_rollout";
    case DynamicClearanceRejectReason::InvalidObstacle:
      return "invalid_obstacle";
    case DynamicClearanceRejectReason::NewOverlap:
      return "new_overlap";
    case DynamicClearanceRejectReason::InitialOverlapWorsened:
      return "initial_overlap_worsened";
    case DynamicClearanceRejectReason::InitialOverlapNotImproved:
      return "initial_overlap_not_improved";
  }
  return "unknown";
}

DynamicClearanceRejectReason observe_dynamic_clearance(
  DynamicClearanceSequence & sequence, const double clearance_m) noexcept
{
  if (!finite(clearance_m)) {
    return DynamicClearanceRejectReason::InvalidObstacle;
  }
  if (!sequence.initialized) {
    sequence.initialized = true;
    sequence.initial_overlap =
      clearance_m < -kClearanceComparisonToleranceM;
    sequence.checked_pose_count = 1U;
    sequence.initial_clearance_m = clearance_m;
    sequence.previous_clearance_m = clearance_m;
    sequence.final_clearance_m = clearance_m;
    sequence.minimum_clearance_m = clearance_m;
    return DynamicClearanceRejectReason::None;
  }
  DynamicClearanceRejectReason reason = DynamicClearanceRejectReason::None;
  if (
    !sequence.initial_overlap &&
    clearance_m < -kClearanceComparisonToleranceM)
  {
    reason = DynamicClearanceRejectReason::NewOverlap;
  } else if (
    sequence.initial_overlap &&
    clearance_m + kClearanceComparisonToleranceM <
    sequence.previous_clearance_m)
  {
    reason = DynamicClearanceRejectReason::InitialOverlapWorsened;
  }
  ++sequence.checked_pose_count;
  sequence.previous_clearance_m = clearance_m;
  sequence.final_clearance_m = clearance_m;
  sequence.minimum_clearance_m = std::min(
    sequence.minimum_clearance_m, clearance_m);
  return reason;
}

DynamicClearanceRejectReason finalize_dynamic_clearance(
  const DynamicClearanceSequence & sequence) noexcept
{
  if (!sequence.initialized || sequence.checked_pose_count == 0U) {
    return DynamicClearanceRejectReason::InvalidRollout;
  }
  if (
    sequence.initial_overlap &&
    sequence.final_clearance_m <
    sequence.initial_clearance_m + kMinimumEscapeImprovementM)
  {
    return DynamicClearanceRejectReason::InitialOverlapNotImproved;
  }
  return DynamicClearanceRejectReason::None;
}

std::optional<double> circle_obstacle_clearance_at_time(
  const FootprintExtents & footprint, const Pose2D & pose,
  const CircleObstacle & obstacle, const double elapsed_time_sec) noexcept
{
  if (
    !footprint.valid() || !valid_pose(pose) ||
    !valid_circle_obstacle(obstacle) || !finite(elapsed_time_sec) ||
    elapsed_time_sec < 0.0)
  {
    return std::nullopt;
  }
  return circle_to_footprint_clearance(
    footprint, pose,
    obstacle.x_m + obstacle.velocity_x_mps * elapsed_time_sec,
    obstacle.y_m + obstacle.velocity_y_mps * elapsed_time_sec,
    obstacle.radius_m);
}

DynamicClearanceResult evaluate_circle_obstacle_clearance(
  const FootprintExtents & footprint, const std::vector<RolloutPose> & rollout,
  const CircleObstacle & obstacle, const double prediction_horizon_sec)
{
  DynamicClearanceResult result;
  if (!footprint.valid()) {
    result.reason = DynamicClearanceRejectReason::InvalidFootprint;
    return result;
  }
  if (!valid_circle_obstacle(obstacle)) {
    result.reason = DynamicClearanceRejectReason::InvalidObstacle;
    return result;
  }
  if (
    rollout.empty() || !finite(prediction_horizon_sec) || prediction_horizon_sec < 0.0 ||
    !valid_pose(rollout.front().pose) || !finite(rollout.front().reverse_distance_m) ||
    rollout.front().reverse_distance_m < 0.0)
  {
    result.reason = DynamicClearanceRejectReason::InvalidRollout;
    return result;
  }
  double previous_distance_m = rollout.front().reverse_distance_m;
  for (const auto & rollout_pose : rollout) {
    if (
      !valid_pose(rollout_pose.pose) || !finite(rollout_pose.reverse_distance_m) ||
      rollout_pose.reverse_distance_m < previous_distance_m - kNumericalEpsilon)
    {
      result.reason = DynamicClearanceRejectReason::InvalidRollout;
      return result;
    }
    previous_distance_m = rollout_pose.reverse_distance_m;
  }
  const double total_distance_m = rollout.back().reverse_distance_m;
  if (!finite(total_distance_m) || total_distance_m <= kNumericalEpsilon) {
    result.reason = DynamicClearanceRejectReason::InvalidRollout;
    return result;
  }

  result.valid = true;
  DynamicClearanceSequence clearance_sequence;
  for (std::size_t index = 0U; index < rollout.size(); ++index) {
    const auto & rollout_pose = rollout[index];
    const double distance_fraction = std::clamp(
      rollout_pose.reverse_distance_m / total_distance_m, 0.0, 1.0);
    const double prediction_time_sec = prediction_horizon_sec * distance_fraction;
    const double obstacle_x_m =
      obstacle.x_m + obstacle.velocity_x_mps * prediction_time_sec;
    const double obstacle_y_m =
      obstacle.y_m + obstacle.velocity_y_mps * prediction_time_sec;
    const double clearance_m = circle_to_footprint_clearance(
      footprint, rollout_pose.pose, obstacle_x_m, obstacle_y_m, obstacle.radius_m);
    const auto observation = observe_dynamic_clearance(
      clearance_sequence, clearance_m);
    result.checked_pose_count = clearance_sequence.checked_pose_count;
    result.initial_clearance_m = clearance_sequence.initial_clearance_m;
    result.minimum_clearance_m = clearance_sequence.minimum_clearance_m;
    result.final_clearance_m = clearance_sequence.final_clearance_m;
    if (observation != DynamicClearanceRejectReason::None) {
      result.reason = observation;
      result.rejected_at_distance_m = rollout_pose.reverse_distance_m;
      return result;
    }
  }

  const auto completion = finalize_dynamic_clearance(clearance_sequence);
  if (completion != DynamicClearanceRejectReason::None) {
    result.reason = completion;
    result.rejected_at_distance_m = total_distance_m;
    return result;
  }
  result.clear = true;
  result.reason = DynamicClearanceRejectReason::None;
  return result;
}

RolloutResult generate_reverse_rollout(
  const Pose2D & initial_pose, const ReversePrimitive primitive,
  const ReverseRolloutParameters & parameters)
{
  if (!valid_pose(initial_pose)) {
    return RolloutResult{false, RejectReason::InvalidInitialPose, {}};
  }
  if (!valid_rollout_parameters(primitive, parameters)) {
    return RolloutResult{false, RejectReason::InvalidRollout, {}};
  }
  const auto segment_count =
    subdivision_count(parameters.reverse_distance_m, parameters.rollout_step_m);
  if (!segment_count.has_value() || segment_count.value() + 1U > kMaximumSamples) {
    return RolloutResult{false, RejectReason::SampleLimitExceeded, {}};
  }

  RolloutResult result;
  result.valid = true;
  result.reason = RejectReason::None;
  result.poses.reserve(segment_count.value() + 1U);
  result.poses.push_back(RolloutPose{initial_pose, 0.0});
  for (std::size_t index = 1U; index <= segment_count.value(); ++index) {
    const double distance = index == segment_count.value() ?
      parameters.reverse_distance_m :
      parameters.reverse_distance_m * static_cast<double>(index) /
      static_cast<double>(segment_count.value());
    result.poses.push_back(
      RolloutPose{
        pose_at_reverse_distance(initial_pose, primitive, parameters, distance), distance});
  }
  return result;
}

FootprintSample sample_footprint(
  const OccupancyGrid & grid, const FootprintExtents & footprint, const Pose2D & pose)
{
  FootprintSample sample;
  if (!grid.valid() || !footprint.valid() || !valid_pose(pose)) {
    return sample;
  }
  sample.valid = true;

  const OrientedBox box = make_box(footprint, pose);
  const double cell_half = 0.5 * grid.resolution_m;
  const double map_min_x = grid.origin_x_m - cell_half;
  const double map_max_x =
    grid.origin_x_m + (static_cast<double>(grid.width) - 0.5) * grid.resolution_m;
  const double map_min_y = grid.origin_y_m - cell_half;
  const double map_max_y =
    grid.origin_y_m + (static_cast<double>(grid.height) - 0.5) * grid.resolution_m;

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & corner : box.corners) {
    min_x = std::min(min_x, corner.x_m);
    max_x = std::max(max_x, corner.x_m);
    min_y = std::min(min_y, corner.y_m);
    max_y = std::max(max_y, corner.y_m);
    if (
      corner.x_m < map_min_x - kNumericalEpsilon ||
      corner.x_m > map_max_x + kNumericalEpsilon ||
      corner.y_m < map_min_y - kNumericalEpsilon ||
      corner.y_m > map_max_y + kNumericalEpsilon)
    {
      sample.out_of_map = true;
    }
  }
  if (sample.out_of_map) {
    return sample;
  }

  const auto clamp_index = [](const long long value, const std::size_t maximum) {
      return static_cast<std::size_t>(std::clamp<long long>(
               value, 0LL, static_cast<long long>(maximum - 1U)));
    };
  const long long raw_min_column = static_cast<long long>(std::floor(
      (min_x - grid.origin_x_m) / grid.resolution_m)) - 1LL;
  const long long raw_max_column = static_cast<long long>(std::ceil(
      (max_x - grid.origin_x_m) / grid.resolution_m)) + 1LL;
  const long long raw_min_y_index = static_cast<long long>(std::floor(
      (min_y - grid.origin_y_m) / grid.resolution_m)) - 1LL;
  const long long raw_max_y_index = static_cast<long long>(std::ceil(
      (max_y - grid.origin_y_m) / grid.resolution_m)) + 1LL;
  const std::size_t min_column = clamp_index(raw_min_column, grid.width);
  const std::size_t max_column = clamp_index(raw_max_column, grid.width);
  const std::size_t min_y_index = clamp_index(raw_min_y_index, grid.height);
  const std::size_t max_y_index = clamp_index(raw_max_y_index, grid.height);

  // A summed-area query can prove that the entire conservative AABB contains
  // no occupied or unknown cell.  In that case exact oriented geometry has
  // nothing to intersect.  An absent index deliberately falls through to the
  // original exact scan.
  if (grid.has_non_free_integral_index()) {
    const std::size_t minimum_row =
      grid.y_axis == YAxisConvention::RowZeroAtMaximumY ?
      grid.height - 1U - max_y_index : min_y_index;
    const std::size_t maximum_row =
      grid.y_axis == YAxisConvention::RowZeroAtMaximumY ?
      grid.height - 1U - min_y_index : max_y_index;
    const auto contains_non_free = grid.contains_non_free_cell(
      minimum_row, maximum_row, min_column, max_column);
    if (contains_non_free.has_value() && !contains_non_free.value()) {
      return sample;
    }
  }

  for (std::size_t y_index = min_y_index; y_index <= max_y_index; ++y_index) {
    const std::size_t row = grid.y_axis == YAxisConvention::RowZeroAtMaximumY ?
      grid.height - 1U - y_index : y_index;
    for (std::size_t column = min_column; column <= max_column; ++column) {
      // Occupancy is an exact broad phase. Free cells cannot contribute a
      // footprint contact, so avoid the substantially more expensive
      // oriented-box/cell intersection for the common racing-surface case.
      // Occupied and unknown cells retain the original exact geometry and
      // fail-closed semantics.
      if (grid.cell(row, column) == CellState::Free) {
        continue;
      }
      const double cell_x =
        grid.origin_x_m + static_cast<double>(column) * grid.resolution_m;
      const double cell_y =
        grid.origin_y_m + static_cast<double>(y_index) * grid.resolution_m;
      if (!box_intersects_cell(box, cell_x, cell_y, cell_half)) {
        continue;
      }
      sample.contact_cells.push_back(row * grid.width + column);
    }
  }
  std::sort(sample.contact_cells.begin(), sample.contact_cells.end());
  sample.contact_cells.erase(
    std::unique(sample.contact_cells.begin(), sample.contact_cells.end()),
    sample.contact_cells.end());
  return sample;
}

PathClearanceResult evaluate_clear_footprint_path(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const std::vector<Pose2D> & path, const double swept_step_m)
{
  PathClearanceResult result;
  if (!grid.valid()) {
    result.reason = RejectReason::InvalidGrid;
    return result;
  }
  if (!footprint.valid()) {
    result.reason = RejectReason::InvalidFootprint;
    return result;
  }
  if (
    path.empty() || !finite(swept_step_m) || swept_step_m <= 0.0 ||
    swept_step_m > grid.resolution_m + kNumericalEpsilon)
  {
    result.reason = RejectReason::InvalidRollout;
    return result;
  }
  if (!valid_pose(path.front())) {
    result.reason = RejectReason::InvalidInitialPose;
    return result;
  }
  if (!std::all_of(path.begin() + 1, path.end(), valid_pose)) {
    result.reason = RejectReason::InvalidRollout;
    return result;
  }

  result.valid = true;
  const auto evaluate_pose =
    [&](const Pose2D & pose, const std::size_t path_index,
      const std::size_t substep, const std::size_t subdivision_count,
      const double segment_ratio) {
      const auto sample = sample_footprint(grid, footprint, pose);
      ++result.checked_pose_count;
      result.rejected_path_index = path_index;
      result.rejected_pose_available = true;
      result.rejected_pose = pose;
      result.rejected_segment_substep = substep;
      result.rejected_segment_subdivision_count = subdivision_count;
      result.rejected_segment_ratio = segment_ratio;
      if (!sample.valid) {
        result.reason = RejectReason::InvalidInitialPose;
        return false;
      }
      if (sample.out_of_map) {
        result.reason =
          path_index == 0U ? RejectReason::InitialOutOfMap : RejectReason::OutOfMap;
        return false;
      }
      if (!sample.contact_cells.empty()) {
        result.reason = RejectReason::Collision;
        return false;
      }
      return true;
    };

  if (!evaluate_pose(path.front(), 0U, 0U, 0U, 0.0)) {
    return result;
  }

  const double corner_radius = footprint_corner_radius(footprint);
  for (std::size_t path_index = 1U; path_index < path.size(); ++path_index) {
    const auto & from = path[path_index - 1U];
    const auto & to = path[path_index];
    const double translation = std::hypot(to.x_m - from.x_m, to.y_m - from.y_m);
    const double yaw_delta = wrap_to_pi(to.yaw_rad - from.yaw_rad);
    const double maximum_corner_motion = translation + corner_radius * std::abs(yaw_delta);
    const auto subdivisions = subdivision_count(maximum_corner_motion, swept_step_m);
    if (
      !subdivisions.has_value() ||
      result.checked_pose_count > kMaximumSamples - subdivisions.value())
    {
      result.reason = RejectReason::SampleLimitExceeded;
      result.rejected_path_index = path_index;
      return result;
    }

    for (std::size_t substep = 1U; substep <= subdivisions.value(); ++substep) {
      const double ratio =
        static_cast<double>(substep) / static_cast<double>(subdivisions.value());
      const Pose2D pose{
        from.x_m + ratio * (to.x_m - from.x_m),
        from.y_m + ratio * (to.y_m - from.y_m),
        wrap_to_pi(from.yaw_rad + ratio * yaw_delta)};
      if (!evaluate_pose(
          pose, path_index, substep, subdivisions.value(), ratio))
      {
        return result;
      }
    }
  }

  result.clear = true;
  result.reason = RejectReason::None;
  result.rejected_path_index = path.size() - 1U;
  result.rejected_pose_available = false;
  result.rejected_segment_substep = 0U;
  result.rejected_segment_subdivision_count = 0U;
  result.rejected_segment_ratio = std::numeric_limits<double>::quiet_NaN();
  return result;
}

MarginEscapePathClearanceResult evaluate_clearance_margin_escape_path(
  const OccupancyGrid & grid, const FootprintExtents & physical_footprint,
  const FootprintExtents & clearance_footprint,
  const std::vector<Pose2D> & path, const double swept_step_m,
  const bool allow_initial_margin_escape,
  const double maximum_margin_escape_distance_m)
{
  MarginEscapePathClearanceResult result;
  if (
    !grid.valid() || !physical_footprint.valid() || !clearance_footprint.valid() ||
    path.empty() || !finite(swept_step_m) || swept_step_m <= 0.0 ||
    swept_step_m > grid.resolution_m + kNumericalEpsilon ||
    !finite(maximum_margin_escape_distance_m) ||
    maximum_margin_escape_distance_m < 0.0)
  {
    return result;
  }

  const auto initial_sample = sample_footprint(
    grid, clearance_footprint, path.front());
  result.margin_checked_pose_count = 1U;
  result.rejected_path_index = 0U;
  if (!initial_sample.valid) {
    result.margin_reason = RejectReason::InvalidInitialPose;
    return result;
  }
  result.valid = true;
  if (initial_sample.out_of_map) {
    result.margin_reason = RejectReason::InitialOutOfMap;
    result.reason = MarginEscapePathReason::MarginPathBlocked;
    return result;
  }

  result.initial_margin_contact_count = initial_sample.contact_cells.size();
  result.maximum_margin_contact_count = initial_sample.contact_cells.size();
  result.final_margin_contact_count = initial_sample.contact_cells.size();
  if (initial_sample.contact_cells.empty()) {
    const auto margin_clearance = evaluate_clear_footprint_path(
      grid, clearance_footprint, path, swept_step_m);
    result.margin_reason = margin_clearance.reason;
    result.margin_checked_pose_count = margin_clearance.checked_pose_count;
    result.rejected_path_index = margin_clearance.rejected_path_index;
    result.clear = margin_clearance.valid && margin_clearance.clear;
    result.reason = result.clear ?
      MarginEscapePathReason::None : MarginEscapePathReason::MarginPathBlocked;
    return result;
  }
  if (!allow_initial_margin_escape) {
    result.margin_reason = RejectReason::Collision;
    result.reason = MarginEscapePathReason::MarginEscapeNotAllowed;
    return result;
  }

  // Only the exceptional inherited-margin case needs a second full sweep.
  // A fully clear clearance footprint already contains the physical body, so
  // evaluating both paths in the common case would double preflight cost.
  const auto physical_clearance = evaluate_clear_footprint_path(
    grid, physical_footprint, path, swept_step_m);
  result.physical_reason = physical_clearance.reason;
  result.physical_checked_pose_count = physical_clearance.checked_pose_count;
  result.rejected_path_index = physical_clearance.rejected_path_index;
  if (!physical_clearance.valid) {
    result.valid = false;
    return result;
  }
  if (!physical_clearance.clear) {
    result.reason = MarginEscapePathReason::PhysicalPathBlocked;
    return result;
  }

  result.margin_escape_used = true;
  bool margin_cleared = false;
  double travelled_distance_m = 0.0;
  Pose2D previous_sample_pose = path.front();
  const double corner_radius = footprint_corner_radius(clearance_footprint);
  for (std::size_t path_index = 1U; path_index < path.size(); ++path_index) {
    const auto & from = path[path_index - 1U];
    const auto & to = path[path_index];
    const double translation = std::hypot(to.x_m - from.x_m, to.y_m - from.y_m);
    const double yaw_delta = wrap_to_pi(to.yaw_rad - from.yaw_rad);
    const double maximum_corner_motion = translation + corner_radius * std::abs(yaw_delta);
    const auto subdivisions = subdivision_count(maximum_corner_motion, swept_step_m);
    if (
      !subdivisions.has_value() ||
      result.margin_checked_pose_count > kMaximumSamples - subdivisions.value())
    {
      result.valid = false;
      result.margin_reason = RejectReason::SampleLimitExceeded;
      result.reason = MarginEscapePathReason::InvalidInput;
      result.rejected_path_index = path_index;
      return result;
    }

    for (std::size_t substep = 1U; substep <= subdivisions.value(); ++substep) {
      const double ratio =
        static_cast<double>(substep) / static_cast<double>(subdivisions.value());
      const Pose2D pose{
        from.x_m + ratio * (to.x_m - from.x_m),
        from.y_m + ratio * (to.y_m - from.y_m),
        wrap_to_pi(from.yaw_rad + ratio * yaw_delta)};
      travelled_distance_m += std::hypot(
        pose.x_m - previous_sample_pose.x_m,
        pose.y_m - previous_sample_pose.y_m);
      previous_sample_pose = pose;

      const auto sample = sample_footprint(grid, clearance_footprint, pose);
      ++result.margin_checked_pose_count;
      result.rejected_path_index = path_index;
      if (!sample.valid) {
        result.valid = false;
        result.margin_reason = RejectReason::InvalidInitialPose;
        result.reason = MarginEscapePathReason::InvalidInput;
        return result;
      }
      if (sample.out_of_map) {
        result.margin_reason = RejectReason::OutOfMap;
        result.reason = MarginEscapePathReason::MarginPathBlocked;
        return result;
      }

      const std::size_t contact_count = sample.contact_cells.size();
      result.maximum_margin_contact_count = std::max(
        result.maximum_margin_contact_count, contact_count);
      result.final_margin_contact_count = contact_count;
      if (margin_cleared) {
        if (contact_count != 0U) {
          result.margin_reason = RejectReason::NewContact;
          result.reason = MarginEscapePathReason::MarginRecontact;
          return result;
        }
        continue;
      }

      // Margin contacts may move to neighbouring cells while the vehicle
      // travels parallel to one continuous wall, and the count may gain one
      // boundary cell due to rasterization.  Treating that as physical
      // worsening would reject the intended centreward escape.  The physical
      // footprint sweep above remains the hard safety contract; the margin
      // layer must clear within the bounded distance and remain clear.
      if (contact_count == 0U) {
        margin_cleared = true;
        result.margin_clear_path_index = path_index;
        result.margin_clear_distance_m = travelled_distance_m;
        result.margin_reason = RejectReason::None;
        continue;
      }
      if (
        travelled_distance_m >
        maximum_margin_escape_distance_m + kNumericalEpsilon)
      {
        result.margin_reason = RejectReason::InitialContactNotCleared;
        result.reason = MarginEscapePathReason::MarginNotCleared;
        return result;
      }
    }
  }

  if (!margin_cleared) {
    result.margin_reason = RejectReason::InitialContactNotCleared;
    result.reason = MarginEscapePathReason::MarginNotCleared;
    return result;
  }
  if (
    result.margin_clear_distance_m >
    maximum_margin_escape_distance_m + kNumericalEpsilon)
  {
    result.margin_reason = RejectReason::InitialContactNotCleared;
    result.reason = MarginEscapePathReason::MarginNotCleared;
    return result;
  }

  result.clear = true;
  result.reason = MarginEscapePathReason::None;
  result.margin_reason = RejectReason::None;
  result.rejected_path_index = path.size() - 1U;
  return result;
}

LateralClearanceResult clamp_lateral_offset_to_static_map(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & reference_pose, const double desired_lateral_offset_m,
  const double fallback_lateral_offset_m, const double additional_lateral_clearance_m,
  const double sample_step_m)
{
  return clamp_lateral_offset_to_static_map_with_heading(
    grid, footprint, reference_pose, desired_lateral_offset_m,
    fallback_lateral_offset_m, 0.0, additional_lateral_clearance_m,
    sample_step_m);
}

LateralClearanceResult clamp_lateral_offset_to_static_map_with_heading(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & reference_pose, const double desired_lateral_offset_m,
  const double fallback_lateral_offset_m, const double path_heading_offset_rad,
  const double additional_lateral_clearance_m, const double sample_step_m)
{
  LateralClearanceResult result;
  result.lateral_offset_m = fallback_lateral_offset_m;
  if (
    !grid.valid() || !footprint.valid() || !valid_pose(reference_pose) ||
    !finite(desired_lateral_offset_m) || !finite(fallback_lateral_offset_m) ||
    !finite(path_heading_offset_rad) ||
    !finite(additional_lateral_clearance_m) || additional_lateral_clearance_m < 0.0 ||
    !finite(sample_step_m) || sample_step_m <= 0.0)
  {
    return result;
  }

  recovery_footprint::FootprintExtents clearance_footprint = footprint;
  clearance_footprint.left_extent_m += additional_lateral_clearance_m;
  clearance_footprint.right_extent_m += additional_lateral_clearance_m;
  if (!clearance_footprint.valid()) {
    return result;
  }

  const double lateral_distance =
    std::abs(fallback_lateral_offset_m - desired_lateral_offset_m);
  const auto segment_count = subdivision_count(lateral_distance, sample_step_m);
  if (!segment_count.has_value() || segment_count.value() + 1U > kMaximumSamples) {
    return result;
  }

  result.valid = true;
  const double left_x = -std::sin(reference_pose.yaw_rad);
  const double left_y = std::cos(reference_pose.yaw_rad);
  for (std::size_t index = 0U; index <= segment_count.value(); ++index) {
    const double ratio = segment_count.value() == 0U ? 1.0 :
      static_cast<double>(index) / static_cast<double>(segment_count.value());
    const double lateral_offset = desired_lateral_offset_m +
      ratio * (fallback_lateral_offset_m - desired_lateral_offset_m);
    const Pose2D candidate_pose{
      reference_pose.x_m + lateral_offset * left_x,
      reference_pose.y_m + lateral_offset * left_y,
      wrap_to_pi(reference_pose.yaw_rad + path_heading_offset_rad)};
    const auto sample = sample_footprint(grid, clearance_footprint, candidate_pose);
    ++result.checked_pose_count;
    if (sample.valid && !sample.out_of_map && sample.contact_cells.empty()) {
      result.feasible = true;
      result.adjusted = std::abs(lateral_offset - desired_lateral_offset_m) > 1e-9;
      result.lateral_offset_m = lateral_offset;
      return result;
    }
  }
  return result;
}

LateralProfileClearanceResult evaluate_lateral_profile_static_map(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const double current_lateral_offset_m,
  const std::vector<LateralProfileSample> & samples,
  const double additional_lateral_clearance_m)
{
  LateralProfileClearanceResult result;
  if (
    !grid.valid() || !footprint.valid() || !finite(current_lateral_offset_m) ||
    samples.empty() || !finite(additional_lateral_clearance_m) ||
    additional_lateral_clearance_m < 0.0)
  {
    return result;
  }

  auto clearance_footprint = footprint;
  clearance_footprint.left_extent_m += additional_lateral_clearance_m;
  clearance_footprint.right_extent_m += additional_lateral_clearance_m;
  if (!clearance_footprint.valid()) {
    return result;
  }

  double previous_distance_m = 0.0;
  double previous_lateral_m = current_lateral_offset_m;
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    const auto & profile_sample = samples[index];
    const double delta_s_m = profile_sample.path_distance_m - previous_distance_m;
    if (
      !valid_pose(profile_sample.reference_pose) ||
      !finite(profile_sample.path_distance_m) || delta_s_m <= kNumericalEpsilon ||
      !finite(profile_sample.lateral_offset_m) ||
      !finite(profile_sample.base_curvature_radpm))
    {
      result.rejected_path_index = index;
      return result;
    }

    const double lateral_gradient =
      (profile_sample.lateral_offset_m - previous_lateral_m) / delta_s_m;
    double tangential_scale =
      1.0 - profile_sample.base_curvature_radpm * profile_sample.lateral_offset_m;
    if (std::abs(tangential_scale) < 1e-3) {
      tangential_scale = std::copysign(
        1e-3, tangential_scale == 0.0 ? 1.0 : tangential_scale);
    }
    const double heading_offset_rad = std::atan2(lateral_gradient, tangential_scale);
    const double base_yaw = profile_sample.reference_pose.yaw_rad;
    const Pose2D pose{
      profile_sample.reference_pose.x_m -
      profile_sample.lateral_offset_m * std::sin(base_yaw),
      profile_sample.reference_pose.y_m +
      profile_sample.lateral_offset_m * std::cos(base_yaw),
      wrap_to_pi(base_yaw + heading_offset_rad)};
    const auto footprint_sample = sample_footprint(grid, clearance_footprint, pose);
    ++result.checked_pose_count;
    if (
      !footprint_sample.valid || footprint_sample.out_of_map ||
      !footprint_sample.contact_cells.empty())
    {
      result.valid = true;
      result.rejected_path_index = index;
      result.rejected_heading_offset_rad = heading_offset_rad;
      return result;
    }
    previous_distance_m = profile_sample.path_distance_m;
    previous_lateral_m = profile_sample.lateral_offset_m;
  }

  result.valid = true;
  result.clear = true;
  result.rejected_path_index = samples.size() - 1U;
  return result;
}

LateralClearRunsResult find_clear_lateral_runs_with_heading(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & reference_pose, const double lower_lateral_offset_m,
  const double upper_lateral_offset_m, const double path_heading_offset_rad,
  const double additional_lateral_clearance_m, const double sample_step_m)
{
  LateralClearRunsResult result;
  if (
    !grid.valid() || !footprint.valid() || !valid_pose(reference_pose) ||
    !finite(lower_lateral_offset_m) || !finite(upper_lateral_offset_m) ||
    upper_lateral_offset_m < lower_lateral_offset_m ||
    !finite(path_heading_offset_rad) ||
    !finite(additional_lateral_clearance_m) || additional_lateral_clearance_m < 0.0 ||
    !finite(sample_step_m) || sample_step_m <= 0.0)
  {
    return result;
  }

  FootprintExtents clearance_footprint = footprint;
  clearance_footprint.left_extent_m += additional_lateral_clearance_m;
  clearance_footprint.right_extent_m += additional_lateral_clearance_m;
  if (!clearance_footprint.valid()) {
    return result;
  }

  const auto segment_count = subdivision_count(
    upper_lateral_offset_m - lower_lateral_offset_m, sample_step_m);
  if (!segment_count.has_value() || segment_count.value() + 1U > kMaximumSamples) {
    return result;
  }

  bool active_run{false};
  double active_run_lower{0.0};
  const double left_x = -std::sin(reference_pose.yaw_rad);
  const double left_y = std::cos(reference_pose.yaw_rad);
  result.valid = true;
  for (std::size_t index = 0U; index <= segment_count.value(); ++index) {
    const double ratio = segment_count.value() == 0U ? 0.0 :
      static_cast<double>(index) / static_cast<double>(segment_count.value());
    const double lateral_offset = lower_lateral_offset_m +
      ratio * (upper_lateral_offset_m - lower_lateral_offset_m);
    const Pose2D candidate_pose{
      reference_pose.x_m + lateral_offset * left_x,
      reference_pose.y_m + lateral_offset * left_y,
      wrap_to_pi(reference_pose.yaw_rad + path_heading_offset_rad)};
    const auto sample = sample_footprint(grid, clearance_footprint, candidate_pose);
    ++result.checked_pose_count;
    const bool clear =
      sample.valid && !sample.out_of_map && sample.contact_cells.empty();
    if (clear && !active_run) {
      active_run = true;
      active_run_lower = lateral_offset;
    }
    if (!clear && active_run) {
      const double previous_ratio = index == 0U ? 0.0 :
        static_cast<double>(index - 1U) /
        static_cast<double>(segment_count.value());
      const double previous_lateral = lower_lateral_offset_m +
        previous_ratio * (upper_lateral_offset_m - lower_lateral_offset_m);
      result.clear_runs.push_back(
        LateralClearRun{active_run_lower, previous_lateral});
      active_run = false;
    }
  }
  if (active_run) {
    result.clear_runs.push_back(
      LateralClearRun{active_run_lower, upper_lateral_offset_m});
  }
  return result;
}

LateralClearIntervalResult select_lateral_clear_interval(
  const LateralClearRunsResult & runs, const double lower_lateral_offset_m,
  const double upper_lateral_offset_m, const double preferred_lateral_offset_m,
  const double boundary_guard_m)
{
  LateralClearIntervalResult result;
  if (
    !runs.valid || !finite(lower_lateral_offset_m) ||
    !finite(upper_lateral_offset_m) ||
    upper_lateral_offset_m < lower_lateral_offset_m ||
    !finite(preferred_lateral_offset_m) ||
    !finite(boundary_guard_m) || boundary_guard_m < 0.0)
  {
    return result;
  }
  result.valid = true;
  result.checked_pose_count = runs.checked_pose_count;

  std::vector<LateralClearRun> intersected_runs;
  intersected_runs.reserve(runs.clear_runs.size());
  for (const auto & run : runs.clear_runs) {
    if (
      !finite(run.lower_lateral_offset_m) ||
      !finite(run.upper_lateral_offset_m) ||
      run.upper_lateral_offset_m < run.lower_lateral_offset_m)
    {
      return LateralClearIntervalResult{};
    }
    const double lower = std::max(
      lower_lateral_offset_m,
      run.lower_lateral_offset_m + boundary_guard_m);
    const double upper = std::min(
      upper_lateral_offset_m,
      run.upper_lateral_offset_m - boundary_guard_m);
    if (upper + kNumericalEpsilon >= lower) {
      intersected_runs.push_back(LateralClearRun{lower, upper});
    }
  }
  if (intersected_runs.empty()) {
    return result;
  }

  const auto distance_to_run = [&](const LateralClearRun & run) {
      if (preferred_lateral_offset_m < run.lower_lateral_offset_m) {
        return run.lower_lateral_offset_m - preferred_lateral_offset_m;
      }
      if (preferred_lateral_offset_m > run.upper_lateral_offset_m) {
        return preferred_lateral_offset_m - run.upper_lateral_offset_m;
      }
      return 0.0;
    };
  const auto selected = std::min_element(
    intersected_runs.begin(), intersected_runs.end(),
    [&](const LateralClearRun & lhs, const LateralClearRun & rhs) {
      const double lhs_distance = distance_to_run(lhs);
      const double rhs_distance = distance_to_run(rhs);
      if (std::abs(lhs_distance - rhs_distance) > kNumericalEpsilon) {
        return lhs_distance < rhs_distance;
      }
      return
        lhs.upper_lateral_offset_m - lhs.lower_lateral_offset_m >
        rhs.upper_lateral_offset_m - rhs.lower_lateral_offset_m;
    });
  result.feasible = true;
  result.lower_lateral_offset_m = selected->lower_lateral_offset_m;
  result.upper_lateral_offset_m = selected->upper_lateral_offset_m;
  result.preferred_lateral_contained =
    preferred_lateral_offset_m + kNumericalEpsilon >=
    selected->lower_lateral_offset_m &&
    preferred_lateral_offset_m <=
    selected->upper_lateral_offset_m + kNumericalEpsilon;
  return result;
}

LateralClearIntervalResult find_clear_lateral_interval_with_heading(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & reference_pose, const double lower_lateral_offset_m,
  const double upper_lateral_offset_m, const double preferred_lateral_offset_m,
  const double path_heading_offset_rad,
  const double additional_lateral_clearance_m, const double sample_step_m)
{
  const auto runs = find_clear_lateral_runs_with_heading(
    grid, footprint, reference_pose, lower_lateral_offset_m,
    upper_lateral_offset_m, path_heading_offset_rad,
    additional_lateral_clearance_m, sample_step_m);
  return select_lateral_clear_interval(
    runs, lower_lateral_offset_m, upper_lateral_offset_m,
    preferred_lateral_offset_m);
}

LateralClearIntervalResult find_clear_lateral_interval(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & reference_pose, const double lower_lateral_offset_m,
  const double upper_lateral_offset_m, const double preferred_lateral_offset_m,
  const double additional_lateral_clearance_m, const double sample_step_m)
{
  return find_clear_lateral_interval_with_heading(
    grid, footprint, reference_pose, lower_lateral_offset_m,
    upper_lateral_offset_m, preferred_lateral_offset_m, 0.0,
    additional_lateral_clearance_m, sample_step_m);
}

WallProximityResult classify_nearby_wall(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & pose, const double search_margin_m, const double ambiguity_m)
{
  WallProximityResult result;
  result.nearest_distance_m = std::numeric_limits<double>::infinity();
  if (
    !grid.valid() || !footprint.valid() || !valid_pose(pose) ||
    !finite(search_margin_m) || search_margin_m < 0.0 ||
    !finite(ambiguity_m) || ambiguity_m < 0.0)
  {
    return result;
  }

  FootprintExtents search_footprint = footprint;
  search_footprint.margin_m += search_margin_m;
  const FootprintSample nearby = sample_footprint(grid, search_footprint, pose);
  const FootprintSample current = sample_footprint(grid, footprint, pose);
  if (!nearby.valid || nearby.out_of_map || !current.valid || current.out_of_map) {
    return result;
  }
  result.valid = true;
  result.nearby_cell_count = nearby.contact_cells.size();
  result.intersects_footprint = !current.contact_cells.empty();
  if (nearby.contact_cells.empty()) {
    result.region = WallRegion::None;
    return result;
  }

  const double forward_x = std::cos(pose.yaw_rad);
  const double forward_y = std::sin(pose.yaw_rad);
  const double left_x = -forward_y;
  const double left_y = forward_x;
  const double front = footprint.front_extent_m + footprint.margin_m;
  const double rear = footprint.rear_extent_m + footprint.margin_m;
  const double left = footprint.left_extent_m + footprint.margin_m;
  const double right = footprint.right_extent_m + footprint.margin_m;
  const double cell_projection = 0.5 * grid.resolution_m *
    (std::abs(forward_x) + std::abs(forward_y));

  std::array<double, 4> nearest{
    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  for (const std::size_t cell_index : nearby.contact_cells) {
    if (cell_index >= grid.cells.size()) {
      result.valid = false;
      result.region = WallRegion::Unknown;
      return result;
    }
    const auto cell_center = grid.grid_to_world(
      cell_index / grid.width, cell_index % grid.width);
    if (!cell_center.has_value()) {
      result.valid = false;
      result.region = WallRegion::Unknown;
      return result;
    }
    const double dx = cell_center->x_m - pose.x_m;
    const double dy = cell_center->y_m - pose.y_m;
    const double local_forward = dx * forward_x + dy * forward_y;
    const double local_left = dx * left_x + dy * left_y;

    const double front_gap = std::max(0.0, local_forward - front - cell_projection);
    const double rear_gap = std::max(0.0, -rear - local_forward - cell_projection);
    const double left_gap = std::max(0.0, local_left - left - cell_projection);
    const double right_gap = std::max(0.0, -right - local_left - cell_projection);
    const bool longitudinal_inside = local_forward >= -rear && local_forward <= front;
    const bool lateral_inside = local_left >= -right && local_left <= left;

    std::size_t side = 0U;
    double gap = 0.0;
    if (!longitudinal_inside && !lateral_inside) {
      const double longitudinal_gap = local_forward > front ? front_gap : rear_gap;
      const double lateral_gap = local_left > left ? left_gap : right_gap;
      if (std::abs(longitudinal_gap - lateral_gap) <= ambiguity_m) {
        // A true corner contact cannot prove a unique escape direction.
        continue;
      }
      if (longitudinal_gap < lateral_gap) {
        side = local_forward > front ? 0U : 1U;
        gap = longitudinal_gap;
      } else {
        side = local_left > left ? 2U : 3U;
        gap = lateral_gap;
      }
    } else if (!longitudinal_inside) {
      side = local_forward > front ? 0U : 1U;
      gap = local_forward > front ? front_gap : rear_gap;
    } else if (!lateral_inside) {
      side = local_left > left ? 2U : 3U;
      gap = local_left > left ? left_gap : right_gap;
    } else {
      // Cell geometry intersects the footprint while its centre is inside it.
      const double longitudinal_ratio = local_forward >= 0.0 ?
        local_forward / std::max(front, kNumericalEpsilon) :
        -local_forward / std::max(rear, kNumericalEpsilon);
      const double lateral_ratio = local_left >= 0.0 ?
        local_left / std::max(left, kNumericalEpsilon) :
        -local_left / std::max(right, kNumericalEpsilon);
      if (std::abs(longitudinal_ratio - lateral_ratio) <= kNumericalEpsilon) {
        continue;
      }
      side = longitudinal_ratio > lateral_ratio ?
        (local_forward >= 0.0 ? 0U : 1U) : (local_left >= 0.0 ? 2U : 3U);
      gap = 0.0;
    }
    nearest[side] = std::min(nearest[side], gap);
  }

  const auto best = std::min_element(nearest.begin(), nearest.end());
  if (best == nearest.end() || !std::isfinite(*best)) {
    result.region = WallRegion::Mixed;
    return result;
  }
  result.nearest_distance_m = *best;
  const std::size_t near_side_count = static_cast<std::size_t>(std::count_if(
      nearest.begin(), nearest.end(), [&](const double distance) {
        return std::isfinite(distance) && distance <= *best + ambiguity_m;
      }));
  if (near_side_count != 1U) {
    result.region = WallRegion::Mixed;
    return result;
  }
  constexpr std::array<WallRegion, 4> kRegions{
    WallRegion::Front, WallRegion::Rear, WallRegion::Left, WallRegion::Right};
  result.region = kRegions[static_cast<std::size_t>(std::distance(nearest.begin(), best))];
  return result;
}

RejectReason evaluate_contact_transition(
  const OccupancyGrid & grid,
  const std::vector<std::size_t> & initial_contact_cells,
  const std::vector<std::size_t> & previous_contact_cells,
  const std::vector<std::size_t> & current_contact_cells) noexcept
{
  if (!grid.valid()) {
    return RejectReason::InvalidGrid;
  }

  const auto is_explicitly_occupied = [&grid](const std::size_t cell_index) {
      if (cell_index >= grid.cells.size()) {
        return false;
      }
      return grid.cell(cell_index / grid.width, cell_index % grid.width) == CellState::Occupied;
    };
  if (
    !std::all_of(
      initial_contact_cells.begin(), initial_contact_cells.end(), is_explicitly_occupied) ||
    !std::all_of(
      previous_contact_cells.begin(), previous_contact_cells.end(), is_explicitly_occupied) ||
    !std::all_of(
      current_contact_cells.begin(), current_contact_cells.end(), is_explicitly_occupied))
  {
    return RejectReason::Collision;
  }
  const auto absolute_difference = [](const std::size_t lhs, const std::size_t rhs) {
      return lhs > rhs ? lhs - rhs : rhs - lhs;
    };
  const auto touches_patch = [&](
    const std::size_t cell, const std::vector<std::size_t> & patch) {
      const std::size_t row = cell / grid.width;
      const std::size_t column = cell % grid.width;
      return std::any_of(
        patch.begin(), patch.end(), [&](const std::size_t patch_cell) {
          return absolute_difference(row, patch_cell / grid.width) <= 1U &&
          absolute_difference(column, patch_cell % grid.width) <= 1U;
        });
    };
  if (
    !std::all_of(
      previous_contact_cells.begin(), previous_contact_cells.end(),
      [&](const std::size_t cell) {return touches_patch(cell, initial_contact_cells);}))
  {
    return RejectReason::NewContact;
  }
  if (current_contact_cells.empty()) {
    return RejectReason::None;
  }
  if (initial_contact_cells.empty() || previous_contact_cells.empty()) {
    return RejectReason::NewContact;
  }
  if (current_contact_cells.size() > previous_contact_cells.size()) {
    return RejectReason::ContactWorsened;
  }

  for (const std::size_t current : current_contact_cells) {
    if (
      !touches_patch(current, initial_contact_cells) ||
      !touches_patch(current, previous_contact_cells))
    {
      return RejectReason::NewContact;
    }
  }
  return RejectReason::None;
}

RejectReason evaluate_improving_contact_transition(
  const OccupancyGrid & grid,
  const std::vector<std::size_t> & initial_contact_cells,
  const std::vector<std::size_t> & previous_contact_cells,
  const std::vector<std::size_t> & current_contact_cells) noexcept
{
  if (!grid.valid()) {
    return RejectReason::InvalidGrid;
  }
  const auto is_explicitly_occupied = [&grid](const std::size_t cell_index) {
      return cell_index < grid.cells.size() &&
             grid.cell(cell_index / grid.width, cell_index % grid.width) == CellState::Occupied;
    };
  if (
    !std::all_of(initial_contact_cells.begin(), initial_contact_cells.end(), is_explicitly_occupied) ||
    !std::all_of(previous_contact_cells.begin(), previous_contact_cells.end(), is_explicitly_occupied) ||
    !std::all_of(current_contact_cells.begin(), current_contact_cells.end(), is_explicitly_occupied))
  {
    return RejectReason::Collision;
  }
  if (current_contact_cells.empty()) {
    return RejectReason::None;
  }
  if (initial_contact_cells.empty() || previous_contact_cells.empty()) {
    return RejectReason::NewContact;
  }
  if (current_contact_cells.size() > initial_contact_cells.size()) {
    return RejectReason::ContactWorsened;
  }

  const auto absolute_difference = [](const std::size_t lhs, const std::size_t rhs) {
      return lhs > rhs ? lhs - rhs : rhs - lhs;
    };
  for (const std::size_t current : current_contact_cells) {
    const std::size_t row = current / grid.width;
    const std::size_t column = current % grid.width;
    const bool touches_previous = std::any_of(
      previous_contact_cells.begin(), previous_contact_cells.end(),
      [&](const std::size_t previous) {
        return absolute_difference(row, previous / grid.width) <= 1U &&
               absolute_difference(column, previous % grid.width) <= 1U;
      });
    if (!touches_previous) {
      return RejectReason::NewContact;
    }
  }
  return RejectReason::None;
}

FeasibilityResult evaluate_recovery_candidate(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & initial_pose, const ReversePrimitive primitive,
  const ReverseRolloutParameters & parameters, const ContactEscapePolicy policy,
  const double minimum_contact_reduction_ratio)
{
  FeasibilityResult result;
  result.primitive = primitive;
  result.steering_angle_rad = steering_for(primitive, parameters);
  if (!grid.valid()) {
    return rejected(std::move(result), RejectReason::InvalidGrid, 0.0);
  }
  if (!footprint.valid()) {
    return rejected(std::move(result), RejectReason::InvalidFootprint, 0.0);
  }
  if (!valid_pose(initial_pose)) {
    return rejected(std::move(result), RejectReason::InvalidInitialPose, 0.0);
  }
  if (!valid_rollout_parameters(primitive, parameters)) {
    return rejected(std::move(result), RejectReason::InvalidRollout, 0.0);
  }
  if (
    (policy != ContactEscapePolicy::RequireClear &&
    policy != ContactEscapePolicy::RequireImprovement &&
    policy != ContactEscapePolicy::AllowNonWorsening) ||
    !std::isfinite(minimum_contact_reduction_ratio) ||
    minimum_contact_reduction_ratio < 0.0 || minimum_contact_reduction_ratio > 1.0 ||
    (policy == ContactEscapePolicy::RequireImprovement &&
    minimum_contact_reduction_ratio <= 0.0))
  {
    return rejected(std::move(result), RejectReason::InvalidRollout, 0.0);
  }
  if (parameters.swept_step_m > grid.resolution_m + kNumericalEpsilon) {
    // The one-cell contact-patch continuity rule is conservative only when each swept sample
    // moves every vehicle corner by no more than one map cell.
    return rejected(std::move(result), RejectReason::InvalidRollout, 0.0);
  }

  RolloutResult rollout = generate_reverse_rollout(initial_pose, primitive, parameters);
  if (!rollout.valid) {
    return rejected(std::move(result), rollout.reason, 0.0);
  }
  result.rollout = std::move(rollout.poses);

  const FootprintSample initial_sample = sample_footprint(grid, footprint, initial_pose);
  if (!initial_sample.valid) {
    return rejected(std::move(result), RejectReason::InvalidInitialPose, 0.0);
  }
  result.checked_pose_count = 1U;
  if (initial_sample.out_of_map) {
    return rejected(std::move(result), RejectReason::InitialOutOfMap, 0.0);
  }

  const std::vector<std::size_t> initial_contacts = initial_sample.contact_cells;
  result.initial_contact_count = initial_contacts.size();
  result.maximum_contact_count = initial_contacts.size();
  result.final_contact_count = initial_contacts.size();
  std::size_t previous_contact_count = initial_contacts.size();
  std::vector<std::size_t> previous_contacts = initial_contacts;
  const double corner_radius = footprint_corner_radius(footprint);
  if (!initial_contacts.empty()) {
    if (
      policy == ContactEscapePolicy::RequireClear &&
      primitive != ReversePrimitive::Straight && primitive != ReversePrimitive::ForwardStraight)
    {
      // A changing body orientation needs a directional penetration metric before an existing
      // contact can be proven to improve. Runtime actuation currently uses Straight only.
      return rejected(
        std::move(result), RejectReason::InvalidRollout, 0.0, initial_contacts.size());
    }
    const RejectReason initial_contact_reason = evaluate_contact_transition(
      grid, initial_contacts, initial_contacts, initial_contacts);
    if (initial_contact_reason != RejectReason::None) {
      return rejected(
        std::move(result), initial_contact_reason, 0.0, initial_contacts.size());
    }
    if (
      policy == ContactEscapePolicy::RequireClear &&
      primitive == ReversePrimitive::Straight &&
      !initial_contact_patch_is_forward(grid, initial_pose, initial_contacts))
    {
      return rejected(
        std::move(result), RejectReason::InitialContactNotForward, 0.0,
        initial_contacts.size());
    }
    if (
      policy == ContactEscapePolicy::RequireClear &&
      primitive == ReversePrimitive::ForwardStraight &&
      !initial_contact_patch_is_rear(grid, initial_pose, initial_contacts))
    {
      return rejected(
        std::move(result), RejectReason::InitialContactNotRear, 0.0,
        initial_contacts.size());
    }
  }

  for (std::size_t segment = 1U; segment < result.rollout.size(); ++segment) {
    const auto & from = result.rollout[segment - 1U];
    const auto & to = result.rollout[segment];
    const double translation = std::hypot(
      to.pose.x_m - from.pose.x_m, to.pose.y_m - from.pose.y_m);
    const double yaw_delta = wrap_to_pi(to.pose.yaw_rad - from.pose.yaw_rad);
    const double maximum_corner_motion = translation + corner_radius * std::abs(yaw_delta);
    const auto swept_subdivisions =
      subdivision_count(maximum_corner_motion, parameters.swept_step_m);
    if (
      !swept_subdivisions.has_value() ||
      result.checked_pose_count > kMaximumSamples - swept_subdivisions.value())
    {
      return rejected(
        std::move(result), RejectReason::SampleLimitExceeded,
        from.reverse_distance_m, previous_contact_count);
    }

    for (std::size_t substep = 1U; substep <= swept_subdivisions.value(); ++substep) {
      const double ratio =
        static_cast<double>(substep) / static_cast<double>(swept_subdivisions.value());
      const double distance = from.reverse_distance_m +
        ratio * (to.reverse_distance_m - from.reverse_distance_m);
      const Pose2D pose =
        pose_at_reverse_distance(initial_pose, primitive, parameters, distance);
      const FootprintSample sample = sample_footprint(grid, footprint, pose);
      ++result.checked_pose_count;
      if (!sample.valid) {
        return rejected(
          std::move(result), RejectReason::InvalidInitialPose, distance,
          previous_contact_count);
      }
      if (sample.out_of_map) {
        return rejected(
          std::move(result), RejectReason::OutOfMap, distance,
          previous_contact_count);
      }

      const std::size_t contact_count = sample.contact_cells.size();
      result.maximum_contact_count = std::max(result.maximum_contact_count, contact_count);
      result.final_contact_count = contact_count;
      if (initial_contacts.empty()) {
        if (!sample.contact_cells.empty()) {
          return rejected(
            std::move(result), RejectReason::Collision, distance, contact_count);
        }
      } else {
        const RejectReason transition_reason = policy != ContactEscapePolicy::RequireClear ?
          evaluate_improving_contact_transition(
          grid, initial_contacts, previous_contacts, sample.contact_cells) :
          evaluate_contact_transition(
          grid, initial_contacts, previous_contacts, sample.contact_cells);
        if (transition_reason != RejectReason::None) {
          return rejected(
            std::move(result), transition_reason, distance, contact_count);
        }
      }
      previous_contacts = sample.contact_cells;
      previous_contact_count = contact_count;
    }
  }

  result.contact_reduction = initial_contacts.size() > previous_contact_count ?
    initial_contacts.size() - previous_contact_count : 0U;
  if (
    policy == ContactEscapePolicy::RequireClear && !initial_contacts.empty() &&
    previous_contact_count != 0U)
  {
    return rejected(
      std::move(result), RejectReason::InitialContactNotCleared,
      parameters.reverse_distance_m, previous_contact_count);
  }
  if (policy == ContactEscapePolicy::RequireImprovement) {
    if (initial_contacts.empty()) {
      return rejected(
        std::move(result), RejectReason::ContactNotImproved,
        parameters.reverse_distance_m, previous_contact_count);
    }
    const std::size_t minimum_reduction = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(
        minimum_contact_reduction_ratio * static_cast<double>(initial_contacts.size()))));
    if (result.contact_reduction < minimum_reduction) {
      return rejected(
        std::move(result), RejectReason::ContactNotImproved,
        parameters.reverse_distance_m, previous_contact_count);
    }
  }

  result.feasible = true;
  result.reason = RejectReason::None;
  result.rejected_at_distance_m = 0.0;
  result.final_contact_count = previous_contact_count;
  return result;
}

FeasibilityResult evaluate_reverse_candidate(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & initial_pose, const ReversePrimitive primitive,
  const ReverseRolloutParameters & parameters)
{
  return evaluate_recovery_candidate(
    grid, footprint, initial_pose, primitive, parameters,
    ContactEscapePolicy::RequireClear, 0.0);
}

}  // namespace multi_purpose_mpc_ros::recovery_footprint
