#ifndef MULTI_PURPOSE_MPC_ROS__RECOVERY_FOOTPRINT_HPP_
#define MULTI_PURPOSE_MPC_ROS__RECOVERY_FOOTPRINT_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::recovery_footprint
{

/// Row-major occupancy value. Any value other than Free is treated as occupied.
enum class CellState : std::int8_t
{
  Unknown = -1,
  Free = 0,
  Occupied = 1,
};

/// Relates row indices to the world y-axis.
///
/// RowZeroAtMaximumY matches the PGM/OpenCV convention used by the existing
/// MPC Map: increasing world y decreases the row index.
enum class YAxisConvention
{
  RowZeroAtMinimumY,
  RowZeroAtMaximumY,
};

struct GridIndex
{
  std::size_t row{};
  std::size_t column{};
};

struct Point2D
{
  double x_m{};
  double y_m{};
};

/// Pure row-major grid whose origin denotes the centre of cell (x=0, y=0).
///
/// For RowZeroAtMaximumY, world grid coordinate (0, 0) is stored at
/// row=height-1, column=0, matching Map::w2m/m2w for in-map coordinates.
struct OccupancyGrid
{
  std::size_t width{};
  std::size_t height{};
  double resolution_m{};
  double origin_x_m{};
  double origin_y_m{};
  YAxisConvention y_axis{YAxisConvention::RowZeroAtMaximumY};
  std::vector<CellState> cells;

  bool valid() const noexcept;
  std::optional<GridIndex> world_to_grid(double x_m, double y_m) const noexcept;
  std::optional<Point2D> grid_to_world(std::size_t row, std::size_t column) const noexcept;
  CellState cell(std::size_t row, std::size_t column) const noexcept;
};

struct Pose2D
{
  double x_m{};
  double y_m{};
  double yaw_rad{};
};

/// Vehicle dimensions relative to the supplied pose reference point.
struct FootprintExtents
{
  double front_extent_m{};
  double rear_extent_m{};
  double left_extent_m{};
  double right_extent_m{};
  double margin_m{};

  bool valid() const noexcept;
};

enum class ReversePrimitive
{
  Straight,
  Left,
  Right,
  // Kept in this compatibility enum because the recovery rollout/result API
  // predates bidirectional escape support.
  ForwardStraight,
  ForwardLeft,
  ForwardRight,
};

bool primitive_is_forward(ReversePrimitive primitive) noexcept;
double primitive_steering_sign(ReversePrimitive primitive) noexcept;

enum class WallRegion
{
  None,
  Front,
  Rear,
  Left,
  Right,
  Mixed,
  Unknown,
};

/// Select the bounded stop-and-reassess cadence for a recovery candidate.
///
/// A clear footprint may still have physical/nearby side-wall evidence that
/// the occupancy cells do not classify as an overlap. Such Left/Right/Mixed
/// cases use the same short step as a direct side contact. Clear Front/Rear
/// cases retain their direction-specific bounded maneuver. Simulation-race
/// continuous escape may explicitly bypass short steps only after the current
/// footprint itself is clear.
bool use_stepwise_escape_mode(
  bool side_escape_enabled, bool current_footprint_clear,
  std::size_t current_contact_count, WallRegion wall_region,
  std::size_t completed_escape_steps,
  bool continuous_clear_escape_enabled = false) noexcept;

struct WallProximityResult
{
  bool valid{false};
  WallRegion region{WallRegion::Unknown};
  double nearest_distance_m{};
  std::size_t nearby_cell_count{};
  bool intersects_footprint{false};
};

/// Parameters for a rear-axle kinematic bicycle reverse rollout.
struct ReverseRolloutParameters
{
  double reverse_distance_m{};
  double rollout_step_m{};
  double swept_step_m{};
  double wheelbase_m{};
  /// Non-negative magnitude. Left uses +steering and Right uses -steering.
  double steering_angle_rad{};
};

enum class RejectReason
{
  None,
  InvalidGrid,
  InvalidFootprint,
  InvalidInitialPose,
  InvalidRollout,
  SampleLimitExceeded,
  InitialOutOfMap,
  InitialContactNotForward,
  InitialContactNotRear,
  OutOfMap,
  Collision,
  NewContact,
  ContactWorsened,
  ContactNotImproved,
  InitialContactNotCleared,
};

enum class ContactEscapePolicy
{
  RequireClear,
  RequireImprovement,
  AllowNonWorsening,
};

const char * to_string(RejectReason reason) noexcept;
const char * to_string(ReversePrimitive primitive) noexcept;
const char * to_string(WallRegion region) noexcept;

/// Return deterministic positive steering magnitudes from max/count through max.
/// Invalid, non-positive inputs return an empty list.
std::vector<double> steering_magnitude_samples(
  double maximum_steering_angle_rad, std::size_t sample_count);

struct RolloutPose
{
  Pose2D pose;
  double reverse_distance_m{};
};

struct RolloutResult
{
  bool valid{false};
  RejectReason reason{RejectReason::InvalidRollout};
  std::vector<RolloutPose> poses;
};

/// Occupied cells intersecting an oriented footprint at one pose.
///
/// contact_cells contains row-major indices (row * width + column), sorted in
/// ascending order. Unknown and invalid cell enum values are contacts.
struct FootprintSample
{
  bool valid{false};
  bool out_of_map{false};
  std::vector<std::size_t> contact_cells;
};

/// Result of moving a Frenet lateral target away from a static-map wall.
struct LateralClearanceResult
{
  bool valid{false};
  bool feasible{false};
  bool adjusted{false};
  double lateral_offset_m{};
  std::size_t checked_pose_count{};
};

struct FeasibilityResult
{
  bool feasible{false};
  RejectReason reason{RejectReason::InvalidRollout};
  ReversePrimitive primitive{ReversePrimitive::Straight};
  std::size_t initial_contact_count{};
  std::size_t maximum_contact_count{};
  std::size_t final_contact_count{};
  std::size_t checked_pose_count{};
  std::size_t contact_reduction{};
  double steering_angle_rad{};
  double rejected_at_distance_m{};
  std::vector<RolloutPose> rollout;
};

/// Conservatively inflated V2X vehicle represented as a moving circle.
struct CircleObstacle
{
  double x_m{};
  double y_m{};
  double velocity_x_mps{};
  double velocity_y_mps{};
  double radius_m{};
};

enum class DynamicClearanceRejectReason
{
  None,
  InvalidFootprint,
  InvalidRollout,
  InvalidObstacle,
  NewOverlap,
  InitialOverlapWorsened,
  InitialOverlapNotImproved,
};

const char * to_string(DynamicClearanceRejectReason reason) noexcept;

struct DynamicClearanceResult
{
  bool valid{false};
  bool clear{false};
  DynamicClearanceRejectReason reason{DynamicClearanceRejectReason::InvalidRollout};
  std::size_t checked_pose_count{};
  double initial_clearance_m{};
  double minimum_clearance_m{};
  double final_clearance_m{};
  double rejected_at_distance_m{};
};

/// Evaluate one V2X circle over the selected recovery rollout.
///
/// A clear initial pose must remain clear. If conservative inflation already
/// overlaps the ego footprint at t=0, the overlap may be used as an escape
/// contact only when signed clearance never worsens and improves by the end of
/// the rollout. The obstacle is linearly predicted over prediction_horizon_sec
/// according to each rollout pose's travelled-distance fraction.
DynamicClearanceResult evaluate_circle_obstacle_clearance(
  const FootprintExtents & footprint, const std::vector<RolloutPose> & rollout,
  const CircleObstacle & obstacle, double prediction_horizon_sec);

/// Generate deterministic Straight/Left/Right reverse poses, including t=0.
RolloutResult generate_reverse_rollout(
  const Pose2D & initial_pose, ReversePrimitive primitive,
  const ReverseRolloutParameters & parameters);

/// Conservatively rasterize the full oriented vehicle rectangle.
FootprintSample sample_footprint(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & pose);

/// Find the first collision-free lateral target between desired and fallback.
///
/// reference_pose is the zero-offset path pose. Positive lateral offset is to
/// its left. additional_lateral_clearance_m inflates only the left/right
/// extents, preserving the physical front/rear footprint at bends. Unknown and
/// out-of-map samples are not feasible. The search is deterministic and moves
/// monotonically from desired_lateral_offset_m toward fallback_lateral_offset_m.
LateralClearanceResult clamp_lateral_offset_to_static_map(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & reference_pose, double desired_lateral_offset_m,
  double fallback_lateral_offset_m, double additional_lateral_clearance_m,
  double sample_step_m);

/// Classify the nearest occupied/unknown map cells in the vehicle frame.
///
/// search_margin_m expands the normal vehicle footprint only for wall lookup;
/// it does not change collision or rollout clearance. If equally near cells
/// belong to more than one side within ambiguity_m, the result is Mixed and
/// callers must fail closed rather than guessing an escape direction.
WallProximityResult classify_nearby_wall(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & pose, double search_margin_m, double ambiguity_m);

/// Validate one occupied-contact transition while escaping an initial wall contact.
///
/// Every tracked cell must be explicitly Occupied, the contact count must not increase,
/// and each current cell must be within the fixed one-cell halo of the initial patch as well
/// as the same as or 8-neighbour-adjacent to a previous cell. Once previous_contact_cells is
/// empty, any renewed contact is rejected.
RejectReason evaluate_contact_transition(
  const OccupancyGrid & grid,
  const std::vector<std::size_t> & initial_contact_cells,
  const std::vector<std::size_t> & previous_contact_cells,
  const std::vector<std::size_t> & current_contact_cells) noexcept;

/// Validate a short side/corner escape step.
///
/// Contacts must remain explicitly occupied, never exceed the step's initial
/// contact count, and remain locally connected to the previous swept sample.
/// Unlike evaluate_contact_transition, this permits a contact patch to migrate
/// while the vehicle rotates; the short step limit and required endpoint
/// reduction prevent indefinite sliding along a wall.
RejectReason evaluate_improving_contact_transition(
  const OccupancyGrid & grid,
  const std::vector<std::size_t> & initial_contact_cells,
  const std::vector<std::size_t> & previous_contact_cells,
  const std::vector<std::size_t> & current_contact_cells) noexcept;

FeasibilityResult evaluate_recovery_candidate(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & initial_pose, ReversePrimitive primitive,
  const ReverseRolloutParameters & parameters, ContactEscapePolicy policy,
  double minimum_contact_reduction_ratio);

/// Evaluate static-map safety over a swept recovery rollout.
///
/// An initially clear footprint must remain clear. If the initial pose already
/// contacts explicitly Occupied cells, only a straight primitive is supported.
/// Reverse requires a front contact and ForwardStraight requires a rear contact.
/// Later contacts remain inside the
/// fixed initial-patch halo, form a local non-increasing continuation of the
/// previous patch, and clear before the candidate ends. Out-of-map and unknown
/// cells fail closed.
FeasibilityResult evaluate_reverse_candidate(
  const OccupancyGrid & grid, const FootprintExtents & footprint,
  const Pose2D & initial_pose, ReversePrimitive primitive,
  const ReverseRolloutParameters & parameters);

}  // namespace multi_purpose_mpc_ros::recovery_footprint

#endif  // MULTI_PURPOSE_MPC_ROS__RECOVERY_FOOTPRINT_HPP_
