#include "multi_purpose_mpc_ros/mpcc_vehicle_prediction.hpp"
#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_ADAPTER_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_ADAPTER_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"

#include <Eigen/Dense>

#include <limits>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_adapter
{

/// Numerical initialization only. Both policies leave every QP control free
/// and require the same solved-trajectory and physical certificates.
enum class InitialTangentPolicy
{
  CurrentSteering = 0,
  ReferenceSteeringWithRestLaunch = 1,
};

constexpr bool initial_tangent_policy_valid(InitialTangentPolicy policy) noexcept
{
  return policy == InitialTangentPolicy::CurrentSteering ||
         policy == InitialTangentPolicy::ReferenceSteeringWithRestLaunch;
}

inline constexpr int kLegacyStateDimension = 5;
inline constexpr int kLegacyInputDimension = 3;
inline constexpr int kLegacyCurvatureIndex = 1;

struct StateStage
{
  Eigen::Matrix<double, kLegacyStateDimension, 1> reference{
    Eigen::Matrix<double, kLegacyStateDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyStateDimension, 1> lower{
    Eigen::Matrix<double, kLegacyStateDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyStateDimension, 1> upper{
    Eigen::Matrix<double, kLegacyStateDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyStateDimension, 1> weight{
    Eigen::Matrix<double, kLegacyStateDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyStateDimension, 1> linear_cost{
    Eigen::Matrix<double, kLegacyStateDimension, 1>::Zero()};
};

struct InputStage
{
  Eigen::Matrix<double, kLegacyInputDimension, 1> reference{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyInputDimension, 1> lower{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyInputDimension, 1> upper{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyInputDimension, 1> weight{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyInputDimension, 1> linear_cost{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  double path_curvature_radpm{};
  double stage_dt_sec{};
};

struct Request
{
  int horizon_steps{};
  Eigen::Matrix<double, kLegacyStateDimension, 1> initial_state{
    Eigen::Matrix<double, kLegacyStateDimension, 1>::Zero()};
  /// Physical-equivalent steering command at the control origin.  This is
  /// the state driven exactly by the optimized steering-rate input and the
  /// sole origin used by command extraction.  It is not a measured tire
  /// angle; tire response and body motion are separate dynamic states.
  double current_steering_rad{};
  /// Physical tire angle at the control origin from public stamped steering
  /// observations and the shared published-input predictor. It never initializes
  /// the serialized desired-command trajectory.
  double current_response_steering_rad{};
  double wheelbase_m{};
  double curvature_reference_gain{1.0};
  double maximum_abs_steering_rad{};
  double maximum_abs_steering_rate_radps{};
  double minimum_frenet_denominator{0.20};
  double minimum_stage_dt_sec{0.01};
  double maximum_stage_dt_sec{0.25};
  std::vector<StateStage> states;
  std::vector<InputStage> inputs;
  Eigen::Matrix<double, kLegacyInputDimension, 1> previous_input{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyInputDimension, 1> input_delta_weight{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  mpcc_rate_resolved::CourseFrame course_frame;
  double current_lateral_velocity_mps{};
  double current_yaw_rate_radps{};
  mpcc_vehicle_model::Parameters vehicle_model;
  bool maximum_braking_feasibility{false};
  /// Absent only for explicitly synthetic/offline problems.
  std::optional<mpcc_vehicle_model::ObservationProvenance> observation_provenance;
  InitialTangentPolicy initial_tangent_policy{InitialTangentPolicy::CurrentSteering};
};

struct Result
{
  mpcc_rate_resolved_problem::AssemblyRequest problem;
  std::vector<double> steering_reference_rad;
  std::vector<double> steering_lower_rad;
  std::vector<double> steering_upper_rad;
  std::vector<double> curvature_to_steering_jacobian_radpm_per_rad;
  double first_steering_rate_physical_lower_radps{};
  double first_steering_rate_physical_upper_radps{};
  double first_steering_rate_solver_lower_radps{};
  double first_steering_rate_solver_upper_radps{};
  double first_steering_rate_certificate_margin_radps{};
};

struct CourseFrameProgressSupport {
  double lower_progress_m{};
  double upper_progress_m{};
};

/// Static course data needed by accepted state rows and by exact integration
/// of accepted virtual-speed rows from the semantic initial state. This is
/// an allocation range, not a relaxation of any QP or physical constraint.
std::optional<CourseFrameProgressSupport> resolve_course_frame_progress_support(
    const Request &request, const persistent_osqp::PhysicalConstraintTolerance
                                &solver_tolerance) noexcept;

/// Recognize the sealed feasibility contract: no tracking objective, a fixed
/// maximum wire-braking law, shared-body velocity seeds, and terminal rest.
/// Full execution-horizon ownership is checked by the snapshot consumer.
bool is_braking_feasibility_request(const Request & request) noexcept;

/// Solver-coordinate interval whose accepted residual still lies inside the
/// exact physical boundary.  Any temporal reachability envelope derived from
/// an optimized input must use this interval, not the uninset actuator limit.
struct ExactPhysicalBoundaryBounds
{
  double lower{};
  double upper{};
  double certificate_margin{};
};

std::optional<ExactPhysicalBoundaryBounds>
resolve_exact_physical_boundary_bounds(
  double physical_lower, double physical_upper,
  const persistent_osqp::PhysicalConstraintTolerance & tolerance) noexcept;

/// Shared cumulative steering-delta box for QP rows and fixed-input producers.
std::optional<ExactPhysicalBoundaryBounds> resolve_steering_prefix_bounds(
  double initial_steering_rad, double maximum_abs_steering_rad,
  const persistent_osqp::PhysicalConstraintTolerance & tolerance) noexcept;

enum class RejectReason
{
  None,
  InvalidRequest,
  InvalidStateStage,
  InvalidInputStage,
  InitialStateOutsideBounds,
  SteeringBoundsUnavailable,
  SteeringJacobianUnavailable,
  LinearizationUnavailable,
  AccelerationInsetUnavailable,
  SteeringRateInsetUnavailable,
  SteeringPrefixInsetUnavailable,
};

struct BuildDiagnostic
{
  RejectReason reason{RejectReason::None};
  int stage{-1};
  int element{-1};
  double value{std::numeric_limits<double>::quiet_NaN()};
  double lower{std::numeric_limits<double>::quiet_NaN()};
  double upper{std::numeric_limits<double>::quiet_NaN()};
};

enum class RelinearizationReason
{
  Accepted,
  InvalidRequest,
  InvalidPrimal,
  LinearizationUnavailable,
};

struct RelinearizationResult
{
  RelinearizationReason reason{RelinearizationReason::InvalidRequest};
  int stage{-1};
  bool applied{false};
};

const char * to_string(RejectReason reason) noexcept;

std::optional<Result> build(
  const Request & request,
  const persistent_osqp::PhysicalConstraintTolerance & solver_tolerance,
  BuildDiagnostic * diagnostic = nullptr) noexcept;

/// Replace only the temporal Frenet dynamics with tangents at the current QP
/// iterate.  The solver-certified iterate is projected onto its exact variable
/// boxes solely to select a physical linearization point: an accepted residual
/// may otherwise make a nominally non-negative velocity infinitesimally
/// negative and outside the nonlinear model domain.  The solved trajectory is
/// not clamped or certified by this projection.  Costs, state/input boxes,
/// physical-wall rows and dynamic-obstacle rows remain unchanged, so a second
/// solve is one SQP correction of the same semantic problem rather than a new
/// fallback formulation.
RelinearizationResult relinearize_around_primal(
  const Request & request, const Eigen::VectorXd & primal,
  mpcc_rate_resolved_problem::AssemblyRequest & problem) noexcept;

const char * to_string(RelinearizationReason reason) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_adapter

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_ADAPTER_HPP_
