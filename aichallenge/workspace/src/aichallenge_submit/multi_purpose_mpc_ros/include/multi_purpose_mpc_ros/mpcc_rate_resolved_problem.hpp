#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PROBLEM_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PROBLEM_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include "multi_purpose_mpc_ros/persistent_osqp.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <limits>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_problem
{

namespace model = mpcc_rate_resolved;

/// Exact previous input seen by the rate-resolved problem at a ROS command
/// publication boundary.  Acceleration and desired steering angle are wire
/// actuation values.  Steering rate is reconstructed from two consecutive
/// serialized desired angles over their measured publication interval; it is
/// never populated with curvature from the retired five-state formulation.
struct SerializedPreviousInputRequest
{
  double acceleration_mps2{};
  double virtual_progress_speed_mps{};
  double previous_steering_rad{};
  double published_steering_rad{};
  double publication_interval_sec{};
};

std::optional<Eigen::Matrix<double, model::kInputDimension, 1>>
resolve_serialized_previous_input(
  const SerializedPreviousInputRequest & request) noexcept;

/// Exact desired-steering envelope for the cumulative steering-rate input
/// sequence.  This is independent of the physical steering state equality:
/// actuator lag may make the publication predecessor differ from x0.delta.
struct SteeringRatePrefixBounds
{
  double minimum_cumulative_delta_rad{};
  double maximum_cumulative_delta_rad{};
};

/// One affine physical-wall segment per predicted state.  The rows couple the
/// optimized lateral state to the optimized progress state instead of binding
/// the vehicle to a wall interval sampled at a different nominal distance.
struct ProgressAlignedWallConstraints
{
  std::vector<double> lower_slope;
  std::vector<double> lower_intercept;
  std::vector<double> upper_slope;
  std::vector<double> upper_intercept;
};

/// One physical-wall row at an interior point of a state transition. The
/// destination_ratio interpolates lateral state i -> i+1 and therefore closes
/// the former endpoint-only wall contract.
struct SweptLateralWallConstraint
{
  int transition_stage{-1};
  double destination_ratio{};
  double lower_m{};
  double upper_m{};
};

/// One convex branch of the stage-wise dynamic-obstacle disjunction.  The
/// tactical layer chooses a pass side, while the rate-resolved execution layer
/// chooses, for each stage, whether it can already occupy that side or must
/// remain longitudinally behind the opponent.  Keeping this as a separate row
/// prevents a nominal-time opponent sample from becoming an unconditional
/// lateral state box.
enum class DynamicObstacleConstraintAxis
{
  Lateral,
  EffectiveProgress,
};

struct DynamicObstacleConstraint
{
  int state_stage{-1};
  DynamicObstacleConstraintAxis axis{DynamicObstacleConstraintAxis::Lateral};
  double lower{-std::numeric_limits<double>::infinity()};
  double upper{std::numeric_limits<double>::infinity()};
};

struct AssemblyRequest
{
  int horizon_steps{};
  Eigen::Matrix<double, model::kStateDimension, 1> initial_state{
    Eigen::Matrix<double, model::kStateDimension, 1>::Zero()};
  std::vector<model::Linearization> linearizations;
  Eigen::VectorXd state_reference;
  Eigen::VectorXd state_lower;
  Eigen::VectorXd state_upper;
  Eigen::VectorXd state_weight;
  Eigen::VectorXd input_reference;
  Eigen::VectorXd input_lower;
  Eigen::VectorXd input_upper;
  Eigen::VectorXd input_weight;
  Eigen::VectorXd additional_linear_cost;
  Eigen::Matrix<double, model::kInputDimension, 1> previous_input{
    Eigen::Matrix<double, model::kInputDimension, 1>::Zero()};
  Eigen::Matrix<double, model::kInputDimension, 1> input_delta_weight{
    Eigen::Matrix<double, model::kInputDimension, 1>::Zero()};
  std::optional<SteeringRatePrefixBounds> steering_rate_prefix_bounds;
  std::optional<ProgressAlignedWallConstraints>
    progress_aligned_wall_constraints;
  std::vector<SweptLateralWallConstraint> swept_lateral_wall_constraints;
  std::vector<DynamicObstacleConstraint> dynamic_obstacle_constraints;
};

struct Problem
{
  Eigen::VectorXd linear_cost;
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;
  Eigen::SparseMatrix<double> quadratic_cost;
  Eigen::SparseMatrix<double> constraints;
  persistent_osqp::VariableCoordinateScaling variable_scaling;
  int horizon_steps{};
};

std::optional<Problem> assemble(const AssemblyRequest & request) noexcept;

enum class RowKind
{
  Invalid,
  DynamicsEquality,
  StateBox,
  InputBox,
  SteeringRatePrefix,
  ProgressAlignedWallLower,
  ProgressAlignedWallUpper,
  SweptLateralWall,
  DynamicObstacleLateral,
  DynamicObstacleEffectiveProgress,
};

struct RowSemantic
{
  bool valid{false};
  RowKind kind{RowKind::Invalid};
  int stage{-1};
  int element{-1};
};

RowSemantic decode_row(
  int row, int horizon_steps, bool steering_rate_prefix_active = true,
  bool progress_aligned_wall_active = false,
  int swept_lateral_wall_count = 0,
  const std::vector<DynamicObstacleConstraint> * dynamic_obstacle_constraints =
  nullptr) noexcept;
const char * row_kind_name(RowKind kind) noexcept;

/// Exact stage-zero interval implied for one input by the input box and every
/// stage-one state row which depends only on that input.  This is a
/// formulation diagnostic, not a projection or a fallback: an empty interval
/// proves that the QP producer supplied mutually inconsistent hard bounds.
struct FirstStageInputFeasibility
{
  bool evaluated{false};
  bool separable{false};
  bool feasible{false};
  int input_element{-1};
  double declared_lower{};
  double declared_upper{};
  double implied_lower{};
  double implied_upper{};
  int limiting_lower_state_element{-1};
  int limiting_upper_state_element{-1};
};

FirstStageInputFeasibility analyze_first_stage_input_feasibility(
  const AssemblyRequest & request, int input_element) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_problem

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PROBLEM_HPP_
