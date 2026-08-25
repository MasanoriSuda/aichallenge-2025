#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PROBLEM_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_PROBLEM_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include "multi_purpose_mpc_ros/persistent_osqp.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_problem
{

namespace model = mpcc_rate_resolved;

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
};

struct RowSemantic
{
  bool valid{false};
  RowKind kind{RowKind::Invalid};
  int stage{-1};
  int element{-1};
};

RowSemantic decode_row(int row, int horizon_steps) noexcept;
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
