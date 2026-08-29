#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_NORMAL_BRANCH_BANK_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_NORMAL_BRANCH_BANK_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include <cstdint>
#include <memory>
#include <mutex>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_normal_branch_bank
{

namespace certified = mpcc_rate_resolved_certified_plan;
namespace shadow = mpcc_rate_resolved_shadow;
namespace artifact = mpcc_rate_resolved_execution_artifact;

enum class ReplaceReason
{
  Accepted,
  InvalidSource,
  InvalidNegativePlan,
  InvalidPositivePlan,
  StaleSource,
};

const char * to_string(ReplaceReason reason) noexcept;

/// One atomic, observation-epoch-local set of physically certified normal
/// avoidance branches.  This is evidence only: it has no execution ledger,
/// command or publisher surface.
struct Snapshot
{
  artifact::Identity source_identity;
  std::shared_ptr<const certified::CertifiedPlan> negative_plan;
  std::shared_ptr<const certified::CertifiedPlan> positive_plan;

  bool available() const noexcept
  {
    return source_identity.sequence > 0U &&
           (negative_plan != nullptr || positive_plan != nullptr);
  }

  std::shared_ptr<const certified::CertifiedPlan> plan_for_side(
    int side_sign) const noexcept;
};

struct State
{
  std::uint64_t latest_source_sequence{};
  std::uint64_t accepted_count{};
  std::uint64_t invalid_source_count{};
  std::uint64_t invalid_plan_count{};
  std::uint64_t stale_source_count{};
  bool negative_available{false};
  bool positive_available{false};
  ReplaceReason last_reason{ReplaceReason::InvalidSource};
};

/// Atomic same-epoch branch evidence. A newer source replaces both pointers,
/// including an empty set, so candidates from different worlds cannot mix.
class Bank
{
public:
  Bank() = default;
  Bank(const Bank &) = delete;
  Bank & operator=(const Bank &) = delete;

  ReplaceReason replace(
    const shadow::Snapshot & source,
    std::shared_ptr<const certified::CertifiedPlan> negative_plan,
    std::shared_ptr<const certified::CertifiedPlan> positive_plan);
  Snapshot snapshot() const;
  State state() const;
  void clear();

private:
  mutable std::mutex mutex_;
  Snapshot snapshot_;
  std::uint64_t latest_source_sequence_{};
  std::uint64_t accepted_count_{};
  std::uint64_t invalid_source_count_{};
  std::uint64_t invalid_plan_count_{};
  std::uint64_t stale_source_count_{};
  ReplaceReason last_reason_{ReplaceReason::InvalidSource};
};

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_normal_branch_bank

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_NORMAL_BRANCH_BANK_HPP_
