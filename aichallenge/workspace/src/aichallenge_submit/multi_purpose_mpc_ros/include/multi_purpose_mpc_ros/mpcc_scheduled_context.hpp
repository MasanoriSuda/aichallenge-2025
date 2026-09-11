#pragma once

#include <atomic>
#include <memory>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled {
class ContextOwner;

/// Captured with solver input, before work starts. A live owner invalidates its
/// generation BEFORE policy/reference/session mutations, including partial
/// failures. Shared ownership prevents pointer reuse from reviving old work.
class ContextSnapshot {
public:
  bool valid() const noexcept { return generation_ && generation_->active.load(); }
  bool same_generation(const ContextSnapshot &other) const noexcept {
    return valid() && other.valid() && generation_ == other.generation_;
  }
private:
  friend class ContextOwner;
  struct Generation { mutable std::atomic<bool> active{true}; };
  std::shared_ptr<const Generation> generation_;
};

/// One control-thread owner; workers receive only snapshots. Invalidation never
/// allocates. A later capture creates a new generation even if values reverted.
class ContextOwner {
public:
  ContextOwner() = default;
  ~ContextOwner() { invalidate(); }
  ContextOwner(const ContextOwner &) = delete;
  ContextOwner &operator=(const ContextOwner &) = delete;
  ContextSnapshot capture() {
    if (!current_.valid()) current_.generation_ = std::make_shared<const ContextSnapshot::Generation>();
    return current_;
  }
  void invalidate() noexcept {
    if (current_.generation_) current_.generation_->active.store(false);
    current_.generation_.reset();
  }
private:
  ContextSnapshot current_;
};

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled
