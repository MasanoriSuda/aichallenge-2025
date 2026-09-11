#pragma once

#include "multi_purpose_mpc_ros/mpcc_applied_input_prediction.hpp"

#include <deque>
#include <memory>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model {

/// Intended certificate/source, not a claim that post-publication checks passed.
struct PublishedProgramSource {
  std::uint64_t decision_id{};
  std::uint64_t solution_id{};
  std::uint64_t problem_fingerprint{};
  std::uint64_t input_context_fingerprint{};
  std::size_t packet_index{};
};

struct PublicationTransaction {
  std::uint64_t sequence{};
  PublishedCommand nominal;
  PublishedCommand published;
  double before_clock_sec{};
  double after_clock_sec{};
  std::optional<PublishedProgramSource> source;
};

/// Single-writer history owner. Workers consume immutable snapshots only.
/// Recording input provenance never grants normal or Emergency authority.
class PublishedInputLedger {
public:
  class Snapshot {
  public:
    const std::vector<PublishedCommand> &history() const noexcept { return *history_; }
    double last_clock_sec() const noexcept { return last_clock_sec_; }
    std::uint64_t sequence() const noexcept { return sequence_; }

  private:
    friend class PublishedInputLedger;
    Snapshot() = default;
    std::shared_ptr<const char> generation_;
    std::shared_ptr<const std::vector<PublishedCommand>> history_;
    std::uint64_t sequence_{};
    double last_clock_sec_{};
  };

  explicit PublishedInputLedger(std::size_t transaction_capacity);
  PublishedInputLedger(const PublishedInputLedger &) = delete;
  PublishedInputLedger &operator=(const PublishedInputLedger &) = delete;

  const std::vector<PublishedCommand> &history() const noexcept { return history_; }
  bool snapshot_ready() const noexcept;
  std::optional<Snapshot> snapshot() const;
  std::optional<std::vector<PublicationTransaction>> since(const Snapshot &snapshot) const;

  /// Existing history values, causal floor, equal epochs and retention remain
  /// owned by record_serialized_publication(). A raw clock regression breaks
  /// snapshot continuity until explicit reset, even if the floor conceals it.
  bool record(const PublishedCommand &nominal, double before_clock_sec,
              double after_clock_sec, double retain_sec,
              std::optional<PublishedProgramSource> source = {});
  void reset();

  /// Exactly all sends since the snapshot must match this declared prefix.
  /// Source/index and both raw clock endpoints are checked for every packet.
  /// No full-rest, current-world, observation or intent authority is inferred.
  bool matches_prefix(const Snapshot &snapshot, const PublishedInputProgram &program,
                      const std::vector<std::optional<PublishedProgramSource>> &sources) const;

  std::uint64_t sequence() const noexcept { return sequence_; }
  std::size_t transaction_count() const noexcept { return transactions_.size(); }
  std::uint64_t discontinuities() const noexcept { return discontinuities_; }
  const PublicationTransaction *latest_transaction() const noexcept {
    return transactions_.empty() ? nullptr : &transactions_.back();
  }

private:
  void change_generation();
  std::size_t capacity_;
  std::shared_ptr<const char> generation_;
  std::vector<PublishedCommand> history_;
  std::deque<PublicationTransaction> transactions_;
  std::uint64_t sequence_{};
  std::uint64_t discontinuities_{};
  std::optional<double> last_clock_sec_;
  bool clock_consistent_{true};
};

} // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
