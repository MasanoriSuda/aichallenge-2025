#include "multi_purpose_mpc_ros/mpcc_publication_ledger.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model {
namespace {
bool finite_nonnegative(double value) { return std::isfinite(value) && value >= 0; }
bool serialized(double value) {
  return std::isfinite(value) && value == static_cast<double>(static_cast<float>(value));
}
bool source_valid(const std::optional<PublishedProgramSource> &source) {
  return !source || (source->decision_id && source->solution_id &&
    source->problem_fingerprint && source->input_context_fingerprint);
}
bool same_source(const std::optional<PublishedProgramSource> &a,
                 const std::optional<PublishedProgramSource> &b) {
  if (a.has_value() != b.has_value()) return false;
  return !a || (a->decision_id == b->decision_id && a->solution_id == b->solution_id &&
    a->problem_fingerprint == b->problem_fingerprint &&
    a->input_context_fingerprint == b->input_context_fingerprint && a->packet_index == b->packet_index);
}
bool same_wire(double a, double b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }
} // namespace

PublishedInputLedger::PublishedInputLedger(std::size_t transaction_capacity)
: capacity_(transaction_capacity), generation_(std::make_shared<const char>(0)) {
  if (!capacity_) throw std::invalid_argument("publication ledger requires positive capacity");
}

void PublishedInputLedger::change_generation() {
  generation_ = std::make_shared<const char>(0);
  sequence_ = 0;
  transactions_.clear();
  if (discontinuities_ != std::numeric_limits<std::uint64_t>::max()) ++discontinuities_;
}

void PublishedInputLedger::reset() {
  change_generation();
  history_.clear();
  last_clock_sec_.reset();
  clock_consistent_ = true;
}

bool PublishedInputLedger::snapshot_ready() const noexcept {
  return clock_consistent_ && last_clock_sec_.has_value() && !history_.empty();
}

std::optional<PublishedInputLedger::Snapshot> PublishedInputLedger::snapshot() const {
  if (!snapshot_ready()) return std::nullopt;
  Snapshot result;
  result.generation_ = generation_;
  result.sequence_ = sequence_;
  result.last_clock_sec_ = *last_clock_sec_;
  result.history_ = std::make_shared<const std::vector<PublishedCommand>>(history_);
  return result;
}

std::optional<std::vector<PublicationTransaction>>
PublishedInputLedger::since(const Snapshot &snapshot) const {
  if (!snapshot_ready() || snapshot.generation_ != generation_ || !snapshot.history_ ||
      snapshot.sequence_ > sequence_ || *last_clock_sec_ < snapshot.last_clock_sec_) return std::nullopt;
  const auto count = sequence_ - snapshot.sequence_;
  if (count > transactions_.size()) return std::nullopt;
  const auto begin = transactions_.end() - static_cast<std::ptrdiff_t>(count);
  if (count && begin->sequence != snapshot.sequence_ + 1) return std::nullopt;
  return std::vector<PublicationTransaction>(begin, transactions_.end());
}

bool PublishedInputLedger::record(const PublishedCommand &nominal, double before,
                                  double after, double retain,
                                  std::optional<PublishedProgramSource> source) {
  if (!finite_nonnegative(nominal.published_sec) || !finite_nonnegative(before) ||
      !finite_nonnegative(after) || !finite_nonnegative(retain) ||
      !serialized(nominal.wire_acceleration_mps2) || !serialized(nominal.wire_steering_rad) ||
      !source_valid(source)) {
    // The caller is recording an already sent packet. Losing that record must
    // invalidate every pending snapshot, even though legacy history is intact.
    change_generation();
    clock_consistent_ = false;
    return false;
  }
  const bool raw_regressed = after < before || (last_clock_sec_ && before < *last_clock_sec_);
  const double published = std::max(nominal.published_sec, after);
  const bool history_regressed = !history_.empty() && published < history_.back().published_sec;
  if (raw_regressed || history_regressed || sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    change_generation();
  }
  if (raw_regressed) clock_consistent_ = false;
  if (!record_serialized_publication(history_, nominal, after, retain)) return false;
  last_clock_sec_ = after;
  transactions_.push_back({++sequence_, nominal, history_.back(), before, after, std::move(source)});
  while (transactions_.size() > capacity_) transactions_.pop_front();
  return true;
}

bool PublishedInputLedger::matches_prefix(const Snapshot &snapshot,
    const PublishedInputProgram &program,
    const std::vector<std::optional<PublishedProgramSource>> &sources) const {
  if (program.commands.empty() || !valid(program, program.commands.front().published_sec) ||
      snapshot.last_clock_sec_ > program.commands.front().published_sec) return false;
  const auto actual = since(snapshot);
  if (!actual || actual->size() != sources.size()) return false;
  double previous_clock = snapshot.last_clock_sec_;
  for (std::size_t i = 0; i < actual->size(); ++i) {
    if (i >= program.commands.size() && !program.repeat_last_until_rest) return false;
    const auto &event = (*actual)[i];
    const auto &packet = program.commands[std::min(i, program.commands.size() - 1)];
    const auto epoch = publication_epoch(program, i);
    if (!epoch || event.nominal.published_sec != *epoch || !source_valid(sources[i]) ||
        !same_source(event.source, sources[i]) ||
        !same_wire(event.published.wire_acceleration_mps2, packet.wire_acceleration_mps2) ||
        !same_wire(event.published.wire_steering_rad, packet.wire_steering_rad) ||
        event.before_clock_sec < previous_clock ||
        !scheduled_publication_bracket_admitted(program, i, snapshot.last_clock_sec_,
          event.before_clock_sec, event.after_clock_sec) ||
        event.published.published_sec != event.after_clock_sec) return false;
    previous_clock = event.after_clock_sec;
  }
  return true;
}

} // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
