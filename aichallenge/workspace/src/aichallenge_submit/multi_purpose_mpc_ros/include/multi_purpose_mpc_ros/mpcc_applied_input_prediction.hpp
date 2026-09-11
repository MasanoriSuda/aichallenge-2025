#pragma once

#include "multi_purpose_mpc_ros/mpcc_vehicle_prediction.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model {

inline constexpr char kAppliedInputSchema[] =
    "source-packet-windows-body-stop-v1";

/// Explicit empirical assumption, separate from mechanical tire response.
/// Validating these fields never establishes an actual transport guarantee.
struct InputApplicationProfile {
  std::string profile_id;
  double acceleration_age_sec{};
  double steering_receipt_age_sec{};
  double steering_mechanical_delay_sec{};
};

/// One common serialized program, independent of which input is applied.
/// Commands start at the observation's publication epoch and follow its fixed
/// publication period. An enabled tail republishes the final nonpositive
/// packet at that same period until all admitted body states reach rest.
struct PublicationNanosecondClock {
  std::int64_t first_ns{};
  std::int64_t interval_ns{};
  std::int64_t maximum_delay_ns{};
};

struct PublishedInputProgram {
  double publication_interval_sec{};
  std::vector<PublishedCommand> commands;
  bool repeat_last_until_rest{};
  /// Each future packet may publish anywhere in its closed nominal-to-latest
  /// window. Actual history keeps exact recorded epochs. Zero is legacy exact
  /// timing; canonical proof uses the existing publisher period as its deadline.
  double maximum_publication_delay_sec{};
  /// Explicit integer grid, when the input seconds uniquely round-trip to ns.
  /// Absence retains the original continuous-double clock and fingerprints.
  std::optional<PublicationNanosecondClock> nanosecond_clock{};
};

/// Conversion is fail-closed outside the uniquely representable range and for
/// non-grid values. It never rounds a continuous timestamp into permission.
std::optional<PublicationNanosecondClock> publication_nanosecond_clock(
  double first_sec, double interval_sec, double maximum_delay_sec) noexcept;
/// The common endpoint used by input coverage, rest proof and final guards.
std::optional<double> publication_epoch(const PublishedInputProgram &program,
  std::size_t index, bool latest = false) noexcept;

/// Bounded input provenance carried by a materialized Stop artifact. This
/// stores no parent plan pointer, so repeated Stop joins cannot retain an
/// unbounded chain of past plans. It is data, not an execution certificate.
struct AppliedProgramProvenance {
  std::uint64_t nominal_solution_id{};
  std::uint64_t nominal_problem_fingerprint{};
  ObservationProvenance observation;
  InputApplicationProfile profile;
  PublishedInputProgram program;
  double proved_rest_sec{};
  // Required throughout a source-horizon programme and every materialized
  // successor. Absence preserves legacy immediate/solved Stop provenance.
  std::optional<double> forward_velocity_ceiling_mps;
};

std::uint64_t applied_program_provenance_fingerprint(
  const AppliedProgramProvenance & provenance, const Parameters & parameters) noexcept;

struct ScalarRange {
  double lower{};
  double upper{};
};

struct AppliedInputBounds {
  /// Overall bounding interval for diagnostics. Propagation uses the union
  /// below, so absent near-zero values cannot enter the moving body branch.
  ScalarRange acceleration_mps2;
  ScalarRange wire_steering_rad;
  /// Negative, exact zero, positive hulls of every admissible actual packet.
  /// An absent sign has no member; channel selection remains independent.
  std::array<std::optional<ScalarRange>, 3> acceleration_sign_groups;
};

/// Same order as State: x/y/yaw/u/vy/yaw-rate/desired/tire. Pose coordinates
/// use the fixed initial body heading; velocities and steering retain their
/// usual COM/body definitions. No observation/model error bound is implied.
using BodyRanges = std::array<ScalarRange, 8>;

/// Publication body (begin == end), then each subsequent swept body in order.
/// Rejecting a sample aborts prediction; accepting does not bypass full rest.
using AppliedInputValidator =
  std::function<bool(const BodyRanges &, double begin_sec, double end_sec)>;

/// Four rigid vertex XY pairs in world axes, relative to the observation XY
/// origin. Unlike BodyRanges, these positions are not in the initial heading
/// frame. Local offsets include the caller's complete physical clearance.
using FootprintRanges = std::array<ScalarRange, 8>;
struct AppliedFootprintValidation {
  FootprintRanges local_offsets;
  std::function<bool(const BodyRanges &, const FootprintRanges &,
                     double begin_sec, double end_sec)> validate;
};

struct AppliedInputSample {
  double begin_sec{};
  double end_sec{};
  /// Native integration duration in the publication-relative clock. Absolute
  /// timestamp subtraction must not invent another mechanical tire substep.
  double duration_sec{};
  AppliedInputBounds inputs;
  BodyRanges swept_body;
  BodyRanges endpoint_body;
  std::optional<FootprintRanges> swept_footprint{};
  std::optional<FootprintRanges> endpoint_footprint{};
};

struct AppliedInputTube {
  /// Binds profile, model, complete original provenance and common program.
  std::uint64_t context_fingerprint{};
  ObservationProvenance observation;
  PublishedInputProgram program;
  InputApplicationProfile profile;
  State coordinate_origin;
  BodyRanges publication_body;
  std::optional<FootprintRanges> publication_footprint{};
  std::vector<AppliedInputSample> source_to_rest;
  double rest_sec{};
  std::size_t maximum_body_partitions{};
};

enum class AppliedInputRejectReason {
  None,
  InvalidModel,
  InvalidObservation,
  InvalidProfile,
  InvalidProgram,
  HistoryUnavailable,
  NumericalFailure,
  StepLimit,
  ValidationRejected,
};

struct AppliedInputPrediction {
  AppliedInputRejectReason reason{AppliedInputRejectReason::InvalidModel};
  std::optional<AppliedInputTube> tube;
};

bool valid(const InputApplicationProfile &profile) noexcept;
bool valid(const PublishedInputProgram &program,
           double publication_sec) noexcept;
/// First-packet timing only; source/decision/float identity and full physical
/// authority remain the caller's responsibility. No grace outside this window.
bool first_publication_time_admitted(const PublishedInputProgram &program,
                                     double actual_publication_sec) noexcept;
/// Raw ROS clocks must not regress from callback entry through publication.
/// A received observation may be ahead of /clock; its nominal epoch remains
/// the causal floor, but it must not hide a reset of the underlying clock.
bool first_publication_bracket_admitted(const PublishedInputProgram &program,
                                        double decision_clock_sec,
                                        double before_clock_sec,
                                        double after_clock_sec) noexcept;
std::uint64_t
applied_input_context_fingerprint(const ObservationProvenance &observation,
                                  const PublishedInputProgram &program,
                                  const InputApplicationProfile &profile,
                                  const Parameters &parameters) noexcept;

/// Enclose the values of all causal packets admissible over a native substep.
/// Equal historical epochs retain every value. Missing history rejects.
std::optional<AppliedInputBounds>
applied_input_bounds(const std::vector<PublishedCommand> &history,
                     const PublishedInputProgram &program,
                     const InputApplicationProfile &profile, double begin_sec,
                     double end_sec) noexcept;

/// Shared numerical model only; wall/peer/state/input/intent checks and final
/// publication still need one immutable physical certificate. An optional
/// result or a context fingerprint alone is never execution authority.
AppliedInputPrediction
predict_applied_inputs_to_rest(const ObservationProvenance &observation,
                               const PublishedInputProgram &program,
                               const InputApplicationProfile &profile,
                               const Parameters &parameters) noexcept;

/// Same numerical domain and clocks; rejection returns no partial tube.
AppliedInputPrediction
predict_applied_inputs_to_rest(const ObservationProvenance &observation,
                               const PublishedInputProgram &program,
                               const InputApplicationProfile &profile,
                               const Parameters &parameters,
                               const AppliedInputValidator &validator) noexcept;

/// Additional rigid-vertex enclosure from the same map and original body
/// population. Both validators, when supplied, must accept every sample.
AppliedInputPrediction
predict_applied_inputs_to_rest(const ObservationProvenance &observation,
                               const PublishedInputProgram &program,
                               const InputApplicationProfile &profile,
                               const Parameters &parameters,
                               const AppliedInputValidator &validator,
                               const AppliedFootprintValidation *footprint) noexcept;

} // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
