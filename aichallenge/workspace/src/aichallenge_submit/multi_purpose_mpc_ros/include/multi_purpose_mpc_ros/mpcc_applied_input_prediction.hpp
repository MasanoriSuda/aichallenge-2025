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
struct PublishedInputProgram {
  double publication_interval_sec{};
  std::vector<PublishedCommand> commands;
  bool repeat_last_until_rest{};
};

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

struct AppliedInputSample {
  double begin_sec{};
  double end_sec{};
  /// Native integration duration in the publication-relative clock. Absolute
  /// timestamp subtraction must not invent another mechanical tire substep.
  double duration_sec{};
  AppliedInputBounds inputs;
  BodyRanges swept_body;
  BodyRanges endpoint_body;
};

struct AppliedInputTube {
  /// Binds profile, model, complete original provenance and common program.
  std::uint64_t context_fingerprint{};
  ObservationProvenance observation;
  PublishedInputProgram program;
  InputApplicationProfile profile;
  State coordinate_origin;
  BodyRanges publication_body;
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

} // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
