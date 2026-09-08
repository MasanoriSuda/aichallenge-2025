# Design

The existing terminal feasibility check admits one hardcoded zero-lateral
path-tracking law. The immutable solved normal trajectory can legitimately
use a different offset to satisfy the same wall model; immediately steering
toward zero under maximum braking is not guaranteed feasible.

Use a bounded set of two reference hypotheses with one common evaluator:
first the immutable solved-path profile (if valid), then the existing track
reference. A failure in identity/current-state/actuator/publisher-prefix gates
ends evaluation; only terminal-contingency failure warrants the second
reference. Full Stop and current-world proof remain necessary for either.
The result owns one selected trajectory and actuation trace; there is no
additional command authority, retry timer, grace interval or retained clock.

The strict profile domain is preserved. A stationary or short normal horizon
must not be silently extrapolated; the already-defined track reference still
has full physical course support and is evaluated independently. Unknown
physical feasibility stays rejection if both references fail.

Record which Stop reference was certified in runtime telemetry. Remove the
temporary counterfactual call from the controller when production selection
is installed; the standalone historical comparison remains reproducible.
Keep a diagnostics-only observation API only if needed by regression tests;
otherwise remove it with its tests after equivalent production coverage.

Failing evidence before promotion: frozen11046 zero-reference rejected and
normal-path accepted; live9401 same-plan/current-world comparison accepted.
Validation: preserve baseline short-profile and world-identity rejection;
test bounded reference selection and exact Stop trace binding, whole package,
full build, six-lap single repeats, then dev2/Stop publication acceptance.
Do not mark MPCC complete until the parent campaign is accepted.

Local verification: StopCertificateTracksTheSolvedNormalGeometry fails before
promotion: the generated Stop ends at steering-0.071895rad rather than the
solved-profile+0.074767rad. Both use the synthetic vehicle's explicitly declared
-10m/s2 bound so its0.4m fixture covers the complete stop; production remains-3.
After promotion all64 retained tests pass. A first iteration selected the
second reference by the outer reason and broke three existing feedback-join
tests because that layer preserves SteeringUnreachable even for terminal
failure. Selection now uses terminal_stop_attempted && !terminal_stop_certified,
so identity/actuator/prefix failures cannot reach the second reference. All
existing assertions are retained. Runtime accounting sums both hypotheses.
