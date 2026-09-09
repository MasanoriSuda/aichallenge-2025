# Review boundary

Baseline9bca3af6, MPCC production source still1c4f377e. All additions in this
slice are diagnostic source, reproducible evidence manifests and documentation.
No normal authority, wire limits, gain, delay, solver setting or safety tolerance
changed. Existing userJSON/crash/bag/output/install artifacts are preserved.

The baseline body comparison's Rigidbody/base_link origin error is corrected
explicitly in the original slice and integration spec. Raw historical results
are retained. Current pose conclusions use the corrected base_link comparison.
No earlier root-origin result is relabeled as base_link validation.

Prediction inputs are declared: public samples must be both received and sourced
before the anchor; prescribed future commands are exogenous evaluation controls,
not history-only information. Private body data score predictions. Private
contact/applied input are used only for component replay. Only single16..35s
physical/contact averages establish original coefficients. Rejected online/fit/
steering/cached-contact variants have separate reports and revisit conditions.

The native Drive component has a Drive-only precondition. The recorded datasets
contain20147/20147gear=3samples; no Reverse/Park parity is claimed. Sleep requires
all four contacts and the inspected speed/input condition; zero speed alone is
insufficient. Steering replay excludes the declared first0.3sof unknown history.
Future history, invalid input and clock reversal fail the native interface.

The Stop counterexample is deliberately an isolated planar conditional model;
its distances are not AWSIM measurements. The unconstrained no-contact branch
does not prove a reachable road trajectory or physical infeasibility. Actual
recorded latest-value overwrite evidence is preserved in the previous slice.

Validation scope: native strict compilation and assertions; existing20147tick
component replay; input/output JSON and Python AST; local documentation links;
registry validation and git whitespace check. No participant production change,
so no new package build or race acceptance is claimed. Current dev2 remains
rejected; final same-HEAD/intents/dev3/dev4/gates/submission remain unexecuted.
