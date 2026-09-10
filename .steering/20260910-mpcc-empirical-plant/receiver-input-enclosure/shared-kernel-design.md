# Shared body arithmetic slice

2026-09-11 JST, parent checkpoint `bef879b2`, unchanged controller `cbcda112`.
The autonomous M1–M6 implementation and local commits remain authorised.

The receiver's selected input can differ from the nominal publication history.
The preserved dev2-r15/956 and application-r3/1773 failures establish the lost
Stop reserve; the numerical enclosure supplies a candidate interpretation, not
an executable certificate. The existing A/B/C/D and same-control comparisons are
in the receiver audit and enclosure results. No new solver experiment is needed
to remove the duplicate arithmetic before integration.

This slice extracts wheel forces, COM/base-link kinematics, tire response and
body increment into one internal templated kernel. The public double model and
the diagnostic interval/derivative arithmetic use it. Branch policy supplies
only scalar versus set-valued reverse correction and rolling operations; it
cannot substitute another vehicle equation. Public parameter/state layout,
fingerprint, rest decision, substep schedule and failure checks remain stable.
The native per-wheel operation order is preserved, including moment summation.

Before replacing the producer, freeze the old source and compare the new public
advance/derivative against that independently compiled source on rest/launch,
reverse, steering limits/rates, nonzero COM offsets, varied parameters and actual
timestamp subdivisions. Compare all scalar outputs, rejection, substep count
and rest flags. Rerun the numerical enclosure and original earlier source-world
comparisons because their arithmetic implementation changes. Run the required
build/package checks. The unchanged-control simulator failure is not repaired
by this extraction and must not be retried as an acceptance candidate yet.

Delete the diagnostic wheel/body/tire transcription in the same slice. Keep one
production normal authority. No input-age parameter, wider margin, new hold or
rate rule is introduced. Rollback for this slice is `bef879b2`. Receiver history,
steering receipt, common executable controls, immutable proof/artifact binding,
all-response rest horizon and runtime timing remain the next integration work.

## Validation result

The independent frozen-source comparison passes37,956cases and494,904bitwise
scalar values, including36rejections and1,676rest cases. The initial diagnostic
initializer type error is preserved. `make autoware-build` r33 passes26packages;
package r28 passes2,410records with0errors/failures/skips. The shared-kernel
numerical r9 suite and source-world r3 comparison pass; all four minimum peer
clearances are unchanged. Source-world range/proof costs9.3–14.3ms standalone.
No integrated receiver or runtime acceptance is claimed. Evidence is in
[shared-kernel-evidence.json](shared-kernel-evidence.json).
