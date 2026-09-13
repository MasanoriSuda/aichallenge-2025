# Recovery preference must preserve the remaining candidate population

2026-09-13, baseline1e59065e/controller7f263cc2, single-r28/D1/2906.
R818 exactly reproduces the four actual trials. All are Forward; the three
selection candidates fail original physical/course guards. The policy header
defines prefer_forward_course_escape as ordering only. Its adapter uses an
exclusive if/else chain and never evaluates the remaining population after
preferred failure. ManeuverDirectionUnknown and SafeStop correctly contain this
missing candidate; deleting the safety stop would mask the producer defect.

R819 holds the original full map, pose, body, course and existing five steering
magnitudes. The configured4m reverse diagnostic selects+0.15rad with original
physical/course checks;3.9995/4.0005m agree, existing8m selects+0.10rad. The
unattempted original reverse distance is not captured. These are explicitly
bounded Recovery comparisons, not an exact reverse-query replay or a normal
nine-state A/B/C/D certification. The earlier normal-source stop remains open.

Extract preferred-versus-remaining selection without changing branch contents.
First reproduce the exclusive behavior in a failing real-map regression, then
let preferred failure reach the existing remaining selector. Preferred success
keeps its original identity and skips the remaining work. Disabled preference
keeps the original branch selection. Preserve committed/contact/stepwise/forced
reverse handling, original distances, steering ranking, physical/course/V2X
proofs, gear and current-world revalidation. No new primitive, retry, fallback
authority, solver budget, margin, deadline or configuration change.

Use the exact2906 cell crop (outside Unknown) and crosscheck the native full-map
comparison. Test failed preferred candidates followed by successful existing
reverse, preferred success without remaining calls, disabled preference, and
all-physical-rejected absence. Preserve r22's committed/contact/guide negatives.
Run native/source tests, build142/package129 and fixed-commit single-r29.
Observe actual selection/gear/commands/normal Rejoin and whole callback/true
publication timing. A native positive never authorizes a raw Recovery command.

Removed path: preferred failure that excludes the remaining candidate branches.
Remaining authorities: one certified normal plus existing Emergency and Recovery.
Rollback: eventual ordering commit. Existing first-wall-interval observation is
retained only through this newly changed Recovery scene, then capture/deletion
review. Previous actual late publications and all M4–M6 remain unclosed.

R821 old2fail/other71pass; R822native73 and finalsource118 pass. R823 full-map
old3Forward rejects, current same3rejects then11Reverse selects+0.15rad/4m
and+0.10rad/8m. Final build142all26/package129all2664pass. Forward and remaining
branch bodies are unchanged; identical forward-selector result reused only in
this query, separate contact/stepwise evaluations retained. No cross-tick cache.
Fixed-commit single-r29 next. [Validation](recovery-preference-order-validation.json).
