# Post-motion stationary loss and source supply

2026-09-13, standard single-r8, controller `ec8e822b`, build122/package109.
Original DLL/model/settings,120s host cap. One extra fixed final failure pair
captures1052 after actual moving normal1001. Normal1051 is the last send;
no later normal publication, Start or lap. No claim of full M4–M6 acceptance.

R587 validates both recorded motion witnesses against the actual publication
log.4229callbacks max19.291101ms,508normal sends, no publication-window violation.
R588 public speed peaks1.148950815m/s at9.519999787s. Original outputs and user
JSON/DLL restored, runtime stopped. Bag receipt and controller receipt remain
separate. Public frequency window7.9..14.3s, not a whole-race acceptance window.

R589 reproduces exact source/domain/history/prior publication and final1052.
DispatchReason5 is RestExpired; the CurrentCheck fields are default/unexecuted
because the dispatch returns before current-evidence checking. These values
must not be misread as a physical wall/context rejection. Original rest14.299999690,
next nominal14.289999826 and deadline14.314999826: the send window exceeds
already certified rest. Safety rejection is correct. Extending its deadline,
repeating a hold indefinitely or relaxing the certificate is not a repair.

The upstream supply gap remains: Store stays at652 while fresh source work
continues but does not supply new certification. New programmes cannot bind a
prospective packet from that expired artifact. Latest logs show wall-refined
QP rejection; this does not prove all intermediate/later failures identical.
Do not call the completed stop/rejection itself the root defect.

Before changing production, R590 compares the complete frozen source430
(wall-refinement numerical rejection at9.394999790) and622 (outer physical wall
rejection at11.794999736) with the existing A/B/C/D/G harness, same world/state/
model/reference/hard constraints. Source622's failed physical comparison is
separate from later certified652 and current rest1052. Cold reassembly is not
exact failed QP/warm-start replay. Unsupported target-free arms remain
inconclusive. Use the earliest supported source failure and explicit native
candidate/proof provenance before choosing a producer repair. No new unchanged
live run or parameter scan is justified by expiry alone.

## Results and remaining detection gap

R590A repeats430 numerical wall-refinement rejection and622 physical stage99
wall contact. The latter reports physical endpoint progress30.789m versus reference36.317m
(delta-5.528m); that output is not evidence of current1052 or a
completed restart. Existing B/C/D/G require a dynamic target and are unavailable
here. No alternative was promoted and no infeasibility certificate was obtained.
Both sources use reference-steering/rest-launch initialization; later accepted
652 and failed fresh sources after final1052 are distinct solver inputs.

The new final capture worked. Its upstream solver failure is still hidden by
older intent/stage/outcome slots. Capture the first fresh source failure after
this actual post-motion final loss, retaining each of the two native initializer
owners, exact QP/warm start or full outer physical-proof source. Propagate an
immutable diagnostic final-loss witness through async clones, outside all proof
fingerprints; preserve original buckets. This is a newly isolated missing
boundary, not authorization to retry the same physics/model run unchanged.
[Complete evidence and run/diagnostic hashes](post-motion-source-evidence.json).
