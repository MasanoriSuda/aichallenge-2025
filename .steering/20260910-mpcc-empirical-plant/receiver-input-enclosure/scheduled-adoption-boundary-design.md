# Scheduled adoption evidence boundary

Baseline48b6623b. This slice supplies necessary context and atomic evidence APIs;
normal dispatch is still the old synchronous path. No current-evidence result
or diagnostic scheduled certificate grants permission to publish.

## Context capture and revocation

ContextOwner is a single control-thread owner. It captures opaque shared identity
with an atomic active flag. invalidate() revokes all copies without allocating;
a later capture creates a distinct generation, even when changed values returned
to their original state. Destruction revokes the generation. Workers receive only
ContextSnapshot, not a mutable owner. Empty or revoked generations cannot pass.

The scheduled factory retains its input source_context in NominalProof. This token
MUST come from actual SOLVER input capture, travel with that solved plan and survive
worker/physical/Store paths. Attaching today's token to a previously solved plan
would violate the producer contract. This source binding is not yet implemented
in the node. Numerical-only diagnostic Requests may omit it and cannot pass the
new context gate. No token is serialized as reusable live authority.

Context check preserves source/current model, intent, mission, target, execution
side, dynamic obstacle identity/side, horizon, formulation and all schema IDs.
Current decisions and observation generations change under separate evidence
checks. Newly adopted sources also require exact current stage geometry. Already
published remainder uses its immutable source window under current physical
recheck. A published remainder cannot change bounds/cost policy: the generation
must still bind the complete immutable policy/reference/session context.

The main producer must invalidate BEFORE every actual policy, reference, bounds,
session or clock reset, including partial mutation failure. Audit includes runtime
v_max/Q/R/QN/ay/filter/preview settings, current speed-window updates, v_ref/speed
profile, trajectory replacement and external path constraints/border cells.
Unchanged setters must not invalidate every40Hz callback. Mission/target semantics
also require current comparison. These live hooks remain C5integration work.

## One current evidence input

check_current_evidence takes one retained Request and its own original component
observation. It binds the current problem decision/intent and target/dynamic-world
generations to that Request. It then checks components, the complete actual ledger
prefix, context, and original remaining body/corners under the current world.
Context revocation is checked again after physical work. A caller cannot request
the published-remainder rule with a mode flag: it is derived only from the count
of suffix sends which first passed actual ledger authentication.

This is a necessary evidence result, not a normal authority object. Actual next
packet/index, raw before/after clock, steering predecessor duration, final context
revocation and single-writer adoption/commit remain obligations at dispatch.

## Native evidence and remaining work

- R266:118/118pass (657ms). Generation revocation/restoration/destruction, separate
  sessions, empty diagnostic context and thirteen semantic mutations covered.
  One missing-default-field compiler warning was corrected with an explicit empty
  default; no behavior or acceptance change.
- R267:118/119pass. New composite test correctly rejects a synthetic post-send
  history whose last-issued physical steering was left at its pre-send value.
  This is a test input reconstruction defect, not a reason to weaken measurement.
- R268reconstructs the fresh point prefix and all dependent request fields after
  the actual new suffix send. 119/119pass (667ms), no compiler warnings.
- Buildr76passes26packages/29.2s; testsr65passes2509records/64groups with
  0errors/failures/skips. Latest prior runtime
  is dev2-r44at48b6623b's source: deadline failed, Follow not covered.

See [current evidence](scheduled-current-evidence-design.md),
[context/dispatch obligations](scheduled-context-dispatch-design.md), and
[tasklist](../tasklist.md). All M4–M6and C5promotion remain incomplete.

[46sealed files](scheduled-adoption-boundary-evidence.json).
