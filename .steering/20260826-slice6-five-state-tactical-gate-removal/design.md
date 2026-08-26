# Design

## Current causal flow

```text
tactical Mission geometry
  -> prospective six-state left/right solve and physical proof
  -> six-state selection + Mission hint
  -> causal six-state Gate A worker

in parallel:
  -> old five-state left/right solve and physical proof
  -> old branch selection
  -> overtake_selected_mission / preentry plan / retained entry cache
  -> live side-selection suppression or reconstruction
```

The second path is the remaining cross-formulation authority.  Although its
command publisher was deleted, its decision can still suppress or reconstruct
the Mission that is later presented to the six-state Gate.

## Repair

1. Preserve tactical candidate construction and immutable left/right snapshot.
2. Evaluate each valid side only with the prospective six-state pipeline.
3. Convert that typed result to the common branch metrics used by diagnostics
   and homotopy selection.
4. Publish one selected Mission hint without an execution certificate.
5. Let the causal six-state Gate A worker bind the exact current-world
   certificate atomically.
6. Delete the five-state pre-entry plan, entry-plan cache, certificate
   revalidator, and the old fresh evaluator/lifecycle.

This changes ownership, not tactical parameters.  The tactical geometry remains
a soft input; only the proof and branch-selection authority are unified.

## Final ownership

```text
immutable tactical snapshot
  -> left six-state prospective solve + exact physical proof
  -> right six-state prospective solve + exact physical proof
  -> one six-state branch selection
  -> certificate-free Mission hint
  -> causal current-world six-state Gate A
  -> immutable certified execution prefix
```

The compatibility field named `extended_mpcc_branch_selection` mirrors the
six-state selection for existing diagnostics only.  It does not invoke a
second solver, construct a five-state plan, or certify execution.  Exact wall
fingerprints now live with the six-state physical-wall contract rather than a
retired five-state retained-world module.

The old libraries are also removed from the production executable link graph.
Their remaining source files are deliberately handled by the following Slice
6 physical-deletion audit so this ownership change and broad file deletion
remain separately reviewable.
