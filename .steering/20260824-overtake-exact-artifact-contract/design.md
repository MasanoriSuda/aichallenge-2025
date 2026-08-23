# Design

Replace the boolean-only exact trajectory completeness check with a typed
validation result. Keep the boolean wrapper for existing call sites, but make
the five-state wall proof log the exact reason and stage. This Slice begins as
observation-only.

The relevant contracts are deliberately kept distinct:

1. execution-primal normalization uses per-row numerical tolerances;
2. trajectory extraction checks finite state, monotone solved progress and
   lateral bounds using that admitted tolerance;
3. immutable exact-artifact validation checks the stored representation.

If step 2 admits a value which step 3 rejects, the repair must make these
contracts share one canonical representation. It must not be hidden by an
entry lifecycle delay.
