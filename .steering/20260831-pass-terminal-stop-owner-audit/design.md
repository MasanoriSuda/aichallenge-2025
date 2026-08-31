# Design: Pass terminal Stop owner audit

The first causal defect is upstream of the Pass/Stop path-owner difference.
At decision 3418 the measured lateral state was `0.943174 m`, while the
progress-aligned stage-zero support ended at `0.911453 m`.  The exact grid
reported no physical contact, but every A/B/C/D arm was rejected before solve
as `initial-state-outside-bounds`.  The current state is immutable and cannot
be repaired by an optimized input, so applying the future planning box to the
same stage-zero equality creates an authority hole rather than a safety proof.

The semantic adapter must therefore own stage zero as the exact measured-state
equality.  Future wall/corridor boxes, transition sweep rows and downstream
exact footprint proof remain unchanged.  The pre-existing Stop producer's
manual stage-zero rebase is deleted so this invariant has one owner.

The later decision 3418 snapshot is already beyond recovery.  Once the common
adapter permits every arm to reach the solver, all persistent, stateless,
rough/lattice and bounded Stop candidates collide with the exact wall proof;
the earliest Stop contact is at stage 2.  This is a downstream state and is not
used to justify a new fallback.

The earlier decision 3396 snapshot is the actionable boundary.  With the same
central stage-zero ownership:

- persistent A and the same-side stateless arm remain rejected;
- opposite-side stateless B is solver-, wall-, dynamic- and terminal-Stop
  certified;
- both production current-world populations contain certified candidates,
  including a same-side mid-physical-diagonal candidate.

This proves that the world was still recoverable before decision 3418.  It also
explains why no additional Mission resume rule is appropriate: production
already owns a dual current-world population and a certified sibling bank.  The
duplicated stage-zero constraint prevented that existing architecture from
materializing a usable Bundle.  Removing the duplicate owner repairs the
producer instead of adding another lifecycle exception.

The audit therefore keeps the immutable world, initial state, maximum-braking
contract and exact wall/dynamic certificates fixed, then evaluates:

1. current persistent candidate and existing terminal Stop;
2. stateless receding candidate with the same terminal policy;
3. bounded rough/lattice candidates;
4. bounded causal seven-state Stop feasibility.

After this repair the same frozen snapshot is replayed.  Any remaining failure
is classified at its next causal stage; it may not be labelled physical
infeasibility without an exact bounded certificate.

The full-test harness exposed an independent test-only deadlock.  If worker
startup was not observed within 100 ms, a fatal assertion destroyed the worker
without releasing its deliberately blocked non-cancelable job.  The test now
waits on an explicit startup notification and releases the job even on timeout;
production worker semantics are unchanged.
