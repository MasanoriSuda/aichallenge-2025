# Design

The same-world divergence audit rejects a direct inference from teacher
projection deficit to failure: clean domains exhibit the same deficit.  The
missing evidence is an executed teacher that succeeds in the actual peer
interaction world.

Add one explicit diagnostic Make target that launches the unchanged final
world and assigns the already implemented `speed_committed_teacher` to all
four domains.  A symmetric all-teacher world is deliberately stringent: if
the policy deadlocks or collides with itself, it is not a valid source for the
dynamic-interaction labels needed by production.

The strict decision is run-level rather than sample-level.  One failed domain
rejects the complete run.  A rejected run remains useful as a counterexample
but cannot become a hard steering target.

The first startup attempt exposed a pre-existing integration defect before the
race began: the packaged 4.6 m/s fixed-mode governor was forwarded into every
teacher mode although the controller contract rejects positive governor
authority there.  The documented diagnostic value `0.0` was also rejected by
the shell validator.  Permit the already supported non-negative value and make
every teacher target explicitly export `0.0`; do not weaken the production cap
or expand teacher authority.
