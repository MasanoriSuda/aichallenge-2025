# Requirements: current-world proof runtime audit

## Objective

Identify the first synchronous proof stage which makes a certified
current-world command miss its own 25 ms publication interval.  This Slice is
observation-only: it must not change normal authority, solver settings,
clearance, timeout, lease, fallback or Mission behavior.

## Frozen evidence

- Baseline commit: `5817fee0`.
- Dynamic run: `output/20260831-002556/d1/autoware.log`.
- Decision 1344 accepted Stop-lattice source 1 after 33.152 ms in the retained
  join; the whole callback took 40.043 ms.
- The accepted proof owned one 25 ms publisher interval.
- Decision 1345 arrived 45 ms later and could not rebuild the terminal Stop
  suffix; external Stop followed.
- Candidate and published clocks both resolve source cursor 0.840 s at
  decision 1345, and the command speed is re-anchored to current-world speed.
  Changing the clock or velocity tolerance is therefore not supported by the
  evidence.

## Constraints

- No production authority change.
- No new grace, retry, lease, timeout or fallback.
- No wall, vehicle, dynamic-obstacle or solver parameter change.
- Instrument one existing evaluator rather than adding duplicate timers in
  multiple callers.
- Runtime data must remain diagnostic and must never become an acceptance
  condition in this Slice.

## Definition of done

- The retained evaluator reports non-overlapping runtime regions for state
  join, continuation construction/proof, terminal construction, terminal
  dynamic proof and terminal wall proof.
- The regions and total are visible in the existing retained transition log.
- Unit/source-contract/full tests pass.
- A bounded `make dev2` run identifies the dominant region for at least one
  accepted Stop-lattice bridge or another partial-proof terminal episode.
- The next behavioral Slice is selected from measured evidence.
