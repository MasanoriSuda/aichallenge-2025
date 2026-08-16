# Requirements

## Purpose

Keep lateral hard-constraint ownership continuous from ShiftOut through Pass,
bounded replan holds and Return.  The 20260817-062752 trial showed that the
new stage corridor improved lap throughput and Pass completion, but the MPC
bounds became inactive during target-bound holds and Return.

## Evidence

- GoKart1 improved from six to seven laps over a comparable run and average
  lap time improved from 68.37 s to 59.21 s.
- Four Pass episodes reached Return and three completed Return -> Idle.
- OSQP failures, control callback overruns and controller solver fallback were
  all zero.
- `Overtake stage corridor MPC constraints: active=0` occurred while
  `rh=1/fallback=1/hold=1` in Pass.
- One acquired pass entered Recovery immediately after Return because the
  static-wall clamp exceeded the lateral-acceleration limit.

## Required behaviour

- Every physically evaluated active Mission horizon publishes at least the
  wall-contracted per-stage lateral bounds.
- A fresh validated opponent corridor may further contract those bounds.
- If opponent separation becomes infeasible during a bounded contact-tolerant
  replan hold, only the opponent contraction may be dropped; wall bounds stay
  hard.
- Return keeps wall bounds until its handoff to Idle.
- Existing actual-contact, unknown-map, emergency and solver guards remain.
- No parameter aggression change is part of this work.

## Acceptance criteria

- ShiftOut, Pass replan holds and Return all expose N finite wall bounds.
- Fresh target-bound horizons are distinguishable from wall-only holds in the
  runtime diagnostic.
- Empty or malformed bounds still fail closed before OSQP.
- Build, focused tests and package regression tests pass.
