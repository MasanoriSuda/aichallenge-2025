# Requirements: stage-aligned warm-start transport

> Status: rejected after dynamic A/B. The experiment is preserved only as
> audit evidence; all source and test changes were removed.

## Problem

The certified five-state MPCC warm-start lifecycle is now fail-closed, but the
transport still shifts every accepted solution by exactly one stage.  The
canonical warm-start identity separately proves whether the next spatial stage
geometry is identical or overlaps at an arbitrary offset.  Ignoring that
offset makes the stored primal/dual describe different rows from the current
QP and causes the observed `cold-certified -> warm-rejected` alternation.

## Required invariants

- A warm start is transported only after formulation, schema, horizon and
  spatial stage compatibility have been proved.
- Identical stage geometry uses a zero-stage transport.
- Rolling geometry uses the exact overlap offset proved by the identity
  resolver; it is never hard-coded to one stage.
- Rejected or uncertified solutions remain unable to enter warm history.
- No solver tolerance, physical bound, wall margin, controller fallback,
  retry, feature flag or parameter is changed in this Slice.
- The legacy three-state solver keeps its existing one-stage helper until it
  is removed by the migration plan; this Slice changes only the canonical
  five-state transport.

## Dynamic acceptance

Compared with `output/20260824-014849`, a bounded `make dev2` run must show:

- the stage transport offset in Track/Cruise production outcome telemetry;
- no persistent reuse of a rejected warm artifact;
- a material reduction in warm-only execution-primal rejection, or a new
  typed rejection that falsifies this hypothesis without hiding the failure.

The second condition occurred: the new typed offset showed that most evaluated
cycles used the intended zero-stage alignment, yet bound rejection continued.
