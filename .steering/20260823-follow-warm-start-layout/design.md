# Design

## Typed dual tail layout

Extend the persistent OSQP warm-start contract with a small layout descriptor for each trailing
constraint block:

```text
stage_count
rows_per_stage
```

The existing base layout remains:

```text
dynamics[N+1][nx]
state boxes[N+1][nx]
input boxes[N][nu]
curvature rate[N][1]
```

Declared trailing blocks follow it in builder order and are shifted independently by one stage. The
Follow builder declares exactly one block:

```text
physical effective gap[N+1][1]
```

The shifter validates the complete primal and dual sizes before copying anything. Therefore an unknown
row addition fails closed instead of being silently copied with the wrong temporal alignment.

## Data flow

`build_extended_progress_problem`
→ stores the typed trailing layout beside `P/A/q/l/u`
→ `solve_extended_progress_problem`
→ passes that layout to `shift_mpc_warm_start`
→ persistent OSQP receives a horizon-aligned primal and complete dual.

No production command or canonical authority path changes in this slice.
