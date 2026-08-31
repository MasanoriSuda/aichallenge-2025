# Dynamic evidence

## Invalid MPC peer teacher

- Run: `output/20260901-033329`
- World: 3 controllable vehicles, domain 2 low-speed peer
- Domain 1: caught domain 2, entered ShiftOut, then
  `actual footprint wall margin violated` -> Emergency Stop/Recovery -> stalled
- Domain 2: ShiftOut -> Recovery because no valid current-side Pass prefix
- Domain 3: continuation rejection retained Stop
- Admission: rejected for all domains; no dataset extraction

## Runtime NPC student baseline

- Run: `output/20260901-034202/d1`
- Controller: promoted TinyLidarNet, steering ML + fixed `+0.6 m/s2`
- Distance before/while failing: 203.77 m
- Maximum speed: 3.58 m/s
- Longest low-speed interval after motion: 70.56 s
- Longest positive-acceleration stall: 70.56 s
- Front LiDAR minimum across run: 4.74 m
- Admission: fail

The model did not intentionally stop for a close frontal obstacle. It continued to command positive
acceleration after becoming physically constrained.

## Known clean single-vehicle control

- Run: `output/20260901-031218/d1`
- Distance: 1003.02 m
- Maximum speed: 4.44 m/s
- Longest positive-acceleration stall: 0.0 s
- Admission: pass

This control proves that the stall detector separates the known completed single-vehicle baseline from
the runtime NPC failure.
