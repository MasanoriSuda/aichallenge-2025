# Evidence

## Observation

The production log originally made row 254 look like the cause of the
post-ShiftOut collapse. For horizon 20 the exact row layout is:

- equality rows: 0--125;
- state-box rows: 126--251;
- input-box row 252: stage-zero acceleration;
- input-box row 253: stage-zero steering rate;
- input-box row 254: stage-zero virtual-progress speed.

The added diagnostic computes the exact one-dimensional first-stage interval
for virtual-progress speed. A synthetic test with fixed next progress and a
strictly positive input lower bound reports an empty interval, proving that the
observer detects the suspected contradiction when it is present.

## Dynamic evidence

Run: `output/20260825-235153`

Observed rejects consistently reported a nonempty interval, for example:

```text
row_semantic=input-box/stage=0/element=1,
first_vtheta=separable/feasible/
declared:[0.0121043,11.0801]/implied:[0.0121043,11.0801]
```

The first significant Domain 1 wall incident has this ordering:

1. `1787669557.639451646`: Track/Cruise solved 81/81 recent QPs, 50
   iterations, and the physical wall pipeline accepted the latest horizon.
2. `1787669557.639635258`: production published the exact six-state Cruise
   command at 8.762 m/s with steering 0.3345 rad.
3. `1787669558.966791396`: the recovery monitor observed
   `wall_distance=0.000`, `e_y=-2.426 m`, actual speed 8.21 m/s and no relevant
   V2X target; canonical normal authority failed closed to Emergency.
4. Only afterward did sustained maximum-iteration outcomes report stage-zero
   input residuals, including row 253/steering rate.

This ordering disproves the claim that the later row-254 residual initiated
this incident.

## Result

- ShiftOut execution-source bridge causation: rejected.
- First virtual-progress interval contradiction: rejected for the observed run.
- Solver/warm-start/clearance tuning: not authorized by evidence.
- Production behavior change: none.
- Next root-cause boundary: accepted wall horizon versus actual executed
  Track/Cruise vehicle response before contact.
