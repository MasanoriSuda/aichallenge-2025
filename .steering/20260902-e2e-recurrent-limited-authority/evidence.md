# Evidence

## Frozen identities

- raw production SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- spatial production SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- recurrent candidate SHA-256:
  `8fe1bceb90fbbc09115fe91cd65e14e31472016b0777a42e9d7b585baef1aca6`
- recurrent authority bound: `+/-0.24 rad`

The recurrent artifact remains external to the submission package and the
packaged authority default remains `false`.

## Static verification

```text
make autoware-build:             25 packages passed
TinyLidarNet ML suite:           206 passed
TinyLidarNet controller tests:    37 passed
submit launch tests:               5 passed
system launch tests:              10 passed
git diff --check:               passed
```

The controller contract proves that disabled recurrent authority is bitwise
identical to the existing spatial production command.  Enabled authority owns
steering only after an admitted recurrent inference, clips correction to the
explicit experiment bound, and falls back to the current spatial command when
speed is unavailable.  Acceleration authority is unchanged.

## Closed-loop single-vehicle Gate

The first run, `output/20260902-e2e-recurrent-authority-024`, finished all
three laps with zero penalties and no stall:

```text
laps:        101.074 / 90.535 / 88.761 s
total:       280.370 s
coverage:    6637 / 6638
skip/reset:  1 / 1
```

It was rejected rather than waived because one velocity synchronization gap
correctly removed both spatial and recurrent admission for one scan and reset
the recurrent hidden state.

The unchanged repeat
`output/20260902-e2e-recurrent-authority-024-repeat` passed every strict Gate:

```text
laps:                       101.164 / 90.155 / 90.645 s
total:                      281.964 s
penalty / stall:            0 / 0
coverage:                   6238 / 6238 (100%)
skip / error / reset:       0 / 0 / 0
authority application:      6238 / 6238 admitted scans
authority clips:            12
maximum applied correction: 0.240 rad
weighted inference:         8.062 ms
minimum scan rate:          19.94 Hz
```

The preceding frozen shadow run was `280.930 s`.  Across the two authority
runs the mean total was `281.167 s`, a `+0.237 s` difference.  There is no
material single-vehicle speed claim.

## Closed-loop NPC Gate

`output/20260902-e2e-recurrent-authority-024-npc` used one ego and two runtime
NPCs.  It passed the motion, competition and recurrent-authority Gates:

```text
final position:              1 / 3
laps:                        105.017 / 90.175 / 98.670 s
total:                       293.862 s
penalty / stall:             0 / 0
coverage:                    8760 / 8760 (100%)
skip / error / reset:        0 / 0 / 0
authority application:       8760 / 8760 admitted scans
authority clips:             18
maximum applied correction:  0.240 rad
weighted inference:          7.483 ms
minimum scan rate:           19.92 Hz
minimum observed front scan: 1.093 m
```

The admitted spatial-v11 NPC references were `293.473 s` and `293.653 s`.
The recurrent-authority result is `+0.389 s` and `+0.209 s` respectively, so
it demonstrates closed-loop non-regression but not a material performance
gain.

## Decision

The bounded recurrent execution path is accepted as a default-off experiment.
It closes the runtime ownership, fallback, telemetry and Gate requirements and
does not disturb the packaged spatial-v11 production default.

Do not package or enable the recurrent checkpoint by default from this result.
The next promotion decision needs either repeated NPC benefit or a separately
approved objective other than lap-time improvement.  The rejected first run
also remains evidence that recurrent temporal continuity depends on the
existing velocity synchronization boundary; its Gate must not be weakened.
