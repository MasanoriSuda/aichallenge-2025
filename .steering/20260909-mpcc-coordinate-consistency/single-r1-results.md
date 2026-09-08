# Initial coordinate-model dynamic acceptance

Run `20260909-coordinate-single-r1`, Domain1, baseline
`d54822c982c705098cc2fbfab4160f15e41865b0` plus the run's sealed source patch.
The run manifest preserves both binaries and all model/map/calibration inputs.
This precedes the additional geometry-lineage rejection checks.

Six laps252.253662109375s, penalty0. No logged active-race moving Emergency or
Recovery. Moving overrides11287/11288at9.13m/s occur after the controller sees
AWSIM `finish` at wallclock1788884368.450092137. Normal identity joins remain
canonical; startup holds and post-finish stops are retained in `log-summary.json`.
Callback telemetry covers12893reportedcycles,max20.390ms,0overruns. The logs
are throttled state-change/periodic observations, not a complete per-tick ledger.

Control receive10577messages,264.371973s,mean24.997350ms,p9526.359200ms,
p9927.223229ms,max39.862633ms. No nonpositive receive interval or gap above50ms.
Source timestamps separately contain12duplicates and one119.999997ms gap.
These coincide with a pause in the recorded simulator clock at256.509994266s:
the next `/clock` arrives286.641ms later but advances simulation time only5ms.
Kinematic-state receipt also pauses285.135ms and velocity-status317.419ms.
Commands continue at wallclock25ms while their shared simulation time is held.
The later source-clock catch-up is retained; this is not evidence of fresh
simulation observations arriving during a command-only timestamp freeze.
The underlying cause of the simulator/publication pause is not identified.

The bounded initial single-vehicle campaign is closed with one passing run.
This does not establish the final integrated campaign, complete peer geometry,
multivehicle passing, certified multi-tick Stop, async matrix or submission.
Do not erase older rejected trials or imply that the live7405prefix is fully
explained by this run. No margin, solver budget or actuator tuning was changed.

Evidence under `output/20260909-coordinate-single-r1/`: `manifest.json`,
`runner.log`, `d1/autoware.log`, both same-run resultJSONfiles,
`log-summary.json`, `bag-analysis.json`, `source-clock-anomalies.json`,
and the finalized `d1/rosbag2_autoware/`.
The user's root result-summary and safety-gate JSON hashes remain unchanged.
