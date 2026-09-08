# Prior same-run motion, limited attribution

Source `output/20260909-side-peer-stop-dev2-r1/d2/rosbag2_autoware`, source time
11.1–11.9 seconds. Extraction is `d2-published-stop-motion.json` in that run.
No stream from a different run is joined.

Two actual control commands precede Emergency:

| Source stamp | Receipt wall time | Speed command | Acceleration | Wire steering |
|---|---|---|---|---|
|11.344999746|1788895054.4825482|1.606865525 m/s|-2.959595919 m/s²|-0.202996477 rad|
|11.359999746|1788895054.5110185|1.606865525 m/s|-2.959595919 m/s²|-0.191888392 rad|
|11.394999745|1788895054.5390987|0 m/s|-3 m/s²|-0.165638372 rad|

These align with Stop adoption at decision 976 and Emergency at decision 978;
the second Stop publication was present even though its full decision log was
throttled. This verifies two moving Stop command periods, not continuous safe
stopping through rest.

Velocity-status samples still accelerate from 1.591986 m/s at 11.339999746 to
1.655811 m/s at 11.444999744, then decline to 1.537976 m/s at 11.514999742.
Localization acceleration remains positive longer because it is another filtered
signal. Actual response, source stamp, receipt and the declared 0.13-second
control prediction origin must be compared explicitly; these samples alone do
not prove an incorrect delay constant or justify changing it.

The missing executed Stop 442 prevents comparing its exact certified suffix and
current-world reproof. Complete that tuple before attributing wall-reserve loss
to motion prediction, continuation rebasing or terminal successor representation.
