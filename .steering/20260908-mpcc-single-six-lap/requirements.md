# Single-vehicle six-lap baseline

Run the currently built and tested controller with the existing evaluation
launch and eval.sh (six laps/600s), using the dev image mounted workspace.
This isolates Track/Cruise and course seam from V2X interaction and is not
submission-image acceptance. Keep all production settings unchanged.
The multi-car Pass progress audit continues independently; do not erase its
failure or treat this run as evidence of complete overtake or certified Stop.
