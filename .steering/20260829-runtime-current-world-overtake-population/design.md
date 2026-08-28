# Design: runtime current-world Overtake population

Introduce one normal-worker dispatcher around the existing evaluation
functions:

- Follow -> current Follow escape population;
- ShiftOut/Pass/Return -> same-side bounded current-world Overtake population;
- Track/Cruise/Rejoin -> unchanged direct canonical pipeline.

The Overtake branch requires the physical snapshot. Missing physical evidence
is a typed build rejection; it must not fall through to retained Mission
geometry.

The candidate population result appends candidate source/count to the existing
solver detail so the current decision log remains traceable without a second
authority or telemetry channel.

This Slice deliberately keeps the bounded population unchanged. The separate
late-schedule failure is candidate-generation evidence, but combining it here
would prevent attribution of the runtime lifecycle replacement.
