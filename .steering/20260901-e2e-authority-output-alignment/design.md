# Design

Train one spatial candidate from the immutable `recurrent_direct_v4_trainonly`
view with `max_abs_delta_rad=0.12`.  This bound is not a new tuning value: it is
the existing runtime authority contract.

The previous candidates optimized a model with plus/minus 1.2 rad support and
then clipped its output by a factor of ten at publication.  The aligned model
optimizes inside the set of commands that can actually be published.  Its
architecture, frozen backbone, projection, speed input, normalization,
sampling and loss weights otherwise remain identical to DAgger v3.

Evaluate against recurrent v4 with the evaluator also set to a 0.12 rad model
support and a 0.12 rad runtime authority bound.  Compare with the persisted v3
bounded report.  Runtime launch and weights remain frozen.
