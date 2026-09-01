# Design

The first from-scratch pressure-GRU result was invalidated by a derived scan-unit
defect.  The adapter remains a bounded follow-up architecture, but it may be
evaluated only after the corrected physical-metre dataset reruns the simpler
direct policy.

```text
750-beam LiDAR
  -> frozen admitted TinyLidarNet conv/fc feature (10 dimensions)
  -> base steering (unchanged)

feature + synchronized speed
  -> unidirectional GRU
  -> bounded zero-initial correction

candidate steering = clamp(base steering + correction)
```

The composed model is a single learned lateral policy, but the experiment stays
offline.  Zero initialization makes the first candidate exactly the current
production policy and prevents random recurrent output from becoming a false
regression baseline.

The first comparison freezes the complete base, trains only the speed embedding,
GRU and correction decoder, and uses sample-proportional temporal chunks.  It
does not add a handcrafted gate or a runtime fallback.

If the compressed 10-dimensional base feature cannot separate corrective
states, one final comparison appends the 750 learnable physical-range pressure
tokens to the same GRU.  The zero correction still preserves base output
exactly.  This separates missing obstacle detail from base-policy retention
without changing labels or admission gates.
