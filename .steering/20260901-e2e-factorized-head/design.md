# Design

```text
shared frozen-spatial representation
  + activation_head -> P(material)
  + sign_head       -> P(left | material), P(right | material)
  + magnitude_head  -> bounded |delta|

P(left)    = P(material) * P(left | material)
P(neutral) = 1 - P(material)
P(right)   = P(material) * P(right | material)
```

The existing weighted residual, magnitude, three-class direction and anchor
losses consume the composed probabilities.  Normal evidence primarily trains
activation; it no longer has to redefine the conditional left/right boundary.
