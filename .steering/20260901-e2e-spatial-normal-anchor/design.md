# Design

The prior candidate passed all teacher/peer Gates and failed only independent
normal leakage.  This Slice changes the data support, not the model or limits.

```text
teacher train sequences -> successor - base residual
production normal train -> exact zero residual
                         -> sequence-balanced sampler
                         -> unchanged frozen spatial adapter

teacher validation -----\
peer d3 ------------------+-> unchanged offline Gate
production normal val ---/
```

The normal wrapper ignores stored control labels because they are not a new
teacher.  It restores normalized stored scans to physical metres and emits the
same tuple contract with `teacher == base == 0`.  Prefixed sequence identities
make the anchor source explicit in the manifest.
