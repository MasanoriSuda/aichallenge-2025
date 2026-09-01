# Design

The previous sampler assigned each of twelve runs 1/12 probability mass.  A
722-sample failure prefix therefore had the same influence as a 6,370-sample
successful run.  Three-seed direction-only evidence showed material accuracy
falling from 85.29% to 70.32% under that contract.

The new sample mode uses deterministic shuffled epochs over all stored samples.
Direction class weights are computed from aggregate sample counts, while the
existing material weighting keeps sparse left/right corrections visible.
