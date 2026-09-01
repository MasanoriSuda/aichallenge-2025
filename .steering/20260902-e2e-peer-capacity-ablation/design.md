# Design

The naive peer augmentation regressed under the frozen 64-unit GRU.  An
earlier invalid exploratory run used 512 units and improved offline metrics,
but also changed loss weights and other settings.  It therefore provides a
hypothesis, not evidence.

Train one controlled candidate with hidden dimension 512 and every other
effective setting copied from the valid 64-unit peer experiment.  Use the same
strict evaluation reports and 0.02 rad deployment decode.  Do not infer
closed-loop quality from offline admission; an admitted result becomes only a
shadow candidate for a later Slice.
