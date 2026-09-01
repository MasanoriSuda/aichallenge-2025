# Design

## Runtime form

The participant package remains NumPy-only.  A new runtime model mirrors the
frozen raw TinyLidarNet, frozen production spatial adapter, projected-conv5
normalization, one-layer GRU and direct correction head.  PyTorch GRU gate
ordering (`reset`, `update`, `new`) and its split input/hidden biases are
implemented explicitly and parity-tested over an ordered sequence.

The recurrent artifact is self-contained for reproducibility.  At load time
the core compares its embedded raw base against the production TinyLidarNet and
its embedded spatial baseline against the already-loaded production v11
adapter.  The shadow is rejected if either differs.

The first closed-loop implementation recomputed the identical Conv5 backbone
up to four times per scan (raw production, spatial production, recurrent
features and recurrent embedded baseline).  Although the command value stayed
unchanged, average callback inference rose from about 7.4 ms to 12.8 ms and 18
wheel-speed snapshots became stale.  The canonical production backbone now
owns one Conv5 tensor per scan.  The production head, spatial head and
recurrent correction head consume that tensor after exact embedded-parameter
identity checks at load time.  This preserves authority and arithmetic while
preventing diagnostic shadow work from perturbing production timing.

## State lifecycle

Hidden state starts at zero and advances only after a finite inference with a
fresh wheel-speed sample.  It is reset when:

- wheel speed is missing or stale;
- LiDAR watchdog declares the stream stale;
- recurrent inference raises or produces a non-finite value.

A reset counter and current hidden norm are diagnostic only.  The recurrent
correction never reaches the publisher in this slice.

## Artifact conversion

The converter accepts only the admitted architecture family:

- frozen TinyLidar adapter;
- projected conv5 with fixed train statistics;
- complete frozen production spatial baseline;
- direct correction head;
- no raw pressure-token duplication.

It strict-loads the PyTorch state, emits the exact NumPy parameter dictionary,
and writes a JSON manifest containing model configuration and SHA-256 hashes.

## Launch boundary

Optional checkpoint path and expected SHA-256 flow through the existing launch
layers.  Empty path means disabled.  Supplying a path never grants authority;
there is intentionally no recurrent-authority parameter in this slice.

## Closed-loop evidence

The admitted artifact is:

- NumPy SHA-256:
  `8fe1bceb90fbbc09115fe91cd65e14e31472016b0777a42e9d7b585baef1aca6`
- run: `output/20260902-e2e-recurrent-shadow-shared`
- race: 3/3 laps, zero penalties, total 280.930 s
- recurrent coverage: 6542/6542 scans
- recurrent skipped/error/reset: 0/0/0
- scan frequency minimum: 19.72 Hz
- weighted inference average: 8.005 ms
- minimum reported inference capacity: 84.54 Hz

Both production competition admission and recurrent shadow admission passed.
The recurrent model remains shadow-only; these results do not authorize it to
publish steering.
