# Design

## Observed load

In `output/20260817-010031`:

- D1 worker compute sampled about 74.5 ms on average.
- D2 worker compute sampled about 86.2 ms on average.
- Worker-active callback overrun ratios were about 46% and 58% respectively.
- Worker failures were zero and result adoption was 87.6% / 97.4%.

The worker architecture is therefore functioning, but fixed 10 Hz evaluation
duplicates too much tactical planning work on the available CPU budget.

## Policy

The configured evaluation interval becomes the normal/base interval. For the
asynchronous worker only, calculate:

```text
compute_budget_interval = last_compute_seconds / target_worker_utilization
effective_interval = clamp(
  max(base_interval, compute_budget_interval),
  base_interval,
  maximum_interval)
```

The initial settings are:

- base interval: 0.20 s (5 Hz)
- target worker utilization: 0.35 of one CPU
- maximum interval: 0.30 s (3.3 Hz)

At 75 ms compute the effective interval is about 0.214 s. At 100 ms it is
about 0.286 s. A larger spike is capped at 0.30 s. With the existing 0.50 s
result-age guard, this leaves time for a bounded compute while preventing the
10 Hz worker from continuously occupying a core.

Until the first result is available the base interval is used. Synchronous
shadow mode keeps the configured base interval and does not use this policy.

## Diagnostics

`Overtake MPCC-lite async` adds `interval=<effective> s`. Existing submitted,
replaced, completed, adopted, discarded, snapshot, compute and age counters
remain unchanged.

## Acceptance

The next `make dev2` trial must show:

- `async=enabled` and `load_shedding=enabled` at startup.
- effective worker interval within 0.20--0.30 s.
- no worker failures and bounded pending/replaced jobs.
- materially lower worker-active callback overrun than the 46% / 58% trial.
- no large loss of adopted tactical results.
