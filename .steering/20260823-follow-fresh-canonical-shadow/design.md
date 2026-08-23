# Follow fresh canonical shadow design

## Earliest missing invariant

FollowのQP solve成功はcanonical execution成功と同義ではない。production昇格に必要なのは、
同じ正規化済みprimalから、物理壁証明済みのplanと最初のactuationを再構成し、authority
contractがfresh candidateとして受理することである。

## Pipeline

```text
Follow typed contract
  -> five-state solve
  -> normalized execution primal
  -> Follow longitudinal hard-gap certificate
  -> actuation proposal
  -> pose trajectory + swept wall certificate
  -> CertifiedMpccSolution
  -> CanonicalExecutionPlan
  -> exact current cursor
  -> fresh candidate
  -> fresh canonical authority
  -> stored actuation == direct primal
  -> CanonicalNormalCommand (shadow only)
```

## Target certificate boundary

Followはcoherent front observationから作ったstage-wise target progressに対し、全stateで
`solved_progress <= target_progress - hard_gap`を検査する。現Sliceではこの縦hard-gap証明を
target obstacle certificateとする。横方向へ交わすOvertakeのfootprint証明には流用しない。

## Authority boundary

生成したcommandはtelemetryにのみ保持し、`MpcControlCycleResult`やpublisherへ渡さない。
既存legacy Followが実制御を所有し続ける。production昇格は動的evidence取得後の別Sliceとする。

## Complexity control

retained plan storeは追加しない。fresh周期の完全なchainだけを観測し、古いtarget observationを
再利用する経路を作らない。

plan extraction以降のfresh-only chainはcontrollerへ複製せず、
`canonical_execution_plan_adapter::build_fresh_canonical_command()`へ集約する。
この関数はplan、cursor、candidate、authority、actuation、commandの各境界をfail closedで評価し、
最初に不成立となった段階をtyped reasonで返す。Follow controller側は物理証明とtelemetryだけを
所有し、canonical identityの組み立てを再実装しない。
