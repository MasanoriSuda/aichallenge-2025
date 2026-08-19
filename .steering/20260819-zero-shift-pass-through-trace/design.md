# Design

## 1. Validated pass-through authority

GapPlanner の候補を reachable bridge へ通した後、次を満たすことを controller から
authority resolver へ明示する。

- planner active / feasible
- planner が lateral bounds を所有
- bridge evaluated / feasible
- bridge の検証sampleが1点以上

要求shiftが通常の最小値以上なら従来の `Accepted` を使う。最小値未満の場合は上記の
検証済みフラグがあるときだけ `AcceptedPassThrough` とし、planner boundsとtargetを
MPCへ残す。ゼロでない逆方向shiftは引き続き拒否する。

pass-through は lateral authority の許可であり、速度制限の即時解除ではない。
最初の valid tracking solution が対象ID・sideへ紐づいた次周期からのみFollow capを
解除する。これにより、solverで実行確認できない候補へ前進authorityを渡さない。

## 2. Runtime failover trace

自由文 `trigger` は固定カテゴリへ分類して `trigger_gate` として出力する。
change-aware signatureは raw trigger/reason を含めず、episode/generation/phase、
固定gate、候補状態、action、sourceで構成する。

Mission差し替え結果は次の経路で記録する。

- dynamic wait resolver による current/alternate replacement
- MPCC-lite same-side replacement
- MPCC-lite cross-side replacement
- opponent-side replacement

差し替え後に `replace-*-applied` または `replacement-rejected` を記録し、`source` で
実行経路を区別する。通常走行時の差し替えログを増やさないため、dynamic Mission
wait がactiveだった場合だけ追加結果traceを出す。

## 3. Scope

今回、wall margin、gap width、最小shift設定値、MPCC horizonは変更しない。
Pass入口とruntime物理再検証の整合性は `trigger_gate` の集計で次の修正対象を
定量化する。
