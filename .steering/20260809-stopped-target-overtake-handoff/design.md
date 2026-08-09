# Design

## 原因

`can_start_low_speed_bypass()` は `committed_overtake_execution_active` を一律に拒否する。
そのため通常 Overtake が停止確認より先に commit すると、停止車用 local path の成立性を
評価する入口自体が閉じられる。

## 方針

1. `LowSpeedBypassCandidateRequest` に「commit 済み実行から安全に引継ぎ可能」を表す
   入力を追加する。
2. commit 済みでも、同一 target、追跡連続、現在車体非重複を満たす場合だけ停止車
   candidate の評価を許可する。
3. 安全な引継ぎ candidate では、既に lateral plan が存在するため最小準備距離を 0 m
   まで緩和する。最大開始距離は既存値を維持する。
4. 停止車 local path は、通常 Overtake が commit した side を強制して評価する。
   同一 side が不成立なら LowSpeedAvoidance へ移行せず、通常 Overtake を保持する。
5. local path が成立した場合のみ `LowSpeedAvoidance` を返す。下流では同周期に direct
   low-speed control を開始し、generic OvertakeLine を reset する。

## 非対象

- 接触後の ContactContinuation
- 通常 Overtake の経路評価・side ranking
- SafetyBrake 閾値、Recovery 距離、stuck recovery パラメータ

## ログ

引継ぎ時の reason に `confirmed stopped target takeover from committed Overtake` を含め、
実走で通常の新規 LowSpeedAvoidance と区別できるようにする。

