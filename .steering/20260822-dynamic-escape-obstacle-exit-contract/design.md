# Design

## 根本原因

現行の `WallPathAdmissionGate` は、公開しようとする軌道が物理壁条件を満たすかだけを扱う。
Dynamic Escape 終了後の RacingLine が壁に安全なら、前方対象が残っていても二周期で
handoff を解放する。この判定は壁契約としては正しいが、動的障害物回避の終了契約として
不十分である。

また、実行中の Dynamic Escape 解が壁判定で拒否された周期は
`dynamic_obstacle_lateral_escape_active == true` のため、直前の物理検証済み解を復元しない。
その周期から横操舵を単純保持し、次周期に RacingLine / Follow へ戻るため、別側再計画へ
渡すための横方向の連続性を失っていた。

## 方針

### 1. DynamicEscapeExitGate

壁判定とは独立した小さな状態機械を `overtake_execution_orchestrator` に追加する。

- 開始: Dynamic Escape の終了edge、または実行解の壁棄却
- 保持: 前方障害物が未解消で、安全な置換解も未採用
- 解放: 前方障害物が解消し、出力軌道が壁条件を満たす
- 引継ぎ: 新しい Dynamic Escape 解が壁条件を満たす
- override: Recovery が実行権を取得

このgateは「何を出力するか」を決めず、横方向の継続が必要か、再計画が必要か、終了を
許可できるかだけを返す。

### 2. 物理検証済み解の原子的引継ぎ

Dynamic Escape の壁handoffが保持中なら、現在の新解がactiveかどうかに関係なく、
直前の物理検証済み解を最大 `0.35 s` 復元する。新解は壁判定の連続確認を通過してから
retained解へ昇格し、旧解と入れ替える。

保持解が期限切れの場合は古い操舵を復元せず、通常の安全速度制御を維持しながら再計画を
待つ。これにより古い横軌道の無期限使用を禁止する。

### 3. 前方障害物の未解消判定

前方車が追い越し開始範囲内にあり、次のいずれかなら未解消とする。

- 正のclosing speedが残る
- 安全距離付近まで接近している
- SafetyBrake / EmergencyBrake が有効

対象IDだけでなく、その直後に別車両が最前方へ入った場合も未解消として扱う。

### 4. ログ

状態変化時に `Dynamic escape exit contract:` を一行出力する。

- decision / event / reason
- latched target / observed target
- front distance / protected distance / closing speed
- wall admitted / replacement active / retained available
- hold / replan / release

通常周期の反復は出力せず、状態変化と一定間隔の継続だけを出す。

## 非対象

- 壁・相手車両の制約値変更
- Dynamic Escape 候補生成アルゴリズムの全面変更
- MPC / MPCC のモデルや重み変更
- Recovery 戦略の変更
