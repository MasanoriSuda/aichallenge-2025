# Design

## 因果関係

今回の問題は、`dynamic_obstacle_lateral_escape_active` が
「今周期に fresh candidate が成立した」と「DynamicEscape が現在の実行権を持つ」の
二つの意味を兼ねていたことから始まる。

1. planner/qualification が一周期だけ不成立になる。
2. `dynamic_obstacle_lateral_escape_active` が false になる。
3. authority resolver は Follow/RacingLine を選ぶ。
4. 呼出側は true -> false を DynamicEscape の実行終了と解釈する。
5. wall handoff が、その周期に生成された racing-line prediction を評価する。
6. racing-line の壁不成立を、保持中 DynamicEscape target/side の失敗として記録する。
7. tracking qualification を消去し、side backoff/quarantine を延長する。
8. fresh solution がさらに出にくくなり、Follow/MPCC の往復と加減速が増える。
9. OSQP failure と stuck/recovery はこの往復の下流で増幅される。

問題が見える場所は壁 hold、急減速、solver fallback だが、問題を作る場所は
fresh availability と execution ownership の状態モデルである。

## 仮説の比較

### H1: 壁余裕が厳しすぎる

- 支持: 一部の fresh DynamicEscape prediction に実壁不成立がある。
- 反証: wall replan 31 回中 27 回は DynamicEscape 以外の経路を評価している。
- 結論: 一部の実失敗には関係するが主因ではない。設定変更は行わない。

### H2: OSQP の計算能力・重みが主因

- 支持: max-iteration failure が後半に集中している。
- 反証: authority/path の往復と壁 hold が solver failure より前に発生している。
- 結論: 下流の増幅要因。先に入力問題の不連続を除去する。

### H3: fresh candidate availability と execution ownership の混同

- 支持: 4 attempt 内で action が約 70 回往復し、false 周期でも retained solution は存在する。
- 反証条件: false 周期で retained solution が期限切れまたは identity mismatch なら正しい終了。
- 結論: 主根本原因。確信度: 高。

### H4: 壁判定の経路 provenance 欠落

- 支持: replan 31 回中 27 回が racing-line/safety-hold prediction に基づく。
- 反証条件: 評価経路が DynamicEscape 所有なら side failure の記録は正当。
- 結論: H3 が作る誤終了を backoff ループへ増幅する第二の根本原因。確信度: 高。

## 修正案比較

### A. wall tolerance/backoff 時間を緩和する

症状を減らすだけで、別経路の失敗を誤帰属する構造は残る。不採用。

### B. `dynamic_escape_active == false` の wall replan だけ禁止する

誤 backoff は減るが、authority と速度指令のチャタリングは残る。不採用。

### C. execution lease と prediction ownership を明示する

fresh/retained を一つの実行状態へ解決し、壁失敗を評価経路の所有者だけへ返す。
既存 retained solution、wall safety、exit gate を再利用でき、全面書換えを避けられる。
採用。

## 実装

1. orchestrator に純粋な `resolve_dynamic_escape_execution_lease()` を追加する。
2. V2X behavior output に fresh とは別の effective execution 状態・source・age を持たせる。
3. authority resolver は effective execution を使用する。QP 制約生成は fresh 状態を使い続ける。
4. fresh が欠けても retained identity/lease が有効なら、壁判定前に time-aligned control と
   prediction を復元する。
5. exit edge は effective execution の true -> false でのみ生成する。
6. dynamic wall replan は prediction が DynamicEscape 所有の場合だけ side failure を記録する。
7. decision/wall trace に fresh/effective/source/prediction ownership を残す。

## 安全境界

- SafetyBrake / EmergencyBrake では retained execution を有効にしない。
- attempt、target、既知 side が一致しない retained solution は採用しない。
- 0.35 秒 lease または horizon を超えた解は採用しない。
- retained prediction も既存の物理 footprint wall admission を通す。
- outgoing path が危険なら handoff hold は維持するが、その証拠を DynamicEscape side の
  quarantine には使用しない。

