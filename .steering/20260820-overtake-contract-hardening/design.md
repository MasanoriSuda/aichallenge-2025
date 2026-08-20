# Design

## 1. Speed window normalization

制御適用直前に `[floor, min(reference, limit)]` を正規化する。有限な上端より floor が高い場合は floor を上端へ縮退し、元の要求値と正規化理由を決定ログへ残す。hard limit より Pass floor を優先しない。

## 2. DynamicWait lateral authority

DynamicMissionWait の前進 prefix が安全条件で不許可でも、壁に対して成立する current-`e_y` horizon は lateral hold として出力する。前進 floor/closing は付与せず、縦方向は Follow/front-risk が所有できる。current-`e_y` horizon 自体が成立しない場合だけ明示的な hard fault として扱う。

## 3. Pass/runtime wall contract

Mission が保持する static/dynamic valid-until と予測 rear-clear を一つの純粋関数で評価する。実行時に必要距離を満たさない場合は、壁 preview が発火する前から authority trace に `wall-contract-shortfall` を載せる。Mission 採用側と runtime 側は同じ判定関数を使用する。

## 4. Decision log aggregation

phase/action/lateral/path/control source の構造変化は即時出力する。通常の longitudinal owner や speed-window category の揺れだけでは毎周期出力せず、抑制回数を heartbeat に集約する。solver fallback、wall stop、failsafe、authority conflict は常に即時出力する。

## 影響範囲

- `overtake_execution_orchestrator.*`: 契約正規化、壁契約、ログ集約の純粋ロジック。
- `mpc_controller_cpp.cpp`: 正規化値の制御適用、DynamicWait hold、実行traceへの契約情報接続。
- unit tests: 境界条件と即時warning/集約の回帰試験。

外部インターフェース変更はない。
