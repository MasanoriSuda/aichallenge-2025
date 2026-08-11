# Requirements

## 目的

20260811-133743 の走行で確認した、追い越し開始後の自己失速、横へ流れた状態からの壁接触、遠方停止車に対する候補全滅を局所修正する。

## 対象事象

1. `pass_forward_completion_latched=1` 後も、0.5 秒の tactical revalidation 終了時に full-speed forward escape が解除され、前車速度へ戻る。
2. Pass/Return の横到達性判定が現在の `e_y` だけを使い、外向きの `e_psi` による横速度を無視するため、実車体が壁方向へ流れていても経路を成立扱いする。
3. 高速で遠方の停止車を検出しても ShiftOut 候補が最大 5 m のため、全候補が横加速度上限で棄却される。

## 制約

- 壁接触、EmergencyBrake、solver recovery、V2X continuity loss は引き続き hard fault とする。
- 横加速度上限を緩和して候補を通さない。必要な ShiftOut 距離を延ばして物理成立させる。
- ROS 2 topic/service、提出インターフェース、評価基盤は変更しない。
- 変更範囲は `multi_purpose_mpc_ros` と本ステアリングに限定する。

## 完了条件

- forward-completion ラッチ済みかつ物理hard faultなしなら、rear-clearまたは既存の絶対Mission上限まで full-speed escapeを維持する。
- 横到達性が現在の横速度を含み、Pass再計画とReturnの双方で同じ判定を使う。
- 最大ShiftOut距離を設定可能にし、停止車が遠方にいる場合は長い物理成立候補も評価する。
- core単体テストと `make autoware-build` が成功する。
