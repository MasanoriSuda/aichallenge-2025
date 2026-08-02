# Design

## 共通kinematic rollout

50 ms刻みで以下を更新する。

1. 制御遅延中は現在速度と現在横位置を保持する。
2. 遅延後は `target speed + closing speed` と経路速度上限の小さい方へ向け、`a_max` / `a_min` で速度を更新する。
3. 自車・対象車の縦位置を積分する。
4. 自車のmission進行距離からShiftOut横位置を解決する。
5. body-clear、hard-distance、必要横加速度を同じ時刻軸で評価する。

経路速度上限は候補horizon上の各waypointについて、`v_ref`、domain `v_max`、曲率横加速度上限から作る。

## candidate順位

順位は以下とする。

1. deadlineを実際に評価済み
2. deadline feasible
3. minimum slackを満たす
4. base racing lineを維持するdirect pass
5. body-clearが早い
6. ShiftOut距離が短い
7. 横移動が小さい
8. 必要横加速度が小さい
9. closing speedが高い

minimum slackは初期値0.25秒とし、設定から変更可能にする。

## global side selection

各sideのcandidate latticeは既存処理を維持する。各sideの最良candidateを同じ全順序比較器へ渡すことで、全candidateを一つのvectorへ入れた場合と同じ最終選択を得る。start-grid、locked-side維持、early-replanの安定時間は既存のFSM規則を維持する。

## mission atomic update

mission freeze処理で以下を同時に更新する。

- fixed goal
- ShiftOut distance
- closing speed
- deadline checked
- deadline feasible
- total path distance

Recovery再取得は、現在周期で選択された同一sideのdeadline feasible candidateがある場合だけこの処理を使用する。

