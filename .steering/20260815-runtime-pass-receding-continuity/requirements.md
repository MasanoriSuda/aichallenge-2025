# Requirements

## 背景

`output/20260815-183839` では ShiftOut 入口の物理ゲート改善後も、Pass 中に
rear-clear までの実時間予測が更新されないまま SafeSeparation の局所予算を
消費し、最終的に 40 m の絶対 Pass 上限で FollowPrepare へ落ちる事象が残る。

特に progressive entry では初期 `predicted_rear_clear_pass_m` が `inf` のため、
SafeSeparation 開始時の予算が 12 m の既定値に固定される。一方、走行中には
kinematic rollout が有限の rear-clear 距離・時間を得ている。

## 要求

1. Pass 中の有限な rear-clear rollout を SafeSeparation の残予算へ反映する。
2. 予算は増加方向だけに更新し、Mission 全体の 10 s / 40 m 上限を越えない。
3. runtime rollout の completion reserve は entry-time と同じ意味・値を使う。
4. 現在側で絶対予算内に rear-clear できないと判明した場合、停止・Follow 遷移
   より先に既存の左右候補再評価を起動する。
5. 既存の last-feasible prefix を再計画中も実行し、soft failure だけで Pass の
   速度 ownership を失わない。
6. 壁接触、壁計測不能、EmergencyBrake、solver recovery などの hard fault は
   従来どおり緩和しない。
7. ROS topic、message、launch、評価結果 schema は変更しない。

## Definition of Done

- pure helper の境界値テストが通る。
- `multi_purpose_mpc_ros` がビルド・テストできる。
- runtime reserve と entry-time reserve が一致する。
- SafeSeparation の更新ログで runtime 距離・時間と絶対上限を確認できる。
- ユーザー所有の `aichallenge/result-summary.json` をコミットしない。
