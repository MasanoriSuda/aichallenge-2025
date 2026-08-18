# Requirements

## Goal

Overtake 開始直後に集中する MPC callback の計算超過を減らし、通常の追い越し中の再計画頻度と攻撃性を維持する。

## Evidence

- 比較対象: `output/20260818-094456` と `output/20260818-103217`
- Stage 1 で callback 超過率は約 5.23% から約 0.90% へ低下した。
- 残った 160 回の超過のうち、少なくとも 103 回は 2 回の Overtake 開始直後へ集中した。
- RTI-SQP の第 2 solve は 3351 cycle 中 3275 cycle で実行され、条件付き抑制が開始直後の負荷に十分効いていない。

## Constraints

- 通常の ShiftOut / Pass / Return の receding-horizon 更新頻度を下げない。
- 第 1 feasible QP は常に維持し、負荷抑制で制御出力を欠落させない。
- hard wall、target、emergency guard を緩和しない。
- 既存のユーザー変更 (`steering_tire_angle_gain_var: 1.5`) と結果 JSON を変更・コミットしない。

## Definition of Done

- pre-arm 中に検証済み Mission の物理壁 envelope を少量ずつ予熱できる。
- Mission 開始直後または当該 cycle に物理壁 cache miss がある場合、RTI 第 2 solve を省略し第 1 feasible 解を採用する。
- 抑制理由を telemetry で区別できる。
- MPCC unit test、overtake core test、Autoware build が通る。
