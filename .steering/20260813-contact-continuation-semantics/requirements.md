# Requirements

## 目的

ContactContinuationを「接触直前から安全制約を緩める処理」ではなく、既に発生した回復可能な横接触から前進分離する処理へ限定する。

## 実測

`output/20260813-020856/d1`ではContactContinuationが2回発火したが、いずれも`evidence=near`であり、評価のcrashペナルティ時刻と一致した。

- 1回目: `ego_v=5.26 m/s`で開始し、75 ms後に`1.98 m/s`で終了
- 2回目: `ego_v=5.14 m/s`で開始し、50 ms後に`2.38 m/s`で終了
- 終了理由は横速度ではなく、near/overlap証拠が1周期消えたこと
- `vlat_hysteresis=1`は0回

## 要件

1. near-contactはPrearm専用とし、それだけでSafetyBrake抑制、overlap許容、full-speed forward escapeを有効にしない。
2. ContactContinuationの開始には、確認済み実overlapまたはPrearm直後の衝突相当速度急落を要求する。
3. 開始後は短い接触証拠欠落を保持し、V2X/矩形境界の1周期揺れでMissionを解除しない。
4. 相手速度ベクトルから相対ヨーを推定し、並走とみなせる角度内だけ許可する。姿勢推定不能時はfail-closedとする。
5. ContactContinuation開始には前周期の壁余裕確認を要求し、壁接触・壁余裕不足・solver fault・emergencyは従来どおりhard faultとする。
6. 接触中の横目標は相手から離れる方向とし、壁内の実行可能区間へclampする。
7. target ID、side、Mission generationを維持するのは上記の有界ContactContinuation中だけとする。

## 非対象

- 接触を目標にする軌道
- AWSIMのcrashペナルティ自体の解除
- SafeSeparation距離上限の変更
- 壁余裕や車体寸法の縮小

