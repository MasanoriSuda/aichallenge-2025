# Requirements

## 目的

追い越し中の40 Hz control callbackに残る実行horizon、物理壁走査、RTI-SQPの重複計算を削減し、25 ms deadline超過を減らす。

## 対象

- 同一WP・同一Missionで短時間再利用できるhard-validated receding horizon
- progress-contouring MPCCの条件付きRTI refinement
- static physical wall envelope cacheのworking setと退避方式
- 負荷削減効果を判定できる周期ログ

## 制約

- `/control/command/control_cmd`の40 Hz publish契約を維持する。
- 現在車体の壁接触、EmergencyBrake、odometry/NaN fail-safeは低周期化しない。
- target、Mission generation、phase、side、WPが変化した場合はhorizonを即時再計算する。
- hard wall fault、target jump、course-progress reject、corridor block、solver recoveryではhorizonを再利用しない。
- ユーザー変更中の`steering_tire_angle_gain_var`と`result-summary.json`を変更しない。

## Definition of Done

- receding horizonのfresh solveとbounded reuseをログで区別できる。
- RTIのrefinement試行、条件skip、deadline skipをログで判別できる。
- wall cacheがdeterministic LRUとなり、現行4096件より大きいworking setを保持する。
- pure helperの単体テスト、対象package buildが成功する。
- 実走ではShiftOut/Pass callback、deadline超過、cache hit率、Pass完遂を比較できる。
