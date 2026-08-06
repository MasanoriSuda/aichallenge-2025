# Requirements

## 目的

`output/20260806-111221` で確認した、衝突・壁接触・solver fallbackを伴わない追い越し失速を減らす。

## 対象事象

- 完全な ShiftOut/Pass/Return Mission が成立しているのに、新規 entry の相対速度確認待ちで Follow の front-risk brake が先行する。
- body-clear deadline を満たす検証済み ShiftOut が、同じ locked target の将来交差だけで SafetyBrake に落ちる。
- cap 解放済み Pass が、V2X 1周期相当の車体境界揺れで直ちに Follow/SafetyBrake へ戻る。

## 制約

- `/control/command/control_cmd` など既存 ROS 2 interface は変更しない。
- 実車体重複が継続する場合、壁接触、経路不成立、target discontinuity、solver failure の停止・Recoveryを無効化しない。
- 変更は競技シミュレーション向け attack mode の範囲に限定する。
- `aichallenge/result-summary.json` と `output/` は変更しない。

## 完了条件

- 検証済みMissionは速度確認待ちを経ずに Overtake へ handoff できる。
- 検証済み ShiftOut は current body separation がある間、縦距離だけの SafetyBrake に負けない。
- Passのcurrent-overlap確認は0.30秒継続時にのみ成立する。
- core unit test と package build が通る。
