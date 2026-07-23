# Pass中 no-gap 速度制限修正 設計

## 原因

gap planner が実行corridorを不成立とした場合、`GapPlannerOutput` は
`no_gap_target_velocity` を速度上限として返す。

横離隔確認済みのPassでは、このcorridor不成立を
`committed_execution_corridor_bypass` によりdiagnostic-onlyとして扱い、
横方向の追い越しlineを継続している。しかし後段のno-gap速度制限適用条件は
このbypassを参照しておらず、2.0 m/sのreachable hard limitだけが残っていた。

## 修正方針

no-gap速度制限の適用可否を純粋関数へ切り出し、以下の場合だけ抑止する。

- `committed_execution_corridor_bypass == true`
  - Pass phase
  - locked targetとの横離隔latch済み
  - live corridor不成立を既にdiagnostic-onlyとして扱っている

通常Follow、設定で許可されたFollow no-gap制限、通常Overtake、ShiftOut、
横離隔未確認Pass、Recoveryは従来どおりとする。

## 安全性

- corridor bypassが成立しない状態ではno-gap速度制限を維持する。
- front riskとEmergencyBrakeは別経路のため維持される。
- MPC速度上限、加減速度上限、solver fallbackは変更しない。
- topic/service/message契約に変更はない。

## 効果確認

1. 純粋関数の状態組合せを単体テストする。
2. パッケージtest/buildを実行する。
3. dev2で `diagnostic-only ... active=1` の直後を抽出する。
4. 修正前の `6.40 -> 5.14 -> 3.46 -> 1.84 m/s` と比較し、
   2.0 m/sへの制御収束が消えたか確認する。

## 検証結果

- `make autoware-build`相当のDocker build: 25 packages成功。
- `colcon test --packages-select multi_purpose_mpc_ros`: 600 tests、
  0 errors、0 failures、0 skipped。
- 修正前 `output/20260724-021819/d1/autoware.log`:
  - corridor bypass開始直前の実速度は6.401 m/s。
  - bypass中に5.135、3.463、1.836、1.821 m/sまで低下した。
  - behavior側は`desired_v=11.11 m/s`、`speed_cap=0`、
    `risk=Clear`、`solver_failures=0`であり、plannerの隠れた
    `no_gap_target_velocity=2.0 m/s`が原因だった。
- 修正後 `output/20260724-070818/d1`:
  - corridor bypass開始時の実速度はrosbagで4.484 m/s。
  - 1、2、4、6、8秒後は4.783、5.017、5.670、6.268、
    6.768 m/sとなり、2.0 m/sへの急失速は再現しなかった。
  - bypass後8秒間の指令は速度11.111 m/s、加速度1.000 m/s2を維持した。
  - `solver_failures=0`。Emergency/SafetyBrake、static wall infeasible、
    runtime contactは対象区間で未発生だった。

## 判定と残課題

対象とした「committed Passのdiagnostic-only corridor不成立が残す
2.0 m/s cap」は解消した。通常Follow、未分離Pass、Emergency、Recovery等の
安全経路はコードレビューと単体テストで維持を確認した。

この短時間runでは、横離隔latch前のShiftOut中に実速度が一時3.56 m/sまで
下がった。またPass継続後に`locked target course progress discontinuity`で
Recoveryへ移行したため、正常な`Pass -> Return`完了までは確認していない。
いずれも今回の限定修正の対象外とする。
