# Rejoin Feedback and Stepwise Clearance Reassessment Results

実施日: 2026-07-18
判定: Pass（LowSpeedRejoin完了）、Safe negative確認（深い接触）

## 実装結果

- stepwise Reverse中のcompleteなstatic / V2X blockageは、停止・Drive確認後に
  `STOP_AND_REASSESS -> CHECK_CLEARANCE`へ戻るようにした。
- 非stepwise blockageのSafeStopは維持した。
- LowSpeedRejoinへ参照曲率feedforward、横偏差、heading誤差の専用操舵feedbackを追加した。
- rate-limit後の同じtire angleを0.8 m swept-footprint評価と実commandに使用した。
- 通常MPCが接触後に0 m/sを返しても、全hard gate成立中のLowSpeedRejoinでは
  設定値1.0 m/sを専用目標として使用した。0以下の設定は起動時に拒否する。

## 検証

### Build / test

- `make autoware-build`: 25 packages成功。既存のsetuptools deprecation warningのみ。
- `ctest -R ^test_stuck_recovery_core$ --output-on-failure`: 1/1成功。
- `test_stuck_recovery_core --gtest_color=no`: 70 tests / 9 suites、70/70成功。
- package全体の`colcon test-result`には、本変更外の既存`test_path_core` fixture失敗と
  staleな`joycon_contract_guard` package参照が残るため、対象binaryを直接確認した。

### dev3 run

| run | 結果 | 判定 |
|---|---|---|
| `output/20260718-172154` | D2はstepwise blockage後に`STOP_AND_REASSESS`へ戻り、2.005 mでrejoinへ到達。旧速度調停が通常MPCの0 m/sを採用し、`e_y=2.305 -> 2.263 m`のまま5.006秒でtimeout | 再評価遷移Pass、速度調停Fail |
| `output/20260718-173126` | D2は深い接触から10 stepで0.091 mのみ移動し、`escape_step_limit_reached`でSafeStop。D3も不安全候補を実行せず停止 | Safe negative |
| `output/20260718-173448` | AWSIM processは生存したまま`/clock`が停止し、全Domainのodometryがstale化 | 環境異常として除外 |
| `output/20260718-173846` | D2が9 step・2.049 m後退後にrejoin完了 | Pass |

成功runのD2 LowSpeedRejoinは次の通り。

- 開始: `1784364006.763 s`、`e_y=0.787 m`、`e_psi=-0.070 rad`
- 終了: `1784364009.868 s`、`e_y=0.336 m`、`e_psi=-0.043 rad`
- 所要時間: 約3.105秒（timeout 5.0秒以内）
- 専用速度目標: 1.000 m/s
- 観測実速度: 0.000 -> 0.170 -> 0.402 -> 0.549 -> 0.700 -> 0.879 m/s
- 最大操舵角: -0.350 rad
- 全観測sample: `static=none`、`v2x_clear=1`
- `rejoin_complete`後も約38秒間、通常のCruise / Follow遷移と前進を継続し、
  Recovery再発・odometry failsafe再発はなかった。

dev3はrosbagを生成しない構成だったため、解析根拠は各Domainの`autoware.log`である。

## 結論

LowSpeedRejoinの停止原因は操舵符号ではなく、通常MPC速度との`min`調停だった。
専用速度目標と横偏差feedbackを組み合わせることで、既存の速度・時間・誤差閾値を
緩和せずにruntimeでrejoin完了を確認した。深い物理接触では改善不能な場合が残るが、
step / 距離 / static / V2X gateにより前進せずSafeStopできている。

本値は2025 AWSIM `final_ver3`のローカル実験値であり、2026公式値または実車値ではない。
