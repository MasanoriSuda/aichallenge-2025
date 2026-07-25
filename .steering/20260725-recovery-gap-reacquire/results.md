# Results

## 実装結果

- `v2x_overtake_recovery_reacquire_enabled` を追加し、現行設定では有効化した。
- Recovery の phase hold 経過後、同一 target/side の安全な corridor が再成立すると
  `Recovery -> ShiftOut` へ遷移する。
- solver recovery/cooldown、target discontinuity、rear-clear、EmergencyBrake、追い越し禁止、
  corridor 不成立では fail-closed のままとした。
- Recovery の moving-Follow 速度制御と最大 Recovery 時間は変更していない。

## 検証

```text
make autoware-build
  Build successful

colcon test --packages-select multi_purpose_mpc_ros
  24/24 test targets passed
  649 tests, 0 errors, 0 failures, 0 skipped
```

`colcon test-result --verbose` は古い `build/joycon_contract_guard/package.xml` の欠損を
診断表示したが、対象 package の test result は 0 errors / 0 failures だった。

## 実走で確認するログ

```text
OvertakeLine: Recovery -> ShiftOut
reason=same target gap reacquired during recovery
```

多車両 AWSIM で、再獲得後に同じ静的壁理由へ短周期で戻らないことと、追従距離・速度が
従来の moving-Follow profile を維持することを確認する。
