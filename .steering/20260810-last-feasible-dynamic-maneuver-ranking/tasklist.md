# Tasklist

- [x] 最新runと既存ranking経路を照合する
- [x] 変更範囲とhard fault境界を定義する
- [x] last-feasible reuse pure policyを実装する
- [x] controller stateへ候補cacheを追加する
- [x] SafeSeparation soft abortへatomic replacementを接続する
- [x] configと起動ログを追加する
- [x] unit testを追加する
- [x] 対象packageをbuild/testする
- [x] 実走確認項目を記載する

## 静的確認

- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R ^test_v2x_overtake_core$`: 成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  933 tests、0 errors、0 failures、0 skipped
- `V2XOvertakeCoreLastFeasibleManeuver`: 5ケース成功

## 実走確認

- `last-feasible ... accepted` の直後にPassを維持またはfresh Missionへ置換すること
- soft abortからFollowPrepare / Recoveryへ落ちる回数が減ること
- alternate candidate失敗だけでは現在Passが停止しないこと
- wall contact / Emergency / solver hard failureは従来どおりRecoveryへ入ること
- Pass -> Return -> Idle完遂率と1.389 m/s固定区間が改善すること
