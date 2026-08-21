# Tasklist

- [x] 最新試走の参照証明と実行予測の不整合を特定する
- [x] 変更範囲と受入条件を文書化する
- [x] 実solver解の物理壁検証を共通化する
- [x] 未走行entry専用の安全なロールバックを実装する
- [x] 決定ログへ実行解証明とfailover actionを追加する
- [x] 単体テストを追加する
- [x] package build/testを実行する
- [x] 動的確認項目を記録する
- [x] 変更をコミットする

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 32/32 targets成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  1451 tests、0 errors、0 failures、0 skipped

## 次回試走の確認項目

- `Overtake executed solution wall contract`の`result=accepted/rejected`を確認する。
- rejected時に`action=entry-rollback`となり、同じMissionの危険解が
  `Overtake control decision`でpublishされないことを確認する。
- 未走行entry拒否後に`ShiftOut -> FollowPrepare`へ滞留せず、Idleから反対側を含む
  fresh searchへ戻ることを確認する。
- 旧事象の`planner reserve >= required`かつ`actual predicted physical < required`が
  final wall admissionまで到達しないことを確認する。
- solver failure counter、bounded continuation、Recoveryがこの拒否だけでは増えないことを
  確認する。
