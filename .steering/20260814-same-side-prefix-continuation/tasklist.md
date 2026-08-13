# Tasklist

- [x] `20260814-065705`のPass失敗経路を特定
- [x] 同側prefixへ実行authorityを付与
- [x] tactical replacement専用時計を追加
- [x] full-track transition admissionを実preflightへ接続
- [x] 単体テスト追加・更新
- [x] `multi_purpose_mpc_ros` build/test
- [x] 次回試走の確認項目を記録

## 検証結果

- `make autoware-build`相当（Docker Compose）: 成功、25 packages
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 targets成功
- `colcon test-result --verbose`: 1103 tests、0 errors、0 failures、0 skipped
- 既存の`build/joycon_contract_guard/package.xml`欠損に関する読取警告は対象外

## 次回試走で見るログ

- `prefix=1/1/admitted`かつ同側候補時に`authority=replace`となること
- `OvertakeLine fresh same-side PassPlan replaced`で`mode=receding-prefix`が出ること
- `same_admit`の`age`が0.02〜0.03秒へ周期的に戻らないこと
- `Pass -> FollowPrepare`の`optimized horizon escaped target separation bounds`が消えること
- `Pass -> Return -> Idle`へ到達すること
- wall contact、SafetyBrake、Recoveryが増えないこと
