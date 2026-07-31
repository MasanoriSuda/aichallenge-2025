# Tasklist

- [x] 最新ログとrosbagの速度指令・実速度を照合
- [x] 要件・設計を記録
- [x] locked targetの短時間横位置予測を追加
- [x] 直接Pass復帰条件を強化
- [x] 再開goalの内向き移動を抑止
- [x] 条件未成立時にFollowPrepareを保持
- [x] 単体テストを追加・更新
- [x] `make autoware-build`を実行
- [x] packageテストを実行（717 tests / failure 0）
- [x] `git diff --check`を実行
- [x] 動的効果確認項目を記録

## 静的確認結果

- `make autoware-build`: 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  717 tests, 0 errors, 0 failures, 0 skipped
- `git diff --check`: 成功
- `make dev2`: 未実施（試走による動的効果確認は別途）

## 試走判定項目

- `mode=same-side-pass-resume`時に`goal_ey`が対象車側へ戻らないこと。
- `ego_speed < target_speed`の間は`FollowPrepare -> Pass`が発生しないこと。
- 予測横離隔不足時は同側`FollowPrepare`を維持すること。
- 直接復帰後に`body_clear=0`へ戻る回数を`20260801-072353`と比較すること。
- `committed pass longitudinal progress stalled`とSafetyBrake pauseの回数を比較すること。
- 完遂数と接触・壁Recoveryを同時に確認し、完遂率だけで採否を決めないこと。
