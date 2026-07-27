# Results

## 実装結果

- 新規追い越しでは、左右の車体余裕、corridor幅、連続open距離、
  ShiftOut必要横加速度を同じ品質式で比較する。
- 品質差が`0.25`以下なら従来の幾何preferred sideをtie breakにし、
  内側であることだけでは選択しない。
- ShiftOut中も前半`0.60 m / 5.0 m`以内は左右を再評価する。
- 候補が`0.25 s`継続した場合だけ、現在位置から反対側へ一度だけ再計画する。
- 後半、再計画後、または反対側不成立で選択側競合が継続した場合は、
  相手を横切る直接反転を行わずRecoveryへ移す。
- actual/static wall、横加速度、solver、Emergencyのhard guardは維持した。
- `v2x_prediction_use_course_lateral_velocity`は`false`のまま変更していない。
- ROS topic/service、Domain、評価成果物の契約は変更していない。

## 設定

`config/config.yaml`へ以下を追加した。

- `v2x_overtake_line_side_quality_selection_enabled: true`
- `v2x_overtake_line_side_quality_min_score_advantage: 0.25`
- `v2x_overtake_line_early_side_replan_enabled: true`
- `v2x_overtake_line_early_side_replan_max_lateral_progress: 0.60`
- `v2x_overtake_line_early_side_replan_max_traveled_distance: 5.0`
- `v2x_overtake_line_early_side_replan_stable_sec: 0.25`
- `v2x_overtake_line_side_replan_target_guard_distance: 8.0`

既存の速度、壁margin、車間距離は変更していない。

## 自動検証

- `make autoware-build`
  - 25 packages finished
  - build successful
  - 既存のsetuptools deprecation warningのみ
- Docker内で
  `colcon test --packages-select multi_purpose_mpc_ros`
  - 24/24 test suites passed
  - `colcon test-result --verbose`: 684 tests、0 errors、0 failures、0 skipped
  - 対象外の古い`build/joycon_contract_guard/package.xml`が存在しない旨の
    集計warningは出たが、対象packageの結果は成功
- `git diff --check`
  - 成功

## `make dev2`確認項目

1. 起動ログで`side_quality=1/adv=0.25`、
   `early_replan=1/lat=0.60 m/dist=5.00 m/stable=0.25 s/target=8.00 m`を確認する。
2. `V2X debug`の`left_q`と`right_q`を見て、両側成立時に低品質側を
   内側という理由だけで選んでいないことを確認する。
3. ShiftOut前半で相手が選択側へ移動した場合、
   `side_conflict=1`と同じ`replan_candidate`が0.25秒継続した後に
   `OvertakeLine early side replan: side=X -> Y`が一度だけ出ることを確認する。
4. ShiftOut後半の競合では左右へ直接振らず、
   `reason=selected pass side became occupied`でRecoveryへ入ることを確認する。
5. 同一区間で左右の往復、壁接触、solver failure連発が減り、
   `ShiftOut -> Pass -> Return -> Idle`まで完了するか確認する。

実走効果は未確認であり、ユーザーの`make dev2`結果で採否を判断する。
