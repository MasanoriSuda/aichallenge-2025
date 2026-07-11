# V2X Overtake Stability Tasklist

作成日: 2026-07-11  
更新日: 2026-07-11  
状態: Implemented / Runtime Verification Pending

## Definition of Done

- [ ] 安全な直線で、通過可能gapがある前方車に対して安定してOvertakeへ入る。
- [ ] 1〜数周期のV2X分類欠落だけでPassからReturnへ落ちない。
- [ ] Return直後の同一対象再取得を一貫した規則で処理する。
- [ ] 左右gap、curve不許可、policy拒否、実幅不足をログとtestで区別できる。
- [ ] Overtake中は安全上限内のsoft速度参照を持ち、暗黙のFollow 3.0 m/s制限を受けない。
- [ ] Overtake開始時に連続OSQP fallbackを発生させない。
- [ ] SafetyBrake、front risk、curve cap、wall/path constraintを弱めない。
- [ ] D2 start maximumの適用意味が設定と実装で一致する。
- [ ] LowSpeedAvoidance、gate2、通常trajectory trackingを回帰させない。
- [ ] `make autoware-build`、対象package test、dev2/dev3検証が完了する。
- [ ] `docs/spec/mpc-integration.md`とpackage READMEが実装に一致する。

## Phase 0: 調査・要求確定

- [x] `git status --short`で開始時の既存変更を確認する。
- [x] 現行config、C++実装、関連steeringを確認する。
- [x] `output/latest/d1..d3/autoware.log`のV2X state/phase/reasonを確認する。
- [x] D1/D3がOvertakeへ入らず、D2だけがOvertakeへ入った事実を確認する。
- [x] curve判定で`plan_N=0`となる主要ケースを確認する。
- [x] D2のShiftOut直後OSQP失敗と速度低下を確認する。
- [x] `Overtake -> Cruise -> Overtake`と`Pass -> Return`の不整合を確認する。
- [x] gap 1.8 m、guard 0.8 m、片側探索、multi-front policyを確認する。
- [x] domain 10/20/40 km/hとD2 start 37 km/hの`min()`問題を確認する。
- [x] start期間がAWSIM StartではなくMPC初期化時から計時され、D2で走行前に失効することを確認する。
- [x] trajectory CSV速度がruntime未接続であることを確認する。
- [x] `requirements.md`、`design.md`、`tasklist.md`を作成する。
- [ ] 利用者レビューで対象範囲と優先順位を確定する。

## Phase 1: Baseline Evidence

- [ ] 現行commit、config hash、使用trajectory、ROS_DOMAIN_IDを記録する。
- [ ] dev2 controlled straight overtakeを再実行する。
- [ ] dev3 multi-front / side-by-sideシナリオを再実行する。
- [ ] rosbag記録を有効にし、control、odometry、V2Xを保存する。
- [ ] Overtake進入前後の実速度、速度指令、加速度、操舵、solver状態を時系列化する。
- [ ] curve forbidden率、gap候補幅、phase遷移回数をbaselineとして記録する。
- [ ] 原因別の再現シナリオを最小化する。

## Phase 2: Test Seam / Core Extraction

- [x] V2X overtake判定をpure C++でtest可能な境界へ抽出する。
- [ ] `StableOpponent`相当の入力modelを追加する。
- [x] 左右候補のfeasible結果と拒否reasonを追加する。
- [x] Start/global/domain速度調停結果をpure coreへ追加する。
- [x] CMakeへC++ unit testを登録する。
- [x] config省略・disabled互換fixtureを追加する。
- [ ] 現行動作をgolden testとして固定する。

## Phase 3: Target Detection / FSM Stability

- [x] target vehicle IDとpass sideのlockを追加する。
- [ ] front/side/rear region hysteresisを追加する。
- [x] 短時間欠落を保持する`target_hold_sec`を追加する。
- [x] rear clearanceとclear継続時間でPass完了を判定する。
- [x] `Overtake -> Cruise -> Overtake`相当の短時間欠落testを追加する。
- [x] Return直後の同一target再取得規則を実装する。
- [x] Return後半または別sideへの危険な即再開を拒否する。
- [x] stale/jump/emergencyでlockを安全に解除する。
- [x] phase遷移reasonを全分岐で出す。

## Phase 4: Both-side Gap / Curve Policy

- [x] left/right候補を同じ条件で評価する。
- [ ] raw幅、post-wall-margin幅、target clearanceを分離する。
- [x] `gap_min_width`とOvertake guard幅の適用順序を実装上分離する。
- [x] 開始前だけ反対側へfallbackできるようにする。
- [x] ShiftOut以降のside lockを維持する。
- [x] vehicle-vehicle拒否とmulti-front拒否をreasonにする。
- [x] gap candidateとcurve execution permissionを分離する。
- [x] hard forbidden、soft entry forbidden、continue条件を分離する。
- [ ] 曲率禁止率をtrajectoryごとに計測する。
- [ ] 最初のヘアピンを保護したまま調整候補を作る。

## Phase 5: Overtake Velocity Arbitration

- [x] Overtake soft desired velocityの計算を追加する。
- [x] entry speedをsoft desiredへ反映する。
- [x] front speed + advantageをconfig化する。
- [x] global/domain/ref/curve/front-risk/phase上限との優先順位を実装する。
- [ ] Overtake参照をhard lower boundへ使わないtestを追加する。
- [x] Recovery専用速度上限をFollow速度から分離する。
- [ ] Overtake終了後の参照速度を連続的に戻す。
- [x] desired/hard limitをdebug出力する。
- [x] trajectory CSV `vx/ax`が本変更の入力でないことを維持する。

## Phase 6: Domain Start Maximum

- [x] global hard maximumをconfig parse後も保持する。
- [x] normal domain maximumとstart maximumを分離する。
- [x] start期間のepochをAWSIM `Start`のrace-session遷移へ変更する。
- [x] Readyでduration以上待機してもStart後にoverrideが有効となるtestを追加する。
- [x] reset後の新sessionでepochを再初期化するtestを追加する。
- [x] start期間中はglobal以下でstart maximumを適用する。
- [x] 期間終了後にnormal domain maximumへ戻す。
- [ ] D1/D2/D3、unknown domain、mapping未定義をunit testする。
- [x] 起動ログと周期debugにeffective maximumのsourceを表示する。

## Phase 7: MPC Preflight / Solver Recovery

- [x] Overtake target反映後のfinite、bounds、target、速度/曲率参照をpreflightする。
- [x] phase遷移時の最初のtarget変化を制限する。
- [ ] steering rateと前回horizonの初期整合を確認する。
- [x] preflight失敗時はOSQPへ投入せずfallbackする。
- [x] solver連続失敗の開始・継続・復帰を記録する。
- [x] Overtake由来の連続失敗時に同じtargetを再投入しない。
- [x] deceleration fallbackとSafetyBrakeを維持する。
- [ ] ShiftOut開始のsynthetic infeasible testを追加する。

## Phase 8: Build / Automated Verification

- [ ] formatter / lintを実行する。
- [x] `make autoware-build`を実行する。
- [ ] `colcon test --packages-select multi_purpose_mpc_ros`を実行する。
- [ ] `colcon test-result --verbose`を確認する。
- [x] existing Boost、trajectory、V2X tracker testの回帰を確認する（既存path fixture 1件は別記）。
- [x] config key未指定時に起動できることをbuildとdefault fixtureで確認する。
- [x] topic/message/launch契約差分がないことを確認する。

## Phase 9: Simulator Verification

### dev2 controlled overtake

- [ ] 直線・前方1台・左gapでShiftOut/Pass/Returnを完了する。
- [ ] 直線・前方1台・右gapでShiftOut/Pass/Returnを完了する。
- [ ] 第一候補閉塞時に開始前だけ反対側を選ぶ。
- [ ] front短時間欠落でPassを維持する。
- [ ] Overtake区間にOSQP fallbackがないことを確認する。
- [ ] 速度制限元と実速度低下を説明できるログを保存する。

### dev3 multi-vehicle

- [ ] 前方2台policy拒否を識別する。
- [ ] vehicle-vehicle gap拒否を識別する。
- [ ] side-by-side時のtarget lockとReturnを確認する。
- [ ] target IDが途中で入れ替わらないことを確認する。
- [ ] 全車両のdomain速度上限とstart maximumを確認する。

### safety regression

- [ ] 最初のヘアピンで内側追い越しを開始しない。
- [ ] wall/path constraint接触がない。
- [ ] front risk emergencyでSafetyBrakeが優先される。
- [ ] `make gate2`の停止車回避を回帰させない。
- [ ] gate3相当の車線維持を回帰させない。
- [ ] `/control/command/control_cmd`の周期とdeadlineを確認する。

## Phase 10: Documentation / Rollout

- [x] `docs/spec/mpc-integration.md`へ新しい判定・速度調停を反映する。
- [x] package READMEへconfigとdebugの読み方を追加する。
- [x] CSV速度がruntime未接続である制限を維持する。
- [ ] 実行したbuild/test/devコマンドと結果を本ファイルへ記録する。
- [ ] 新規flagの既定値をruntime証跡に基づいて決める。
- [ ] 既存steeringのruntime pending項目へ本作業への参照を追加する。
- [ ] 2026未確定仕様をTBDとして残す。

## 実装時の停止条件

- V2X IDが安定せずtarget lockの安全性を確保できない。
- controlled dev2でboundsまたはwall clearanceを説明できない。
- Overtake speed参照によりOSQP失敗または接触が増える。
- curve設定を緩和しないと通過できず、最初のヘアピン安全性と両立しない。
- 公式2026 interfaceが現行V2X契約と異なることが判明する。

停止条件に入った場合は、安全処理を迂回せず、原因・ログ・必要な追加仕様を記録して利用者へ確認する。

## 2026-07-11 実装・検証記録

- `make autoware-build`: 成功（25 packages）。
- `test_v2x_overtake_core`: 16/16成功。
- `test_awsim_boost_start_dash`: 17/17成功。
- package test全体: 14 test targets中13成功、合計261 tests中既存fixture 1件のみ失敗。
- 失敗は`PathCoreCircular.RemovesOneEndpointFromConfiguredFinalVer3Trajectory`。現在のtracked `env/final_ver3/traj_mincurv.csv`が重複終端を持たない一方、既存testが重複終端を固定期待しているためで、本変更のV2X/速度/Boost差分ではない。trajectoryまたはfixtureの正本判断が必要なため本作業では変更していない。
- dev2/dev3、最初のヘアピン、実OSQP failure回帰は未実施。シミュレータ起動を伴うためruntime verificationとして残す。
