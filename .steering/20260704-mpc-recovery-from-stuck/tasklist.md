# MPC Stuck Recovery Tasklist

作成日: 2026-07-12
更新日: 2026-07-12
状態: Implementation Complete / P1-P2 AWSIM Scenario Verification Pending

## Definition of Done

- [ ] feature flag無効時に現行MPCの制御結果と40 Hz周期を回帰させない（pure coreの無効時出力は確認済み、40 Hzは未計測）。
- [x] 意図的停止と壁スタックを区別して検出できる。
- [ ] AWSIM標準壁リカバリーと独自Recoveryが競合しない。
- [x] gear report確認前に駆動加速度を出さない。
- [x] 後方が安全かつ情報がfreshな場合だけ限定的に後退する。
- [x] 後方情報unknown、gear timeout、odom異常ではSafeStopとなる。
- [x] 後退距離、時間、signed speed、attemptにhard upper limitがある。
- [x] Recovery中にNormal MPC commandまたはBoostを同時publishしない。
- [x] 復帰後にMPC、filter、V2X / overtake状態をresetし、低速で再合流する。
- [ ] 正面接触、後方壁、後方車、誤検知、gear欠落の自動テストがある（pure static / FSM testは済み、node V2X後方車testは未実装）。
- [ ] make autoware-buildと対象package testが成功する（buildとRecovery 50 testは成功、既存trajectory fixture 1件が失敗）。
- [ ] dev2 / dev3で後続車への接触と意図的妨害がない。
- [x] 公式確認事項とローカル暫定値を文書で区別する。
- [x] package README、mpc-integration、participant-interfaceが実装と一致する。

## Phase 0: 調査・要求確定

- [x] git status --shortで既存変更を確認する。
- [x] input.mdを原資料として確認する。
- [x] 現行MPCが速度下限0、abs速度入力、減速fallbackであることを確認する。
- [x] 現行collision heuristicと未使用is_collidingを確認する。
- [x] 現行の最終control publisherを確認する。
- [x] 2026公式gear command / statusとREVERSE値を確認する。
- [x] AWSIMでlongitudinal.speedが未使用であることを確認する。
- [x] 2026公式wall recoveryの発動条件、時間、角速度を確認する。
- [x] 手動復帰と自律復帰の公式未確定点を確認する。
- [x] requirements.mdを作成する。
- [x] design.mdを作成する。
- [x] tasklist.mdを作成する。
- [x] 利用者レビューで対象範囲、段階導入、SIM限定方針を確定する。

## Phase 1: 公式確認とGear Baseline

- [x] docs/interface/participant-interface.mdへgear command / status契約を追記する。
- [x] docs/spec/open-questions.mdへ自律Recovery確認事項を追記する。
- [x] 運営チャットへ確認し、低速・短時間・後方clearのリカバリ限定、戦略利用禁止寄りの
  運用案内を受けた（公開ルール上の正式許可・数値上限は引き続きTBD）。
- [ ] 最大距離、速度、時間、回数、妨害判定を公式チャットへ確認する。
- [ ] online SIMでgear_statusが配信されることを確認する。
- [x] REVERSE時のacceleration符号を確認する（正が駆動、負が停止）。
- [x] DRIVE -> REVERSE -> DRIVEのgear report遷移時間を計測する。
- [ ] GearCommandの再送要否と適切なQoSを確認する。
- [x] reverse時のVelocityReport / odometry signed velocityが負になることを確認する。
- [ ] racing_kart_gnss_poserのreverse orientation処理を実走確認する。
- [ ] AWSIM wall recovery中のpose、yaw、velocityを記録する。
- [ ] collision penalty中のpose、velocity、commandを記録する。
- [x] 単車校正結果をdesign.mdとtasklist末尾へ記録する。

## Phase 2: Pure Core / Shadow Detector

- [x] stuck_recovery_core.hpp / cppを追加する。
- [x] DetectorInput、DetectorDecision、reject reason enumを追加する。
- [x] race Start / Finish / control enabledのeligible判定を追加する。
- [x] 前進要求とsigned speedの判定を追加する。
- [x] pose displacement windowを追加する。
- [x] 周回seam対応の連続path progressを追加する。
- [x] stationary durationとhysteresisを追加する。
- [x] wall evidenceとcollision hintを補助入力として追加する。
- [x] AWSIM wall recovery settle waitを追加する。
- [x] Follow、SafetyBrake、LowSpeedAvoidance、operator stop除外を追加する。
- [x] solver fallback、odom stale、non-finite除外を追加する。
- [x] 連続2.0秒fallback + current wall + path前進要求 + 停止 / 無進捗だけをRecovery候補にする。
- [x] fallback例外ではcollision hint単独を拒否し、solver復帰で継続timerをresetする。
- [x] 0.2秒を超える観測gapでstationary / fallback timerをresetする。
- [x] Shadow modeでcandidate / reject reasonだけをログする。
- [x] feature flag既定false、shadow既定true、simulation_only既定trueを追加する。
- [x] 同一入力に対する決定性testを追加する。
- [ ] config省略時disabled testを追加する。
- [x] 現行MPC core testの回帰を確認する（Recovery core 33件と既存core testは成功）。

## Phase 3: Recovery FSM / Gear I/O

- [x] RecoveryStateとRecoveryActionをpure coreへ追加する。
- [x] NORMAL -> SUSPECT_STUCK -> WAIT_AWSIM_RECOVERYを追加する。
- [x] STOP_AND_CONFIRMと停止hold時間を追加する。
- [x] CHECK_CLEARANCEとWAIT_FOR_CLEARを追加する。
- [x] SHIFT_TO_REVERSE / WAIT_REVERSE_REPORTを追加する。
- [x] REVERSE_MANEUVERを追加する。
- [x] STOP_BEFORE_DRIVE / SHIFT_TO_DRIVE / WAIT_DRIVE_REPORTを追加する。
- [x] LOW_SPEED_REJOINとSAFE_STOPを追加する。
- [x] state timeout、attempt、cooldownを追加する。
- [x] Spawned / Finish / new session resetを追加する。
- [x] operator stopとodom failsafeの共通割込みを追加する。
- [x] package.xmlへautoware_auto_vehicle_msgs依存を追加する。
- [x] CMakeLists.txtへcoreとunit testを追加する。
- [x] /control/command/gear_cmd publisherを追加する。
- [x] /vehicle/status/gear_status subscriberを追加する。
- [x] gear report freshnessとrequest / report loggingを追加する。
- [x] GearActuationAdapterへ実測済みのreverse加速度意味を実装する。
- [x] gear report前に駆動を出さないtestを追加する。
- [x] gear timeout -> SafeStop testを追加する。
- [x] Drive report確認周期からsolver復帰を必須とし、fallback中のLowSpeedRejoinを阻止する。

## Phase 4: Single Command Ownership / Straight Reverse MVP

- [x] FinalCommandArbitratorをnodeへ統合する。
- [x] Normal / Recovery / SafeStopを排他的に選択する。
- [ ] 1周期でcontrol commandを二重publishしないtest seamを追加する。
- [x] Recovery中にAWSIM Boostを禁止する。
- [x] legacy boost acceleration構成でもRecovery arbitration時はlegacy boostを無効化する。
- [x] signed actual velocityをRecoveryへ渡す。
- [x] Normal MPCのabs速度入力は維持する。
- [x] ReverseStraightだけを実制御へ実装する。
- [x] reverse acceleration、steering、距離、時間、signed speed絶対値にhard limitを追加する。
- [x] speed上限到達時の停止と、command生成側の二重防護を追加する。
- [x] reverse中のodom stale / non-finiteで即停止する。
- [x] runtimeの車体corner motionが0.05 mを超えた場合にfail-closedで停止する。
- [x] reverse中のrear V2X接近で即停止する。
- [x] max_attempts=1を初期値とする。
- [x] disabled時のcontrol output回帰testを追加する。

## Phase 5: Rear Safety / Runtime Footprint

- [x] runtime recovery_footprint coreを追加する。
- [x] front / rear / left / right extentとmarginをconfig化する。
- [x] pose基準とAWSIM colliderのTBDを設定コメントへ記載する。
- [x] world-to-gridでout-of-mapを明示rejectする。
- [x] unknown / invalid cellをoccupiedとして扱う。
- [x] rotated footprint rasterizationを実装する。
- [x] rollout間のswept footprint補間を実装する。
- [x] initial contact cellsとnew contact cellsを区別する。
- [x] 初期接触から単調に離れる条件を実装する。
- [x] initial contactを悪化させる候補をrejectする。
- [x] 前方初期接触だけを許可し、初期patch固定haloと直前8近傍を併用して接触数増加と
  chain migrationをrejectする。
- [x] rolloutとruntime監視で共通のcontact transition helperを使用する。
- [x] clear後の再接触、unknown、離れたwall patch、後方wall通過、chain migration、
  map resolution超過stepをtestする。
- [x] 現行V2X trackerからfresh active vehiclesを安全に取得するadapterを追加する。
- [x] V2X timeout、position jump、self filter、prediction marginを適用する。
- [x] rear information unknown時はReverseを禁止する。
- [x] map boundary、unknown、swept collision unit testを追加する。
- [ ] rear static vehicle / moving vehicle unit testを追加する。

## Phase 6: Three Primitive Evaluation

- [x] ReverseStraight候補rolloutをpure coreへ移す。
- [x] ReverseLeft候補を追加する（pure評価APIのみ）。
- [x] ReverseRight候補を追加する（pure評価APIのみ）。
- [x] 全candidateへ同じstatic hard safety checksを適用する。
- [ ] Left / Rightをruntime候補選択と実controlへ統合する。
- [ ] candidate終端からforward rejoin可能性を確認する。
- [ ] score項目を正規化する。
- [ ] deterministic tie-breakを追加する。
- [ ] 全candidateのaccept / reject reasonをログする。
- [ ] RViz candidate markerをdebug flag付きで追加する。
- [ ] 正面、左前角、右前角のsynthetic testを追加する。
- [ ] 後方壁、後方車、map端でcandidateなしとなるtestを追加する。
- [x] 二段切返しは本Phaseへ含めない。

## Phase 7: Rejoin / State Reset

- [x] DRIVE report後も既存の現在poseからreference pathへの投影を継続する。
- [ ] Recovery開始前wp近傍を使い、周回の誤枝投影を抑制する。
- [x] MPC current_control / prediction / steering historyをresetする。
- [x] fallback speed / solver countersをresetする。
- [x] acceleration / steering filter historyをresetする。
- [x] V2X behavior / OvertakeLine / target / pass sideをresetする。
- [x] LowSpeedAvoidance target lockをresetする。
- [x] Boost latchを再armしない。
- [x] rejoin speed limitを追加する。
- [x] lateral / heading errorとhold時間でNormal復帰を判定する。
- [x] rejoin timeout -> SafeStopを追加する。
- [ ] Normal速度への連続復帰testを追加する。

## Phase 8: Build / Automated Verification

- [ ] formatter / lintを実行する。
- [x] git diff --checkを実行する。
- [x] YAML parseを確認する。
- [x] make autoware-buildを実行する。
- [x] colcon test --packages-select multi_purpose_mpc_rosを実行する。
- [x] colcon test-result --verboseを確認する。
- [ ] existing path / Boost / V2X / trajectory testを回帰させない（既存trajectory fixture期待1件だけ失敗）。
- [x] topic名、message型、launch entryをinterface文書と照合する。
- [x] /control/command/control_cmd publisherが一意であることを確認する。
- [ ] Recovery disabled時の40 Hz deadlineを比較する。
- [ ] Shadow enabled時の40 Hz deadlineを比較する。
- [ ] primitive評価時のworst-case実行時間を計測する。

## Phase 9: AWSIM Verification

### Gear and wall recovery

- [ ] wall recovery有効 / 無効のcontrolled scenarioを用意する。
- [ ] 標準wall recoveryだけで復帰する場合に独自Recoveryを発動しない。
- [ ] 標準wall recovery後も停止する正面接触を再現する。
- [x] gear report確認後だけ校正用駆動commandを与える手順で車両を確認した。
- [x] REVERSE中の加速度とsigned speedを記録する。
- [ ] max distance / durationで停止する。
- [ ] DRIVE復帰とLowSpeedRejoinを完了する。

### False positive

- [x] Start grid待機で発動しない。
- [ ] Follow停止で発動しない。
- [ ] SafetyBrakeで発動しない。
- [ ] LowSpeedAvoidance待機で発動しない。
- [ ] collision penalty低速中に発動しない。
- [x] 一時的fallbackおよびwall証拠のないfallbackで発動しない。
- [ ] 連続2.0秒fallback + wall証拠 + 無進捗でRecoveryへ入る。
- [ ] odometry stale時にRecoveryで動かない。
- [ ] Finish後に発動しない。

### Rear safety

- [ ] 後方壁で後退しない。
- [ ] 後方静止車で後退しない。
- [ ] 後方接近車で後退開始しない。
- [ ] reverse中の後方接近で即停止する。
- [ ] V2X stale / unknownで後退しない。
- [ ] rear clear後にWAIT_FOR_CLEARから再評価する。

### Normal regression

- [ ] Recovery disabledで現行baseline lap timeを再現する。
- [ ] Shadow modeでlap timeとcontrol hzを悪化させない。
- [ ] 最初のヘアピンを回帰させない。
- [ ] V2X Overtakeを回帰させない。
- [ ] LowSpeedAvoidanceを回帰させない。
- [ ] gate1 / gate2 / gate3相当を回帰させない。
- [ ] dev2で後続車へ接触しない。
- [ ] dev3で後続車または並走車を妨害しない。

## Phase 10: Documentation / Rollout

- [x] docs/interface/participant-interface.mdを更新する。
- [x] docs/spec/mpc-integration.mdを更新する。
- [x] docs/spec/open-questions.mdを更新する。
- [ ] docs/spec/safety-gates.mdを更新する。
- [x] multi_purpose_mpc_ros/README.mdを更新する。
- [x] config全keyへ単位、暫定値、simulation-onlyを記載する。
- [x] stateとreject reasonのログ読み方を記載する。
- [x] build / test / AWSIMコマンドと結果を本ファイルへ記録する。
- [ ] Shadow誤検知率とRecovery成功率を記録する。
- [ ] online有効化判断と根拠を記録する。
- [x] 実車は`simulation_only: true`かつactuation二重ラッチで既定無効であることを確認する。

## 実装時の停止条件

- 公式から自律REVERSEが禁止または未許可と回答された。
- AWSIMでgear reportを安定して確認できない。
- REVERSE時のacceleration意味を決定的に確認できない。
- 後方情報のfreshnessまたはself filterを保証できない。
- runtime footprintとAWSIM colliderの差が大きく、安全candidateを説明できない。
- Shadow detectorがFollow、SafetyBrake、Start gridを繰り返し誤検知する。
- Recoveryを追加すると通常MPCの40 Hz deadlineまたはlap性能が有意に悪化する。
- dev2 / dev3で後続車への接触または危険な妨害が発生する。
- command publisherを一意に保てない。
- 実車安全条件をSIM前提だけで満たそうとする必要が生じる。

停止条件に入った場合は、安全処理やgear確認を迂回せず、ログ、再現条件、
不足している公式仕様を記録して利用者へ確認する。

## 2026-07-12 調査記録

- input.md 520行を原資料として確認した。
- 現行MPCが前進専用であり、negative speed対応だけでは復帰できないことを確認した。
- 2026公式interfaceでGearCommand、GearReport、REVERSE=20を確認した。
- AWSIMでlongitudinal.speedが未使用、accelerationが入力であることを確認した。
- 2026公式simulator仕様でwall recoveryの発動条件、1秒、180 deg/sを確認した。
- 公式ルールでは自律後退復帰が明文化されていないためTBDとした。運営チャットの
  低速・短時間・後方clearな復帰限定、戦略利用禁止寄りの案内を運用制約として採用した。
- 最小リスク構成を、同一node内pure core、単一command owner、Shadow先行とした。
- 既定OFF / Shadow / SIM gate、Recovery FSM、gear I/O、Straight実制御adapter、
  static footprint、fresh V2X / Boost hard condition、再合流resetを実装した。
- Left / Rightはpure rollout / static safety評価まで実装し、runtime候補選択と実control、
  RVizは未実装である。
- `make autoware-build`は成功した。
- `test_stuck_recovery_core`は33 / 33、`test_recovery_footprint`は19 / 19成功した。
- Domain 1の既定configで短時間起動し、`mode=disabled`、Boost既定値維持を確認した。
- Shadowを一時的に有効化した短時間起動で、raw occupancy / footprint初期化を確認した。
- 未較正値のままactuationを一時的に有効化すると、drive / stop符号、実測停止減速度、
  V2X self-filter契約不足として起動拒否されることを確認した。
- AWSIM単車校正で正加速度がReverse駆動、負加速度が停止、signed velocityが負になることを
  確認した。report遅延はReverse約0.035 s、Drive約0.015 s、command-to-effect約0.140 s、
  停止約0.154 s、平均停止減速度約0.628 m/s^2だった。
- 未列挙Domainは既定disabled、P1 / P2をActive、P3をdisabledとし、
  0.8 m / 2.0 s / 0.8 m/s / 1 attemptの上限、
  保守的停止減速度0.4 m/s^2、制御遅延予約0.2 sを設定した。
- 一時fallbackを除外しつつ継続スタックを拾うため、連続2.0秒fallback + current wall +
  path前進要求 + 停止 / 無進捗の限定例外を実装した。
- hostに`pre-commit`がないため全体lintは未完了。Recovery footprintのheader / source /
  testはDocker内`ament_uncrustify`に合格し、C++ buildと`git diff --check`は成功した。
- package全体testは288件中Recovery新規testを含む287件が成功し、既存
  `final_ver3/traj_mincurv.csv`の重複終端fixture期待1件だけ失敗した。
- 3台クリーン起動でDomain 1 / 2は`mode=active`、Domain 3は`mode=disabled`、各Domainの
  V2X entry数が2であることを確認した。
- P1の実走で`Confirmed -> WAIT_AWSIM_RECOVERY -> awsim_recovery_resolved`を確認し、
  AWSIM標準補正で動きが戻る場合は独自Reverseへ進まないことを確認した。
- 最終の前方接触 / 固定initial halo / runtime corner-motion強化はpure testとbuildで確認した。
- 3台・360秒走行でP1に長時間solver fallbackが発生したが、wall evidenceがなく
  `solver_fallback_missing_wall_evidence`となり、Recoveryが誤発動しないことを確認した。
- 再ビルド後のP1 / P2起動と、gear publisherのReliable / Volatile QoSを確認した。
- 正面壁スタックからLowSpeedRejoinまでのend-to-end、標準wall recoveryとの競合、
  40 Hz deadline、dev3後方安全の全シナリオは未検証である。
