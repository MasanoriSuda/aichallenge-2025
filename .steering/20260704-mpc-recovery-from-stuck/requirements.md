# MPC Stuck Recovery Requirements

作成日: 2026-07-12
更新日: 2026-07-13
状態: Implementation Complete / 2026-07-13 AWSIM Verification Pending

## 目的

壁衝突や巻き込みにより車両が前進不能となり、そのまま競争中止になる事象に対して、
現行MPCの通常走行性能を変えずに、限定的かつ検証可能な復帰手段を追加する。

本作業の主目的は次の3点である。

1. 意図的な停止と、壁接触後の復帰不能状態を区別して検出する。
2. 安全条件を満たす場合だけ、ギアを用いた短距離の後退復帰を行う。
3. 復帰できない、または安全を確認できない場合は、無理な操作をせず安全停止を維持する。

現行MPCは予選で競争力を持つため、通常走行経路へ影響する変更を最小化する。
スタック復帰はMPCや将来のMPCCとは別責務として設計し、feature flag無効時の制御結果を
変えてはならない。

## 2026-07-12 実装到達点

本状態の`Implementation Complete`は、P1 / P2のSIM限定Active実装、安全ラッチ、pure unit
test、build、AWSIM単車校正までが実装済みであることを表す。実車利用や競技中の戦略的後退を
許可するものではなく、正面壁スタックからの復帰用途だけに限定する。

実装済み:

- `stuck_recovery_core` pure libraryのdetector、Shadow / SIM gate、Recovery FSM、
  gear report確認、timeout、distance / duration / attempt上限、LowSpeedRejoin。
- `recovery_footprint` pure libraryのoriented footprint、swept interpolation、
  out-of-map / unknown reject、初期接触離脱、Straight / Left / Right rollout API。
- 初期接触から離れる際は、Reverse方向と反対側にある前方接触だけを対象とする。現在の
  接触cellは初期patchの固定1-cell halo内かつ直前patchと同一または8近傍の明示Occupiedに
  限り、接触数増加、patchの連鎖移動、一度clear後の再接触をrejectする。rolloutと実後退中の
  runtime監視は同じpure helperを使用する。初期接触を持つLeft / Rightは、向きが変わる場合の
  penetration単調性を証明できるまでfail-closedとする。
- `mpc_controller_cpp`内の `/control/command/gear_cmd` publisher、
  `/vehicle/status/gear_status` subscriber、signed odometry入力、単一control publisher arbitration。
- 静的Straight swept-footprint、fresh V2X後方corridor、freshかつinactiveなBoostを
  実制御前のhard conditionとするnode adapter。
- Recovery開始後のBoost session抑止と、再合流時のMPC / V2X / Overtake状態reset。
- 未列挙Domainの既定を`enabled: false`とし、Domain 1 / 2だけtrue、Domain 3はfalse、
  `shadow_mode: false`、`simulation_only: true`とするP1 / P2限定Active設定。
- AWSIM単車校正に基づく`reverse_acceleration_sign: 1.0`、
  `reverse_stop_acceleration_mps2: -0.8`、保守的停止減速度`0.4 m/s^2`、
  command-to-effect予約`0.2 s`。
- 3台SIMのV2X観測に基づく`expected_v2x_vehicle_count: 2`、
  `self_filter_mode: excluded`。単独走行ではmessage不足でfail-closedとなり、4台構成では
  期待値を3へ変更する。
- 後退上限`0.8 m`、`2.0 s`、`0.8 m/s`、1 attempt。速度上限はsigned speedの絶対値で
  判定し、到達時は直ちに停止シーケンスへ移る。command生成側でも同じ条件を再確認する。
- 通常のsolver fallbackは引き続き除外する。例外は、fallbackが連続2.0秒以上、現在の
  footprint-to-wall証拠、前進path要求、低速、pose / path無進捗がすべて継続した場合だけ。
  collision hint単独では例外を成立させない。
- callback / odometry観測間隔が0.2秒を超えた場合は、停止時間とfallback継続時間を
  resetし、観測途絶時間を「連続」として加算しない。

2026-07-13追加実装:

- solver正常時の1.5秒evidence-free継続スタック判定。
- AWSIM補正待機後は、補正前に選んだmaneuverを使い回さず、現在pose / contactを基準に候補を再選択する。
- map contactが0でもLeft / Right / Mixed近傍wallがある場合は、static swept rolloutから候補を生成する。
- 通常はReverseを優先し、後続車でReverse corridorが塞がれた場合だけ、static mapとforward V2X
  corridorがclearな`ForwardStraight`を最大0.6 mの代替離脱として許可する。
- evidence-free Recoveryでは、AWSIM補正によるpose変化だけを復帰成功とせず、経路進捗を必須とする。
- V2X position jumpは固定距離だけで判定せず、message間隔と許容速度から正常移動距離を算入する。
- clearance timeoutだけを対象とするSafeStopの0.5秒clear継続再評価。
- Straight / Left / Rightの決定的なruntime選択、episode latch、実操舵command。
- 選択rolloutの横変位を含むrear V2X corridor。

引き続き未実装:

- candidate終端からのforward rejoin可能性score、全候補のRViz表示。
- 初期接触を持つLeft / Rightの方向付きpenetration単調性評価。

引き続き未検証:

- 実際の正面壁スタックからのdetector成立、全FSM遷移、LowSpeedRejoin完了。
- AWSIM標準wall recoveryと独自Recoveryの競合。
- online SIMでの正式な利用可否、dev3の後方車安全全シナリオ、通常lap / 40 Hz回帰。
- 実車での利用。`simulation_only: true`を解除してはならない。

検証結果:

- `make autoware-build`: 成功。
- `test_stuck_recovery_core`: 33 / 33成功。
- `test_recovery_footprint`: 19 / 19成功。
- package全体test: Recovery新規testを含むその他は成功。既存
  `final_ver3/traj_mincurv.csv`の重複終端を期待するfixture 1件だけ失敗。
- AWSIM校正: REVERSE report確認後の正加速度`+0.5 m/s^2`でsigned velocityが負方向へ変化し、
  負加速度`-0.8 m/s^2`で停止。gear report遅延はREVERSE約0.035 s、DRIVE約0.015 s、
  command-to-effect約0.140 s、停止約0.154 s、平均停止減速度約0.628 m/s^2。
- 3台クリーン起動: Domain 1 / 2は`mode=active`、Domain 3は`mode=disabled`、各Domainの
  V2X entry数が2であることを確認。P1は実走中に
  `Confirmed -> WAIT_AWSIM_RECOVERY`へ遷移し、AWSIM標準
  wall recoveryで動きが戻ったため`awsim_recovery_resolved`で通常制御へ復帰した。
  最終の前方接触 / 固定halo安全強化はpure testで検証済みだが、独自Reverseから
  LowSpeedRejoinまでの最終binaryによる全FSM実走は引き続き未完了。

## 2026-07-13 追加要求

最新3台走行`output/20260713-051613`では、P1が前進要求を維持したまま停止し、poseと
path progressも変化しなかったが、AWSIM物理壁とoccupancy map / legacy collision通知の
不一致により`wall=0`、`evidence=0`となり、detectorが`SUSPECT_STUCK`から進まなかった。
一方、`v2x_start_grid_grace_time`を1800秒から5秒へ短縮した走行ではP2 / P3は停止連鎖せず
走行を継続した。

この結果を受け、次を追加対象とする。

- solverが正常で、意図的停止・前方車待ち・gear遷移ではなく、前進要求、低速、pose / path
  無進捗が3秒以上連続した場合に限り、wall / collision証拠なしでもRecovery候補とする。
- solver fallbackについては従来どおりcurrent wall evidenceを必須とし、証拠なし救済を
  適用しない。
- clearance timeoutによるSafeStopだけは後方情報とstatic rolloutを継続再評価し、clearが
  一定時間継続した場合に限りCHECK_CLEARANCEへ戻す。gear / odometry / solver等の故障に
  よるSafeStopはlatchedのままとする。
- runtimeでStraight / Left / Rightを決定的に評価し、Straight優先、次にLeft、Rightの順で
  feasible候補を1つlatchedする。選択後は同じprimitiveを後退終了まで維持する。
- Left / RightのV2X後方corridorは選択rolloutの横変位分を含めて保守的に拡張する。
- 初期接触を持つLeft / Rightは、penetration単調性を証明できるまで引き続きrejectする。
- 後続車の空間確保は、新topicを追加せず、5秒のstart-grid grace、既存の停止車回避、
  `v2x_safety_brake_distance: 6.0`を第一段階として検証する。

## 原資料と関連文書

- ChatGPT Proとの壁打ち結果: input.md
- 現行MPC実装:
  aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp
- 現行設定:
  aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml
- 参加者インターフェース契約: docs/interface/participant-interface.md
- 評価インターフェース契約: docs/interface/evaluation-interface.md
- MPC統合仕様: docs/spec/mpc-integration.md
- 競技ルール要約: docs/spec/competition-rules.md
- 安全ゲート: docs/spec/safety-gates.md
- 2026公式インターフェース:
  https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/interface.html
- 2026公式シミュレーター仕様:
  https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/simulator.html
- 2026公式SW部門ルール:
  https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html

input.mdは相談内容の原資料として保持する。本書、design.md、tasklist.mdを実装判断の正本とし、
input.md内の星評価や暫定数値をそのまま確定仕様として扱わない。

## 確認済みの現状

### 現行MPC

- MPC入力の速度下限は0.0であり、最適化問題から後退速度は生成されない。
- solver failure fallbackも速度を0.0以上に制限し、減速停止するだけである。
- MPCへ渡す実速度は絶対値化されている。
- 現行のOvertakeLineにあるRecoveryは追い越し中断後の横復帰であり、
  壁スタックからの後退復帰ではない。
- odometry stale、NaN/Inf、solver failureに対する安全停止は既に存在する。
- /aichallenge/pitstop/conditionから衝突らしき時刻を記録しているが、
  2026公式collision eventではなく、生成したis_collidingも現状未使用である。
- 最終制御出力はmpc_controller_cppが/control/command/control_cmdへpublishしている。

### 2026公式仕様

- /control/command/gear_cmdはGearCommandで、NEUTRAL=1、DRIVE=2、REVERSE=20である。
- /vehicle/status/gear_statusはGearReportである。
- AWSIMではAckermannControlCommand.longitudinal.speedは未使用で、
  longitudinal.accelerationが車両入力として使われる。
- AWSIMには壁リカバリーがあり、壁接触時かつ速度が0.5 m/sを超える場合に、
  1秒間、180 deg/sで姿勢を修正する。
- AWSIMの壁リカバリーは全てのスタックを防ぐものではない。
- SIM決勝・実機決勝の手動復帰は2026-07-12時点で未確定である。
- 自律後退復帰の可否、距離、速度、回数は公式ルールに明記されていない。

## 設計原則

制御判断は次の優先順位を守る。

1. operator stop、race Finish、odometry timeout、非有限値、通信異常。
2. SafetyBrake、後方障害物、後方車両、gear不一致、車体swept-area衝突。
3. AWSIM標準の壁リカバリーが完了するまでの待機。
4. 安全条件を満たす限定的な後退復帰。
5. 低速再合流。
6. 通常MPCへの復帰。

競争復帰を目的として、1〜3の安全処理を無効化してはならない。

## 対象範囲

### 対象

- スタック検出のpure C++ core。
- RecoverySupervisorの状態機械。
- /control/command/gear_cmdのpublishと/vehicle/status/gear_statusのsubscribe。
- signed longitudinal velocityのRecovery側への入力。
- AWSIM標準壁リカバリーとの競合回避。
- 単一の最終command ownerによるNormal / Recovery / SafeStopの排他制御。
- SIM限定・既定無効のfeature flag。
- 直進後退による最初の復帰MVP。
- Straight / Left / Rightのruntime candidate選択と実command。
- 選択candidateのrear V2X corridor hard check。
- 後方静的map、V2X、gear、odometryの安全確認。
- 最大距離、最大時間、最大試行回数、cooldown。
- 復帰後のMPC、filter、V2X追い越し状態の再初期化。
- 低速再合流と通常速度への段階復帰。
- 状態、発動理由、拒否理由、gear応答、移動距離のログ。
- pure C++ unit test、AWSIM integration test、多車両回帰。
- 公式interface確認後のdocs/interface更新。

### 後続段階で対象

- Straight / Left / Rightの全候補にforward rejoin可能性を統一適用する。
- 必要性が実測された場合の単位正規化済みscore。現実装は安全判定後の固定優先順を使う。
- 候補軌跡と拒否理由のRViz可視化。

### 対象外

- 現行MPCへ負速度を許可すること。
- MPCからMPCCへの移行。
- gearを最適化変数に持つhybrid MPC / MPCC。
- Hybrid A*、Reeds-Shepp、後退専用MPCの初期導入。
- /admin/awsim/reset、teleport、respawnを参加者コードから実行すること。
- /set_initial_poseを物理車両復帰として使用すること。
- Domain 0の管理topicや評価FSMを変更すること。
- 2026非公式のcollision topicを必須契約にすること。
- 後方安全情報が不明な状態で、実車をblind reverseさせること。
- 初期段階での実車有効化。
- 壁衝突原因となったtrajectoryやMPC tuningの問題をRecoveryで隠すこと。

## 機能要求

### R-COMPAT: 通常走行互換性

- R-COMPAT-01: feature flag無効時は、制御出力、MPC問題、Boost、V2X behavior、
  topic publish周期を現行から変えない。
- R-COMPAT-02: control_method=mpcと既存launch entryを維持する。
- R-COMPAT-03: /control/command/control_cmdの名前、型、責務を維持する。
- R-COMPAT-04: aichallenge_system、result JSON、Domain 0管理契約を変更しない。
- R-COMPAT-05: Recoveryを別ROS publisherとして追加せず、最終制御publisherを一意にする。
- R-COMPAT-06: 既存OvertakeLineのRecoveryとStuckRecoveryを設定名、state名、
  ログ名で混同しない。

### R-OFFICIAL: 公式仕様と有効化

- R-OFFICIAL-01: gear topic実装前にdocs/interface/participant-interface.mdへ
  2026公式gear契約を反映する。
- R-OFFICIAL-02: 運営チャットの「技術的には実装可、低速・短時間・後方確認付きの
  リカバリ用途に限定し、戦略利用は避ける」という案内を設計制約として採用する。
  ただし公開ルール上の正式許可とは区別する。
- R-OFFICIAL-03: REVERSE中の加速度符号、gear report遷移時間、QoS、拒否条件を
  AWSIMで実測する。
- R-OFFICIAL-04: P1 / P2での段階検証中はSIM限定とし、P3、未列挙Domain、実車を
  無効のまま保つ。
- R-OFFICIAL-05: 実車有効化はoperator stop、remote control、rear sensingを含む
  別ステアリングで扱う。

### R-DET: スタック検出

- R-DET-01: スタック判定は単一のcollision通知に依存しない。
- R-DET-02: 少なくともrace Start、制御有効、前進要求、低実速度、
  位置変位不足、経路進捗不足の継続条件を組み合わせる。
- R-DET-03: nearest waypoint IDだけを進捗の正本にせず、pose変位と
  周回seamを考慮した連続path progressを併用する。
- R-DET-04: 壁近傍、急減速、legacy collision signalは補助証拠として使用できるが、
  単独でRecoveryを開始しない。
- R-DET-05: AWSIM標準壁リカバリーの1秒間と、その直後のpose / yaw安定待ちを設ける。
- R-DET-05A: AWSIM補正によるpose変化だけでは復帰済みとせず、現在の車体footprint contactが
  clearになった場合だけ通常制御へ戻る。contactが残る場合は待機後に限定Recoveryを継続する。
- R-DET-06: detectorはShadow modeを持ち、制御を変えずcandidateとreasonだけを記録できる。
- R-DET-07: 同じ入力、時刻、設定に対して決定的な判定を返す。
- R-DET-08: solver正常時に限り、意図的停止ではない前進要求、低実速度、pose / path無進捗が
  設定時間以上連続した場合、wall / collision証拠なしでも限定的にConfirmedとできる。
- R-DET-09: 証拠なしConfirmedの有効化と継続時間は独立parameterとし、既定無効または
  fail-closedな値を持つ。
- R-DET-10: 前進intentは瞬間的なMPC解だけでなくreference path速度要求との最大値を使用し、
  stopped中のMPC target再構築で0 / 非0が交互になっても無進捗timerを不当にresetしない。
- R-DET-11: solver正常の証拠なしConfirmedでmap footprintと後方3.0 m Straight rolloutがclear、
  V2X情報がfresh / completeかつ後方clearの場合だけReverseStraightを許可する。map invalid、
  out-of-map、unknown、solver fallback、V2X不完全ではこのfallbackを使用しない。escape完了には
  episode実測2.0 mを必須とする。

### R-EXCLUDE: 誤検知防止

- R-EXCLUDE-01: Start前、Finish後、control disable中はRecoveryへ入らない。
- R-EXCLUDE-02: odometry stale、非有限値、短時間または壁証拠のないsolver fallback中は
  既存fail-safeを優先する。連続2.0秒以上のfallbackは、現在のwall footprint証拠、
  前進path要求、停止、pose / path無進捗が全て継続する場合だけRecovery候補にできる。
- R-EXCLUDE-03: V2X Follow、SafetyBrake、LowSpeedAvoidance、停止車待ちを
  スタックと誤判定しない。
- R-EXCLUDE-03A: 通常V2X behaviorの除外状態はRecovery開始判定にだけ使用する。Recovery開始後は
  一時的なFollow / SafetyBrake表示を`control_interrupted`へ変換せず、選択方向のfresh / completeな
  V2X corridorを駆動可否の正本とする。control disableや明示hard stopは引き続き即時停止する。
- R-EXCLUDE-04: gear切替中、Recovery cooldown中、復帰後再合流中に
  新しいRecoveryを重複開始しない。
- R-EXCLUDE-04A: detector更新間隔が設定上限を超えた場合、停止とfallbackの
  継続観測timerをresetする。
- R-EXCLUDE-05: collision penaltyによる低速化だけでRecoveryを発動しない。
- R-EXCLUDE-06: 後方に他車がいる、または後方情報がfreshでない場合、
  初期実装では後退せずWAIT_FOR_CLEARまたはSafeStopとする。

### R-FSM: Recovery状態機械

- R-FSM-01: 少なくともNORMAL、SUSPECT_STUCK、WAIT_AWSIM_RECOVERY、
  STOP_AND_CONFIRM、CHECK_CLEARANCE、SHIFT_TO_REVERSE、WAIT_REVERSE_REPORT、
  REVERSE_MANEUVER、STOP_BEFORE_DRIVE、SHIFT_TO_DRIVE、WAIT_DRIVE_REPORT、
  LOW_SPEED_REJOIN、SAFE_STOPを区別する。
- R-FSM-02: 状態遷移ごとに一意なreasonと開始時刻を記録する。
- R-FSM-03: 車速が停止閾値以下で所定時間継続するまでgearを変更しない。
- R-FSM-04: gear reportが要求gearと一致するまで駆動加速度を出さない。
- R-FSM-05: gear report timeout、不正値、stale時はSafeStopへ遷移する。
- R-FSM-06: reverse中の新しい障害物、V2X接近、odom異常、距離上限、
  時間上限で直ちに停止フェーズへ遷移する。
- R-FSM-07: 試行回数上限、gear / odometry / solver異常後はRecoveryを無限再試行せず、
  latched SafeStopとする。clearance timeoutだけは安全条件の継続確認後に再開できる。
- R-FSM-08: Spawned / Finish / 新race sessionでstateとattemptを決定的に再初期化する。

### R-GEAR: ギア管理

- R-GEAR-01: GearCommandの定数を使用し、数値20などを制御ロジックへ散在させない。
- R-GEAR-02: command送信時刻、要求gear、受信gear、応答時間をログへ残す。
- R-GEAR-03: REVERSEとDRIVEの切替前にはaccelerationを停止側へ固定する。
- R-GEAR-04: gear commandの再送方針を決め、無制限の高頻度publishを行わない。
- R-GEAR-05: signed velocityをRecoveryへ渡し、通常MPCのabs速度処理は初期段階で維持する。
- R-GEAR-06: Recovery中はBoostを発動しない。
- R-GEAR-07: gear reportを使用するracing_kart_gnss_poserとの整合を確認する。

### R-SAFE: 後退安全判定

- R-SAFE-01: 静的mapに対して向き付き車体矩形の全swept-areaを評価する。
- R-SAFE-02: map外、unknown cell、座標変換失敗をoccupiedとして扱う。
- R-SAFE-03: vehicle footprintはwheelbaseではなく、pose基準のfront / rear /
  left / right extentとmarginを明示設定する。
- R-SAFE-04: AWSIM colliderと設定footprintの対応が未確認であることを記録し、
  safety証明として断定しない。
- R-SAFE-05: 初期poseが壁cellへ接触している場合、全候補を即時rejectせず、
  Reverseで離れられる前方接触だけを対象とする。接触は初期patchの固定1-cell halo内かつ
  直前patchの同一または8近傍に限って許し、接触数増加、chain migration、一度clear後の
  再接触をrejectする。
- R-SAFE-06: V2Xはtimeout、position jump、self filter、prediction marginを適用する。
- R-SAFE-07: V2Xだけを後方安全の完全な証明とせず、freshness不明時は安全側へ倒す。
- R-SAFE-08: Safety条件はscoreではなくhard rejectとして候補選択より先に適用する。

### R-MANEUVER: 復帰操作

- R-MANEUVER-01: 実制御はStraightを優先し、Straightが不可能で初期footprintがclearな場合に
  限りLeft、Rightの順で操舵付き後退を選択できる。
- R-MANEUVER-02: 後退距離、継続時間、実速度、加速度、操舵角、試行回数に
  hard upper limitを持つ。初期値は0.8 m、2.0 s、0.8 m/s、1 attemptとする。
- R-MANEUVER-03: longitudinal.speedがAWSIMで未使用であることを前提にせず、
  実測したgearとaccelerationの意味をadapterへ閉じ込める。
- R-MANEUVER-04: Phase 3ではReverseStraight、ReverseLeft、ReverseRightの
  3候補だけを決定的に評価する。
- R-MANEUVER-05: 候補はfull footprint、壁、map外、後方車両を満たさなければrejectする。
- R-MANEUVER-06: scoreを使う場合は単位を正規化し、同点tie-breakを固定する。
- R-MANEUVER-07: 二段切返しは3候補の失敗ログが蓄積してから追加する。

### R-DIRECTION: 壁方向に応じた離脱

- R-DIRECTION-01: occupancy gridの近傍wall cellを車体座標へ変換し、Front / Rear /
  Left / Right / Mixed / Unknownを決定的に分類する。
- R-DIRECTION-02: Frontでは従来のReverse候補、RearではDriveのままForwardStraight候補を
  使用し、接触方向へ押し続けない。
- R-DIRECTION-03: Left / Right / MixedはR-SIDE-ESCAPEの改善証明がある場合だけ駆動し、
  Unknownでは推測せずfail-closedとする。
- R-DIRECTION-04: Rearからの前進もfull swept footprint、complete / fresh V2X corridor、
  Boost inactive、fresh Drive reportを駆動前に必須とする。
- R-DIRECTION-05: 前進は0.6 m、1.5 s、0.8 m/s、1 attemptのローカル上限を持ち、
  0.30 m離脱後はLowSpeedRejoinへ移る。
- R-DIRECTION-06: Reverse reportとV2X完全性が別周期で到着した場合、およびReverseManeuver中に
  V2X完全性が一時欠落した場合は、情報が揃うまでReverseの停止commandを維持する。
  1周期の欠落だけで駆動、Drive復帰、または脱出ステップの進捗初期化を行わない。

### R-SIDE-ESCAPE: Side / Mixed段階離脱

- R-SIDE-01: Side / Mixedかつ実footprint contactがある場合、前進衝突からの離脱を原則とし、
  ReverseのStraight / Left / Right、合計3候補を同じswept-footprint規則で評価する。
  ForwardStraightはRearと明確に分類できた場合だけ使用する。
- R-SIDE-02: 1候補は0.40 m以下とし、初期contact数を超えず、新しい非連結contactを作らず、
  終端で初期contactの5%以上を減らす場合だけacceptする。
- R-SIDE-03: accept候補はcontact減少数最大、同点時はStraight、Left、Rightの決定順で選ぶ。
- R-SIDE-04: 1ステップごとに完全停止し、実測contact数が減少した場合だけ次候補を再評価する。
- R-SIDE-05: 実contactが減らない、map / gear情報が不完全、8ステップ以内にepisode実測
  2.0 mへ到達しない場合はSafeStopとする。Reverse中の失敗は停止してDriveへ戻してから
  SafeStopとする。V2X情報の一時欠落はReverseを維持して停止し、回復すれば距離・contact基準を
  初期化せず同じステップを再開する。情報欠落だけではDriveへ戻さない。
- R-SIDE-06: 段階中にwall regionがFront / Rearへ変化しても、接触が残る間は段階評価を継続する。
- R-SIDE-07: Front / Rearの既存RequireClear候補と固定initial halo規則を弱めない。
- R-SIDE-08: V2X / static clearanceが成立したWAIT_FOR_CLEAR周期では同じsnapshotを使って
  gear要求まで進め、次周期の情報欠落で成立済み候補を失わない。
- R-SIDE-09: gear要求前は0.40 m終端の予測改善を必須とし、開始後は新規contactとcontact増加を
  毎周期禁止する。残距離ごとに追加5%改善を要求せず0.40 mまで継続し、終点で実改善を確認する。

### R-REJOIN: 復帰後の再合流

- R-REJOIN-01: DRIVE report確認後に現在poseをreference pathへ再投影する。
- R-REJOIN-02: MPC prediction、previous steering、fallback履歴、filter、
  solver failure countをresetする。
- R-REJOIN-03: V2X behavior、overtake target、pass side、LowSpeedAvoidance lockをresetする。
- R-REJOIN-04: heading errorとlateral errorが閾値以内になるまで低速上限を適用する。
- R-REJOIN-05: 再合流中も壁、前方車、SafetyBrakeを優先する。
- R-REJOIN-06: 通常速度へ戻すときに加速度や操舵のstepを作らない。
- R-REJOIN-07: Front / Sideはstepをまたぐepisode実移動2.0 m、Rearは実移動0.30 mを
  escape条件とし、距離未達またはfootprint非clearではLowSpeedRejoinへ進まない。
- R-REJOIN-08: V2X / Boost情報の欠落だけでescapeまたはLowSpeedRejoin完了を成立させない。
  LowSpeedRejoin中に不完全となった場合は停止保持し、complete復帰後に再開する。
- R-REJOIN-09: Drive確認後にescape条件未達なら`escape_not_confirmed` SafeStopとし、
  `e_y` / `e_psi`だけで完了させない。

### R-OBS: 観測性

- R-OBS-01: detector条件、除外条件、state、gear、pose変位、path progress、
  target speed、実速度を同一時系列で追跡できる。
- R-OBS-02: 後退候補ごとにaccept / rejectと理由を記録する。
- R-OBS-03: Recovery開始、AWSIM待機、gear要求、gear確認、後退開始、
  停止、再合流、完了、abortをログで区別する。
- R-OBS-04: debug無効時に40 Hzの周期ログを増やさない。
- R-OBS-05: Recoveryの発動回数、成功回数、失敗理由、移動距離を
  評価後に集計できる。
- R-OBS-06: state / reason変化時にmaneuver距離、episode距離、停止予約、escape target、
  escape成立、Boost freshness、V2X message completeness、`e_y`、`e_psi`を記録する。

## 非機能要求

- Recovery判定coreはrclcpp、OSQP、GUIから分離し、synthetic入力でunit testできること。
- command arbitrationは単一thread上で決定的に行い、NormalとRecoveryの二重publishを防ぐこと。
- Shadow detectorは40 Hz deadlineへ有意な悪化を与えないこと。
- primitive評価は状態entry時または低頻度で行い、40 Hzの安全監視だけを毎周期行うこと。
- runtime footprint checkerはallocationとmap copyを制御周期ごとに繰り返さないこと。
- config key省略時は安全な既定値で起動し、Recoveryを無効にすること。
- 実車では明示許可なしに有効化しないこと。
- output、rosbag、result JSONを編集しないこと。

## 受け入れ条件

### 自動テスト

- 前進要求、低速度、無進捗が所定時間継続した場合だけSUSPECT_STUCKとなる。
- 一時停止、Follow、SafetyBrake、LowSpeedAvoidance、Start前、Finish後、
  odometry stale、短時間fallback、壁証拠のないfallbackではRecoveryへ入らない。
- solver正常かつ意図的停止でない証拠なしスタックは、設定時間未満ではSuspectedを維持し、
  設定時間到達後だけConfirmedとなる。
- 連続2.0秒のsolver fallbackでも、現在のwall footprint証拠、path前進要求、停止、
  pose / path無進捗が揃った場合だけRecoveryへ入る。
- AWSIM姿勢補正中のpose / yaw変化でdetector timerが適切にholdまたはresetされる。
- gear reportが来る前に駆動加速度を出さない。
- gear timeoutでSafeStopとなる。
- episode reverse距離、時間、attempt上限で停止する。signed speed上限到達時はReverseのまま
  減速し、上限未満へ戻った場合だけ同じmaneuverを再開する。
- 後方車、後方壁、map外、unknown cellを含む候補をrejectする。
- Straight / Left / Rightから決定的に候補を選び、選択操舵角をReverse commandへ渡す。
- Front wallではReverse、Rear wallではForwardStraightを選び、Side / Mixedでは改善候補、
  Unknownでは駆動候補を生成しない。
- Rear接触から離れるForwardStraightだけを許し、前方接触を貫通する前進候補をrejectする。
- Side / MixedではReverse 3候補から予測contactが5%以上減る候補だけを選ぶ。
- 段階移動後の実contactが減らない場合は次ステップへ進まない。
- 段階移動は0.40 m、最大8回で、各回の間に停止確認と再評価を行い、step間でepisode距離を
  resetしない。
- Reverse gear確認直後にcorridor情報が1周期欠落しても停止したまま再評価し、clear確認後だけ
  ReverseCreepへ入る。
- ReverseManeuver中にcorridor情報が一時欠落しても停止し、回復後は移動距離とcontact基準を
  初期化せず同じ脱出ステップを再開する。
- corridor情報がtimeoutを超えて不完全でもReverse停止を維持し、情報欠落だけを理由にDrive、
  LowSpeedRejoin、RejoinCompleteへ進まない。
- stepwise Reverse完了後のDrive reportは、solver fallback中でもLowSpeedRejoinへ誤遷移せず、
  STOP_AND_REASSESSから次候補を再評価する。LowSpeedRejoin自体はsolver正常を必須とする。
- clearance timeout後も危険中は停止し、clear継続確認後だけRecoveryを再開する。
- 初期接触から離れる直進候補を正しく扱う。
- 初期halo内の隣接・斜め隣接cellへの1対1移動と接触縮小を許し、接触数増加、halo外への
  chain migration、後方wall通過、unknown、clear後の再接触をrejectする。
- Recovery完了時にMPCとV2X lockがresetされる。
- Drive確認後もescape未達ならSafeStopとなり、Front / Side実測2.0 m、footprint clear、
  V2X complete、lateral / heading整列が揃った場合だけRejoinCompleteとなる。
- feature flag無効時の既存core testが回帰しない。

### ビルド・静的検証

- make autoware-buildが成功する。
- multi_purpose_mpc_rosのunit testが成功する。
- package.xmlとCMakeLists.txtへautoware_auto_vehicle_msgs依存が整合して追加される。
- docs/interface/participant-interface.mdのgear契約が公式仕様と一致する。
- /control/command/control_cmdのpublisherが一意である。
- git diff --checkとYAML parseが成功する。

### AWSIM検証

- [確認済み] 停止中のDRIVE / REVERSE commandとgear reportの関係を記録する。
- [確認済み] REVERSE中のacceleration符号とsigned velocityを確認する。
- AWSIM標準壁リカバリーの発動中に独自Recoveryが競合しない。
- 正面壁接触後、標準補正で解消しない場合だけ短距離後退する。
- 後方車がいる場合は後退せず停止または待機する。
- 後方壁がある場合は後退しない。
- gear report欠落時は動かない。
- 後退中のV2X接近で停止する。
- 復帰後、低速再合流から通常MPCへ戻る。
- Recovery無効時のlap time、control hz、ヘアピン、追い越し挙動がbaselineと一致する。
- dev2 / dev3で後続車へ接触せず、他車の進路を意図的に妨害しない。

## 未確定事項

- SIM予選で参加者コードによる自律REVERSE復帰が許可されるか。
- 最大後退距離、速度、時間、試行回数に公式制限があるか。
- 後退が逆走、蛇行、妨害として評価される条件。
- 配布online SIMでもローカルAWSIM校正と同じacceleration符号・GearReport QoS・応答を持つか。
- GearCommandを再送すべき公式条件。
- online SIM予選でgear_statusが常時publishされるか。
- wall recovery中またはcollision penalty中を示す公式statusが提供されるか。
- 2026環境で利用できる公式collision eventの有無。
- V2Xに自車が含まれるか、vehicle IDが一意か、後方全車両が取得できるか。
- 2D LiDARの後方視野と、SIM予選・実車でのrear safety入力としての利用可否。
- trajectory pose、base_link、rear axle、AWSIM colliderの正確な対応。
- manual recovery、checkpoint、lap count、collision penaltyとの関係。
