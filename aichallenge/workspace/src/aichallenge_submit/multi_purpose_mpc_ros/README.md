# multi_purpose_mpc_ros

このパッケージはリポジトリ内（`aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`）に直接収録されています。別途 `git clone` は不要です。

## build

autoware コンテナ内で実行します（`make autoware-bash` または `make autoware-build`）：

```bash
cd /aichallenge/workspace
colcon build --symlink-install --allow-overriding gyro_odometer \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- 通常の MPC 実行系は C++ executable `mpc_controller_cpp` です。
- Python 補助スクリプト用に、ビルド時に仮想環境が `${ROS_WS}/install/multi_purpose_mpc_ros/.venv` に作成されます。

## test

autoware コンテナ内で、C++経路処理、Python trajectory editor／clearance、V2X trackerのテストを実行します。

```bash
cd /aichallenge/workspace
colcon test --packages-select multi_purpose_mpc_ros --event-handlers console_direct+
colcon test-result --verbose
```

## trajectory validation

MPC が使用する7列 trajectory CSV を、実行時と同じ strict loader で検証できます。

```bash
ros2 run multi_purpose_mpc_ros reference_path_validator \
  $(ros2 pkg prefix --share multi_purpose_mpc_ros)/env/final_ver3/traj_mincurv.csv \
  --circular
```

必須列、数値変換、有限値、`s_m` の単調増加、周回重複終点、点間隔、曲率、速度、加速度を確認します。`--resolution <m>` を付けると、点間隔が指定値の105%を超えた場合に非0で終了します。現行raw CSVは0.25m上限を満たさない場合があります。Editorで生成したnormalized copyには`--resolution 0.25`を使用できますが、runtime側の補間と固定Nの移行は別課題です。

本番の内部補間は、固定 `mpc.N` の実距離を維持するため、現時点では legacy `floor` 方式です。`ceil` 分割は pure helper とテストまでを先行追加しており、距離ベース horizon と同時に段階導入します。

## runtime safety settings

- `mpc.odom_timeout_sec`: odometry受信と非ゼロsource stampの更新をsteady clockで監視するローカルtimeout。既定は0.5秒です。
- `mpc.min_linearization_speed_mps`: `1/v` を含むモデルを使わない低速閾値。既定は0.5 m/sです。

staleまたは非有限なodometry、非有限な制御出力、OSQP失敗時には、古い予測制御列を再生せず、速度を下げるfail-safe commandへ移ります。solver fallbackとcontrol disable時はlegacy boostを強制無効化します。これらの既定値は2026公式値ではなく、走行ログとSafety Gateで調整する暫定ローカル基準です。

## stuck recovery（P1 / P2 / P3 SIM限定Active）

壁接触後に前進できない状態を検出するpure C++ core、gear確認を含む
Recovery FSM、静的occupancy grid上の車体swept-footprint検査を追加しています。
通常MPCと将来のMPCCとは別責務で、最終
`/control/command/control_cmd`はpublisherを増やさず既存C++ nodeが1周期に1回だけ
publishします。

現在の設定と多重ラッチは次の通りです。

- 未列挙Domainの既定は`enabled: false`。dev3のDomain 1 / 2 / 3だけ
  `domain_enabled: true`、`shadow_mode: false`です。
- `simulation_only: true`のため、実車環境では制御権を取得しない。
- AWSIM校正済みの`reverse_acceleration_sign: 1.0`、駆動`+0.5 m/s^2`、
  停止`-0.8 m/s^2`を使う。保守的停止減速度は`0.4 m/s^2`、制御遅延予約は`0.2 s`。
- Front / Side後退は実測`2.0 m`のescapeを必須とし、停止予約込み最大`3.0 m`、
  `4.0 s`、`0.8 m/s`、最大3 attempt。速度上限到達時はReverseのまま一時減速し、
  上限未満へ戻れば同じmaneuverを再開する。
- 近傍wallを車体基準で分類し、Frontは後退、RearはDriveのまま最大`0.6 m`、
  `1.5 s`、`0.8 m/s`の直進前進で離脱する。
- Side / Mixedかつmap contactがある場合は、Reverse / Forwardの直進・左・右を同じ0.40 m、
  同じ操舵sample、同じ5% contact改善条件で評価する。Forwardはreverse-onlyでないepisodeかつ
  全Reverse候補よりcontact減少数が大きい場合だけ選び、同値ではReverseを優先する。
  各ステップで停止・実測改善を確認し、episode全体の実移動を保持し、最大10回以内に
  2.0 mへ到達しなければ停止する。
  Unknownまたは改善候補なしでは動かない。
- Reverse中にV2X completenessが一時欠落した場合は即停止し、Reverseを維持して安全情報の
  回復を待つ。情報欠落だけではDriveやLowSpeedRejoinへ移らず、completeへ戻れば
  移動距離・contact基準を保ったまま同じステップを再開する。
- AWSIM標準補正でposeが動いても、現在footprint contactが残る場合は復帰済みと判定しない。
  補正待機後に候補を再評価し、現在footprintがclearの場合だけ通常制御へ戻る。
- 3台SIMではV2Xが自車を除く2 entryだったため、期待数2、self excludedとする。
  単独走行はV2X不足でfail-closed、4台構成では期待数を3へ変更する。
- 停止列の後退V2X判定は、選択済みの0.05 m刻みrolloutに沿って向き付き自車footprintと
  膨張他車円のsigned clearanceを評価する。保守円が初期から重なる場合も、全sampleで重なりを
  悪化させず最終的に改善する後退だけを接触離脱として許可する。これにより最後尾は前方車から
  離れられる一方、後方車へ近づく車両は待機する。他車速度が停止閾値を超える場合は従来の
  現在・予測位置を含むmoving corridorでfail-closed判定する。

現在のP1 / P2 / P3有効化例:

```yaml
stuck_recovery:
  enabled: false
  domain_enabled:
    1: true
    2: true
    3: true
  shadow_mode: false
  simulation_only: true
  reverse_actuation_enabled: true
  reverse_acceleration_sign: 1.0
```

別Domainで再校正する場合は、そのDomainだけ`domain_enabled: true`にした上で
`shadow_mode: true`、`reverse_actuation_enabled: false`から開始してください。

`Stuck detector:`は前進要求、signed speed、pose / path進捗、壁またはlegacy
collision hintとreject reasonを出します。`Stuck recovery:`はexecution mode、FSM
state、action、wall region / 距離、maneuver direction、static candidateの拒否理由、
Boost / V2X完全性、maneuver / episode距離、停止予約、escape target、`e_y` / `e_psi`を
stateまたはreason変化時に出します。V2X block時は車両ID、rollout/moving判定方式、
初期・最小・最終clearance、拒否距離も出します。LowSpeedRejoin中は専用速度目標、実速度、
rate-limit後の操舵、横偏差、heading誤差、static / V2X gateを0.5秒間隔でも出します。
Follow / SafetyBrake / LowSpeedAvoidance、Start前、control disable、odom異常、短時間または
壁証拠のないsolver fallbackはスタック確定から除外されます。例外はfallbackが連続2.0秒以上、
path前進要求、停止、pose / path無進捗、現在のfootprint-to-wall証拠が全て継続する場合だけです。
collision hint単独ではこの例外を成立させません。
detector更新間隔が0.2秒を超えた場合は、停止時間とfallback継続時間をresetします。
solver fallback episodeはwall証拠なしまたは`|e_psi| >= 1.0 rad`の場合だけreverse-onlyです。
wall証拠があり姿勢誤差が小さい場合はwall方向が選んだ候補を既存static / V2X gateで検証します。
このV2X behavior除外はRecovery開始前の誤検知防止です。Recovery開始後は一時的な通常behaviorを
`control_interrupted`へ変換せず、選択方向のstatic / V2X corridorで駆動可否を判定します。
control disableと明示的なRecovery hard stopは従来どおり即時停止します。

solverが正常な場合は、AWSIM物理壁とoccupancy map / collision通知の不一致を救済するため、
意図的停止でない前進要求、低速、pose / path無進捗が1.5秒継続すると、壁証拠なしでも
限定的にRecovery候補となります。solver fallback中はこの救済を使用しません。
前進要求には瞬間的なMPC解とreference path速度要求の最大値を使い、停止中のMPC target再構築で
0 / 非0が交互になってもtimerをresetしません。map footprintと後方3.0 m Straight rollout、
fresh / completeなV2X corridorが全てclearの場合だけReverseStraightを選び、
実測2.0 mのescape後に停止します。

通常footprintの外側0.50 mまでのwall cellを車体座標へ変換し、Front / Rear / Left / Right /
Mixedへ分類します。Frontでは`ReverseStraight`、`ReverseLeft`、`ReverseRight`、Rearでは
`ForwardStraight`を評価します。Side / Mixedのmap contactではReverse / Forwardの
Straight / Left / Rightを0.40 m評価し、改善量最大の候補を選びます。ReverseとForwardが
同じ改善量ならReverseを選びます。近傍wallなしでは推測で動きません。
選択候補はepisode中固定し、どの候補も次をすべて満たさない限り駆動へ進みません。

- 全swept footprintがstatic mapで安全。map外、unknown、新規接触は即reject。非stepwiseの
  初期接触はReverseでは前方wall、ForwardStraightでは後方wallに限定する。Side / Mixedの
  stepwise接触離脱は方向にかかわらず現在cellを初期patchの固定1-cell halo内かつ直前patchと
  同一または8近傍の明示Occupiedだけに限定する。接触数増加、非連続なpatch移動、clear後の
  再接触をrejectし、rolloutと実走中監視は同じ判定helperを使う。
- `/v2x/vehicle_positions`がfreshかつ想定台数分completeで、選択方向へ予測したcorridorに
  車両がいない。期待値は配列の総entry数（環境が自車を含める場合は自車込み）で、
  空ID・重複ID・異常sampleもrejectする。`rear_safety.expected_v2x_vehicle_count: -1`の
  既定値はReverseを阻止する。自車の扱いも`self_filter_mode: unknown`では阻止し、
  公式確認後に`excluded`または正確な`vehicle_id`を明示する。

後方clear待ちがtimeoutした場合も、原因がclearanceだけならSafeStop中に再評価を続け、
0.5秒連続で安全になった後だけRecoveryへ戻ります。gear、odometry、solver、control等の
異常によるSafeStopは従来どおりsession resetまでlatchedします。
- `/awsim/status`がfreshでBoost中ではない。
- `/vehicle/status/gear_status`がfreshで、REVERSE確認後にだけ駆動する。
- Rear-wall前進はfreshなDRIVE report確認後だけ駆動する。
- Side / Mixedは各swept sampleで初期contact数を超えず、新規非連結contactがなく、終端で
  5%以上減少することを要求する。実移動後にも減少を確認し、0.40 mごとに停止・再評価する。
- gear要求後は残距離ごとの追加5%改善を要求せず、contact非増加と新規contactなしを監視して
  0.40 mまで継続する。終点では実測contact減少を改めて必須とする。
- WAIT_FOR_CLEARでstatic / V2X clearanceが成立した周期は、同じsnapshotでgear要求まで進める。
  clear確認とgear要求の間に別周期を挟まず、一時的なV2X欠落で候補を失うことを防ぐ。
- AWSIM補正待機後は待機前のcandidateを使い回さず、現在pose / contactから候補を選び直す。
- evidence-free Recoveryではmap footprintが開始時からclearなので、AWSIMのpose nudgeだけでは
  復帰済みとしない。reference path進捗が確認できない場合はSTOP_AND_CONFIRMへ進む。
- Left / Right / Mixed近傍wallでmap contactが0の場合も、clearなswept rolloutから0.40 mの
  stepwise候補を作る。Reverseを優先し、停止した後続車でReverse corridorだけが塞がれた場合は、
  mapとforward V2X corridorがclearな`ForwardStraight`へ限定的に切り替える。候補固定後の
  途中切替はしない。
- V2X position jumpの許容距離は固定閾値と`v2x_v_max_safety * message_dt`の大きい方とし、
  約1 Hz配信で正常走行車が5 m以上進んでもrear informationを不完全扱いしない。
- stepwise Reverse後のDrive確認は通常MPCを使わず`STOP_AND_REASSESS`へ進めるため、継続中の
  solver fallbackだけでは中断しない。実際のLowSpeedRejoin開始時にはsolver正常を必須とする。
- stepwise Reverse中にcompleteなstatic / V2X blockageが持続した場合も、停止してDriveを確認後、
  step数とepisode距離を保持したまま`STOP_AND_REASSESS`へ戻す。非stepwise blockageは従来どおり
  Drive確認後にSafeStopする。
- ForwardCreep中にstatic / V2X hazardが現れた場合は即時停止して`STOP_AND_REASSESS`へ戻る。
  障害が残れば既存のWAIT_FOR_CLEAR上限を使い、clear後だけ残attemptで再開する。
- 非stepwise Reverseが時間上限へ達しても残attemptがあればDrive停止確認後に再評価する。
- gear command publisherはReliable / KeepLast(1) / Volatile。古いREVERSEを再参加時に
  replayし得るTransientLocalは使用しない。
- episode後退距離、時間、signed speed絶対値、gear request回数、Recovery attemptが設定上限内。
- Drive確認後もepisode escapeが未確認なら`escape_not_confirmed`でSafeStopし、
  LowSpeedRejoinへ進まない。LowSpeedRejoin中のV2X情報欠落は停止保持し、情報復帰後だけ再開する。
- LowSpeedRejoin完了はescape確認、map footprint clear、fresh / completeなV2X、
  `|e_y| <= 0.5 m`、`|e_psi| <= 0.35 rad`の0.3秒継続を必須とする。
- LowSpeedRejoinが5.0秒で整列しない場合は停止し、`retry_on_timeout=true`かつattempt / step
  budgetが残る場合だけescape距離を0へ戻して新しい有界離脱を行う。budget消費後はSafeStopする。
- LowSpeedRejoinは通常MPC速度が接触後に0を返しても、上記hard gate通過中だけ専用目標
  `1.0 m/s`を使用する。操舵は参照曲率feedforwardと`e_y` / `e_psi` feedbackから求め、
  rate-limit後の同じtire angleで0.8 m swept footprintと実commandを評価する。
- solver fallbackを起点に安全条件を通過したepisodeは、その資格をLowSpeedRejoin完了まで保持する。
  これにより離脱移動でdetectorの無進捗条件が消えても独立feedbackで再合流できる。通常の
  recovery中に新しくsolver fallbackが発生した場合は従来どおり停止し、1.0秒の復旧上限を使う。

重要: 上記値は2026-07-12のローカルAWSIM単車校正値です。online SIMや実車へ無条件に
転用せず、`simulation_only: true`を維持してください。正面壁スタックからの全FSM遷移、
標準wall recoveryとの競合、dev3の後方安全全シナリオは引き続き確認が必要です。
運営チャット案内に従い、戦略的な後退には使わず、低速・短時間・後方clearな復帰に限定します。

自動テストでは`stuck_recovery_core` 77件と`recovery_footprint` 34件が成功し、
`make autoware-build`も成功しています。package全体testは既存
`traj_mincurv.csv`の周回末尾fixture期待の1件だけが失敗し、Recovery新規testは
すべて成功しました。`output/20260718-191547`のdev3実験ではrace開始後約3分間、3台とも
走行を継続し、SafeStop / escape step上限 / maneuver方向不明は発生しませんでした。
このrunでは接触Recoveryが発火しなかったため、双方向contact候補の実走選択は未観測です。

## run

### MPC コントローラー
```bash
ros2 run multi_purpose_mpc_ros mpc_controller_cpp \
  --config_path $(ros2 pkg prefix --share multi_purpose_mpc_ros)/config/config.yaml \
  --ref_vel_path $(ros2 pkg prefix --share multi_purpose_mpc_ros)/config/ref_vel.yaml
```

Python 版は比較・検証用に残しています。

```bash
ros2 run multi_purpose_mpc_ros run_mpc_controller.bash
```

#### Domain-specific v_max

`mpc.domain_v_max` を設定すると、`ROS_DOMAIN_ID` ごとに最高車速を km/h で上書きできます。`domain_v_max` に該当 Domain がない場合は `v_max` がそのまま使われます。

```yaml
mpc:
  v_max: 40.0
  domain_v_max:
    1: 40.0
    2: 38.0
    3: 36.0
```

#### Wall-edge trajectory tuning

`config/config.yaml` の `mpc.wp_id_low_offset` / `mpc.wp_id_low_speed` / `mpc.wp_id_offset` で、低速時だけ参照 waypoint の先読み量を変えられます。`wp_id_low_speed` は km/h で指定します。

```yaml
mpc:
  wp_id_low_offset: 1
  wp_id_low_speed: 5.0
  wp_id_offset: 2
```

`config/config.yaml` の `mpc.center_bias` と `mpc.safety_margin_scale` で、編集済み trajectory への追従と壁際の余白を調整できます。

```yaml
mpc:
  center_bias: 0.0          # 0.0 = trajectory追従, 1.0 = 制約中央寄せ
  safety_margin_scale: 1.0 # 0.0 = 追加marginなし, 1.0 = 現行margin
```

まず `center_bias: 0.0`, `safety_margin_scale: 1.0` で中央寄せだけを消し、必要に応じて `safety_margin_scale` を `0.7`, `0.5`, `0.3` の順に下げて確認します。

#### V2X gap planner

C++ MPC には `/v2x/vehicle_positions` から他車位置を受け取り、reference path 上の横方向 gap を選んで `lb/ub` と `xr` に反映する rule-based planner があります。既定では無効です。

```yaml
mpc:
  use_v2x_gap_planner: false
  v2x_vehicle_radius: 1.6
  v2x_vehicle_length: 2.0
  v2x_prediction_margin: 0.2
  v2x_timeout_sec: 1.0
  gap_min_width: 1.8
  gap_target_bias: 1.0
  no_gap_target_velocity: 0.0
  v2x_wall_clearance_margin: 0.0
  v2x_vehicle_side_target_margin: 0.0
  v2x_wall_avoidance_bias: 0.0
  v2x_vehicle_vehicle_gap_enabled: true
  v2x_vehicle_vehicle_gap_min_distance: 0.0
  v2x_vehicle_vehicle_gap_min_width: 0.0
  v2x_multi_front_gap_enabled: true
  v2x_multi_front_gap_distance: 0.0
  v2x_low_speed_pass_side: auto      # auto, left, right
  v2x_low_speed_pass_ramp_ratio: 1.0
```

`V2XVehiclePositionArray` は位置と covariance のみを持つため、C++ 側では vehicle_id ごとの直近2点から速度を推定します。初回、異常ジャンプ、過大速度時は静止障害物として扱います。`v2x_vehicle_radius` は V2X 車両単体の半幅ではなく、自車中心が入ってはいけない横方向禁止幅です。V2X 車両幅 1.45m の場合、相手半幅 0.725m + 自車半幅 0.725m として 1.45m 程度を基準にし、必要に応じて余白を足します。前後方向の占有判定には `v2x_vehicle_length` と自車長を使います。gap がない場合は `no_gap_target_velocity` を速度上限として使います。

free gap が壁と他車に挟まれている場合、`v2x_wall_clearance_margin` で壁側の制約を内側へ削り、`v2x_wall_avoidance_bias` で target を gap 中央から車側へ寄せられます。壁ペナルティを避けたい場合は `v2x_wall_clearance_margin: 0.4`, `v2x_vehicle_side_target_margin: 0.2`, `v2x_wall_avoidance_bias: 0.8` から確認します。

左右が両方とも V2X 車両の free gap は、`v2x_vehicle_vehicle_gap_enabled: false` で候補から外せます。3台同時走行のスタート直後など、前方2台の間をすり抜けようとして操舵が不安定になる場合は、車-車 gap を禁止し、壁-車 gap または no-gap 低速追走へ倒します。

さらに、前方近距離に2台以上いる状況そのものを追い抜き禁止にしたい場合は、`v2x_multi_front_gap_enabled: false` と `v2x_multi_front_gap_distance` を設定します。この条件に入ると gap planner は横目標を作らず、`no_gap_target_velocity` による低速追走へ倒します。

#### V2X behavior FSM

`use_v2x_behavior_fsm: true` にすると、`Cruise` / `Follow` / `Overtake` / `LowSpeedAvoidance` / `SafetyBrake` の最小状態で `V2XGapPlanner` の使用可否を制御します。`Follow` と `SafetyBrake` では gap planner を使わず、速度上限を下げます。`Follow` 中は `v2x_follow_velocity` と前方距離から計算した停止可能速度の小さい方を使います。`Overtake` と `LowSpeedAvoidance` のときだけ gap planner が `lb/ub` と `xr` に反映されます。

```yaml
mpc:
  use_v2x_behavior_fsm: false
  v2x_follow_distance: 8.0
  v2x_safety_brake_distance: 3.0
  v2x_safety_brake_margin: 2.0
  v2x_follow_velocity: 5.0
  v2x_safety_brake_velocity: 0.0
  v2x_overtake_min_gap_width: 2.0
  v2x_overtake_max_curvature: 0.05
  v2x_overtake_try_both_sides: false
  v2x_overtake_velocity_advantage: 0.0
  v2x_overtake_line_enabled: false
  v2x_overtake_target_hold_sec: 0.0
  v2x_overtake_clear_confirm_sec: 0.0
  v2x_overtake_reacquire_enabled: false
  v2x_overtake_reacquire_window_sec: 0.0
  v2x_overtake_reacquire_max_return_progress: 0.0
  v2x_overtake_recovery_velocity_limit_enabled: true
  v2x_overtake_recovery_velocity: 5.0
  v2x_overtake_solver_failure_abort_cycles: 3
  v2x_require_gap_for_overtake: true
  v2x_low_speed_avoidance_enabled: false
  v2x_low_speed_avoidance_distance: 10.0
  v2x_low_speed_avoidance_lookahead_distance: 18.0
  v2x_low_speed_avoidance_velocity: 1.5
  v2x_low_speed_avoidance_max_front_speed: 1.0  # 予備設定。開始条件の hard gate にはしない。
  v2x_low_speed_avoidance_min_gap_width: 0.5
  v2x_low_speed_avoidance_min_gap_points: 2
  v2x_low_speed_avoidance_clear_distance: 8.0
  v2x_low_speed_pass_side: auto      # auto, left, right
  v2x_low_speed_pass_ramp_ratio: 1.0
  v2x_overtake_forbidden_wp_ranges: []
  v2x_state_hold_time: 0.5
```

既定では無効です。`SafetyBrake` は `v2x_safety_brake_distance` と `v^2 / (2 * abs(a_min)) + v2x_safety_brake_margin` の大きい方で判定します。`SafetyBrake` の横方向判定は corridor 全体ではなく、`v2x_vehicle_radius + v2x_prediction_margin` の衝突幅に重なった場合だけ使います。`v2x_low_speed_avoidance_enabled: true` の場合、近距離の前方車両に対して連続した十分な gap があるときは、SafetyBrake より先に `LowSpeedAvoidance` へ倒し、`v2x_low_speed_avoidance_velocity` に速度を制限して徐行回避します。連続点数は `v2x_low_speed_avoidance_min_gap_points` で指定します。V2X メッセージは速度を直接持たないため、差分速度推定は開始条件の hard gate にしません。低速回避中は停止車列を先読みするため、gap planner が見る V2X 車両を `v2x_low_speed_avoidance_lookahead_distance` 程度まで広げ、通常走行では無効にしている車両間 gap も一時的に候補へ戻します。低速回避では `v2x_low_speed_pass_side` で通過側を `auto` / `left` / `right` から選べます。`right` は reference path 座標系の負の lateral 側、`left` は正の lateral 側です。`auto` の場合は最初に選んだ側を低速回避中の side lock として使います。実制御時だけ `v2x_low_speed_pass_ramp_ratio` に従って手前の horizon 点にも side-pass target を入れ、候補判定時は実際に障害物と重なる horizon 点だけで gap を評価します。すでに `LowSpeedAvoidance` 中の場合は、曲率による追い越し禁止区間に入っても gap がある限り低速回避を継続し、さらに `v2x_low_speed_avoidance_clear_distance` 以内に V2X 車両が残る間は通常速度へ戻らず徐行を維持します。`v2x_require_gap_for_overtake: false` にすると、追い越し禁止条件に入っていない前方車は gap 幅の事前判定を必須にせず `Overtake` へ倒します。ヘアピン入口などで追い抜きを禁止したい場合は `v2x_overtake_forbidden_wp_ranges` に wp_id 範囲を追加します。

通常追い越しで `v2x_overtake_try_both_sides: true` にすると、ShiftOut前に限り第一候補側が不成立なら反対側を同じ膨張半径・wall margin・guardで評価します。ShiftOut以降はpass sideをlockし、反対側へ即座に振り替えません。追い越し判定では共通`gap_min_width`による前段除外を使わず、`max(v2x_overtake_min_gap_width, v2x_overtake_guard_min_gap_width)`を候補生成とguardの両方へ適用します。curve zone中も左右の幾何gapはdebugへ残しますが、hard forbidden、soft curve entry、inner curve判定は別の実行許可として維持されます。

OvertakeLine有効時は対象`vehicle_id`とpass sideをlockします。`v2x_overtake_target_hold_sec`以内の一時欠落ではPassを維持し、対象が`v2x_overtake_line_return_clear_distance`以上後方にいる状態を`v2x_overtake_clear_confirm_sec`継続して初めてReturnへ入ります。Return直後の再取得は、安定した同一ID・同一side・gap/curve許可・時間/復帰進捗の全条件を満たす場合だけPassへ戻します。不明ID、position jump、timeout、連続solver失敗はRecovery側へ倒します。Recoveryの速度上限は専用flag/valueで、`v2x_follow_velocity`を暗黙には使いません。

`v2x_behavior_debug_log_enabled`では、左右gapと各拒否理由、対象ID、locked targetの相対位置、soft desired velocity、solver連続失敗数を確認できます。MPC投入前にはbounds・target・速度参照のfinite/range preflightを行い、solverが追い越し中に設定回数連続失敗した場合は減速fallbackを維持したまま同じ横目標を中止してRecoveryへ移ります。

速度上限は`v_max`をglobal hard maximum、`domain_v_max`を通常の車両別maximum、`domain_start_v_max`をStart期間だけのmaximumとして分けます。start値は通常domain値より高くできますがglobal値を超えません。期間はMPC初期化やReady待機ではなく、重複を除いた`/awsim/state=Start`から計時します。通常は`Finish -> Spawned`、Finishを省く手動resetでは`Spawned -> Grounded/Ready -> Start`を新しいsession境界として扱います。Boostを車両別に無効化していても、start速度を設定した車両は`/awsim/state`を購読します。

### MPC シミュレーション
```bash
ros2 run multi_purpose_mpc_ros run_mpc_simulation.bash
```

### Trajectory editor
MPC の `env/final_ver3/traj_mincurv.csv` を Lanelet2 map 上で編集します。

```bash
ros2 run multi_purpose_mpc_ros trajectory_editor
```

任意ファイルと経路の topology を明示する場合:

```bash
ros2 run multi_purpose_mpc_ros trajectory_editor \
  --trajectory /path/to/trajectory.csv \
  --osm /path/to/lanelet2_map.osm \
  --circular
```

非周回経路は `--open` を指定します。引数を省略した組み込み MPC preset は、このリポジトリ固有の周回経路として起動します。これは Automotive AI Challenge 2026 の公式インターフェース仕様ではありません。

安全な基本操作は次の順です。

1. `Open Traj` でCSVを開き、必要なら `Circular` を確認する。
2. `Validate` で error / warning、CSV行、`s_m`、問題値を確認する。
3. 必要なら点を編集する。MPCのXY変更はgeometryとspeed metadataの両方をstaleにする。
4. `Normalize Geometry` で重複終端・退化点の除去、等間隔線形再サンプリング、`s_m/psi/kappa` 再生成、`vx/ax` policyを明示してcandidateを作る。
5. XY、間隔、heading、curvature、速度、加速度、横加速度のBefore/Candidate比較と補正レポートを確認し、`Apply Candidate` または `Discard` を選ぶ。
6. `vx/ax`を作り直す場合は `Recompute Speed` で `v_max/a_max/a_min/ay_max/minimum_speed` と収束条件を設定し、同様にpreviewして適用する。
7. `Validate` 後、`Save As` で `*_normalized.csv`、`*_speed_profiled.csv`、または `*_edited.csv` へ別名保存する。

表示は `Original`（読込時の灰色破線）、`Working`（青実線）、`Candidate`
（橙破線）を個別に切り替えられます。`Original` は Save / Save As 後も更新
されず、別のCSVをOpenしたときだけ置き換わります。拡大後は画面下・右の水平／
垂直スクロールバーで移動でき、右・中ドラッグpanも引き続き利用できます。
`Center Selection` は選択点またはvalidation issueを画面中央へ移動します。
`Original diff`にはpoint数差、path長差、最大・平均変位、1cmを超える変更の
`s_m`範囲を表示し、該当するWorking segmentをmagentaで強調します。Workingを
非表示にした状態で編集を始めると、不可視データの誤編集を避けるため先に再表示します。

MPC trajectoryの静的な壁余白確認は次の順で行います。

1. `Vehicle / Margin Settings` でoccupancy-grid YAML、車体外形、前後左右margin、unknown cell policyを確認する。
2. `Validate Clearance` で各poseの向き付き車体矩形とpoint間のswept footprintを非破壊検証する。
3. reportのissueを選び、`Center Issue` で該当点と車体／margin矩形を確認する。
4. 必要な場合だけ `Adjust Clearance` で法線方向の離散offset候補を作り、既存のBefore/Candidate previewからApplyする。
5. Apply後はgeometryは再生成済みですがspeed metadataはstaleになるため、`Recompute Speed`を実行してから保存する。

車体presetは現在の`vehicle_info.param.yaml`から導出するrear-axle基準の暫定値
（前1.554 m、後0.510 m、左右各0.650 m）で、margin既定値は0 mです。
trajectory pose基準とAWSIM colliderの一致は未確認なので、2026公式寸法や実接触保証
として扱わないでください。判定の正本候補は選択したoccupancy gridで、Lanelet2 railは
表示用です。raw minimumは離散poseでの車体矩形とunsafe cell矩形の距離です。
conservative minimumはpoint側のgrid量子化下限と、segment sweepの距離場下限・
回転膨張を含む値の最小であり、0.1m gridの約0.071mはpoint側の量子化項に限ります。
unknown cellは既定でoccupied扱い、map外は常にerrorです。final_ver3のbinary PGM、
origin yaw=0、negate=0では主要な画像前処理をC++ runtimeへ合わせますが、一般map
loaderの完全互換ではありません。
自動補正は最大shift内で安全なdetached candidateが得られた場合だけApplyでき、
地図内容・設定・document revision・candidate内容をApply直前に再確認します。
既定のsweep step 0.05m、最大shift 0.50m、offset step 0.05m、最大絶対曲率
0.70rad/mもローカル暫定値です。`INFEASIBLE`はこの有限探索内で候補が見つからなかった
意味で、連続空間に解がない証明ではありません。実コース規模のAdjustは数十秒かかる
場合があり、progress dialogの`Cancel`で協調停止できます。clearance stateはnot-run、running、
safe、unsafe、failed、staleを区別し、失敗した再検証は過去のSAFEを無効化します。
設定後のfailed/running/staleは保存不可で、SAFE保存時もmap signatureを再確認します。
これはofflineの静的検査であり、実際の壁接触は`make dev` / gate走行で別途確認が必要です。

Normalizeの`preserve`は点数・topology不変時だけ使用できます。`interpolate`はarc length上で`vx/ax`を補間します。`recompute`は補間値を一時値として入れたうえでspeedをstaleのまま維持し、保存前に`Recompute Speed`を要求します。有限XYと`vx/ax`を読めるMPC CSVなら、非単調`s_m`、不正な`s_m/psi/kappa`、重複・退化点をrepair対象として開けます。schema不正、非有限XY、不正`vx/ax`は開きません。

MPC CSVはcanonical 7列 `s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2`、Pure Pursuit CSVは既存8列を形式別に検証します。高度なNormalize/Speed機能はMPC限定で、Pure Pursuitの既存編集・quaternion再計算は維持します。保存時の暗黙再計算は行いません。stale fieldまたはvalidation errorがあれば保存を止めます。`Overwrite` は対象pathを表示して確認し、同一directoryのtemporary fileを再検証してから原子的に置換します。symlink targetの直接置換は拒否します。

`AI Challenge 2026 Candidate - Safe` の `resolution=0.25 m`、`a_max=1.0 m/s²`、read-onlyの`horizon_distance=16 m`は比較検証用のローカル候補であり、2026公式確定値ではありません。`v_max/a_min/ay_max`の初期値は現在のCSV統計から入る未検証値なので、走行条件に合わせて確認してください。candidate生成と比較model作成はprogress dialog中のworkerで行い、Apply直前にsource revision、candidate内容、validationを再確認します。

Editor内の `vx_mps/ax_mps2` はoffline CSV metadataです。現行C++ MPCの走行速度はruntime側の速度上限処理が優先されるため、Editorで値を保存しただけでは走行速度プロファイルへ直接反映されません。

Pure Pursuit 用の `simple_trajectory_generator/data/raceline_awsim_30km_from_garage.csv` を開く場合:

```bash
ros2 run multi_purpose_mpc_ros pure_pursuit_trajectory_editor
```

### V2X position editor

デバッグ用に仮想車両を地図上へ置き、`/v2x/vehicle_positions` を publish します。C++ MPC の `use_v2x_gap_planner: true` と組み合わせると、配置した車両を避ける方向へ制約が変わるか確認できます。

```bash
ros2 run multi_purpose_mpc_ros v2x_position_editor
```

左クリックで車両追加/選択、ドラッグで移動、右ドラッグまたは中ドラッグで pan、ホイールで zoom、Delete で削除します。配置は JSON で保存/読込できます。

### まとめて起動（コントローラー + シミュレーション）
```bash
ros2 launch multi_purpose_mpc_ros test.launch.xml
```

### Attribution
This repository includes code derived from:

Multi-Purpose-MPC  
Author: Mats Steinweg  
Original repository: https://github.com/matssteinweg/Multi-Purpose-MPC

Used with permission from the author.
