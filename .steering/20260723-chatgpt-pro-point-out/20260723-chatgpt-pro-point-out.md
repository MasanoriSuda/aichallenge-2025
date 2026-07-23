今日 14:56
https://github.com/MasanoriSuda/aichallenge-2025/tree/develop_julyについて自動運転AIチャレンジ2026用のレースで使用するコード及びデバッグ環境を作っています、自己位置推定、経路計画、アダプティブクルーズコントロール、車線維持支援、衝突被害軽減ブレーキ、追い越し支援、完全自動運転、レース戦略の観点から勝ち筋のコードにするため課題出しをしたいのでレビューお願いします
26m 33s考えました
総評

develop_july は、単なる経路追従コードではなく、V2X追い越し、スタートグリッド判断、低速車回避、solver fallback、壁接触からの後退復帰まで入った、かなり実戦寄りの構成です。特に、追い越し状態機械とスタック復帰まわりはよく考えられています。

一方、勝つための最大課題は機能追加ではありません。現状のボトルネックは次の3点です。

経路速度・MPC・AWSIM加速度指令の物理モデルが一致していない
AEBなどの安全機能が、追い越しやMPCと同じ巨大な制御ループに入っている
単独ラップではなく、3台予選・4台決勝を再現した自動評価が不足している

2026年SW部門は、3～4台同時、6周、10分、完走順で争い、衝突や不正加速度には速度ペナルティがあります。予選はチャレンジャー、他チーム、NPCの3台、SIM決勝は4チームです。したがって勝ち筋は、最速単独ラップよりも、DNFしないこと、追い越しを完遂すること、接触後に復帰すること、順位に応じてリスクを変えることです。なお、公式ルールは2026年7月23日時点でもWIPです。

以下は静的コードレビューです。実際のビルド、AWSIM走行、rosbag解析は未実施なので、数値閾値についてはログでの再検証が必要です。

現状の強いところ
V2X追い越しの状態設計

Cruise / Follow / Overtake / LowSpeedAvoidance / SafetyBrake が明確に分かれ、対象車両ID、追い越し側、ShiftOut・Pass・Return、solver失敗後のcooldownまで保持しています。ヘアピンで単純なXY前後判定が破綻しないよう、車両を基準経路上へ投影してコース進行方向で前後判定している点も良いです。

スタック復帰の安全設計

後退候補をoccupancy grid上でswept-footprint評価し、V2Xの完全性、ギア状態、Boost停止、停止距離、接触セル改善まで確認しています。通常の「壁に当たったら一定時間バック」よりはるかに堅牢です。

入力異常・solver失敗への備え

Odometryの受信時刻とsource stampの両方を監視し、非有限値、古いOdometry、OSQP失敗時に古い解をそのまま再生しない構造になっています。

テスト可能なpure coreへの分離

path_core、v2x_overtake_core、stuck_recovery_core、recovery_footprint、mpc_velocity_limitなどに対してGTest／pytestが用意されています。この分離方針は正しいです。

優先課題バックログ
ID	優先度	課題	完了条件
P0-01	最優先	加速度入力を前提に縦制御を作り直す	MPCまたは縦制御器が直接accelerationを決定し、加速度・減速度・jerk・遅延を同一モデルで扱う
P0-02	最優先	独立Safety Supervisor／AEBを追加する	MPC、追い越し、Recoveryに関係なく、衝突予測時に最終コマンドを上書きできる
P0-03	最優先	経路CSVと実行時経路の契約を統一する	CSVのpsi/kappa/vx/axを使うか、使わない列を契約から削除し、実行時の速度プロファイルが一意になる
P0-04	最優先	V2X Healthを統一する	source age、receipt age、欠損、jump、速度推定信頼度をHealthy/Degraded/Stale/Invalidへ集約する
P0-05	最優先	40 Hz制御のdeadlineを保証する	p99.9実行時間20 ms以下、25 ms超過ゼロ。OSQP workspace再利用と計測を導入する
P0-06	最優先	3台予選・4台決勝の自動シナリオ評価を作る	qualifying、final、Safety Gate、V2X障害、接触復帰をCIで再現できる
P0-07	高	自己位置初期化を状態機械化する	stale GNSSや誤方向でEKFを開始せず、service失敗時に再試行する
P1-01	高	Frenet座標の距離ベースtrajectory plannerを導入する	固定WP数ではなく物理距離・時間で追い越し軌道を評価する
P1-02	高	追い越しを不確実性付き占有tubeで判定する	V2X欠損や遅延時に、古いgapを安全なgapとして維持しない
P1-03	高	車両モデルと操舵遅延を同定する	AWSIM／実車別に操舵gain、遅延、横加速度限界、制動能力をログから生成する
P1-04	高	Race Strategistを分離する	Lap、順位、相対進捗、Boost、残り時間に応じてrisk budgetを変更する
P1-05	高	V2X台数のhard-codeを廃止する	予選2他車／決勝3他車をセッション情報から解決する
P1-06	中	オフラインで再現可能なビルドにする	CMake中のネットワークpip installを廃止し、依存をlock／vendor化する
1. 縦制御・速度計画が最大の構造課題
1.1 AWSIMは速度指令を使用しない

公式インターフェースでは、longitudinal.speed は未使用で、AWSIMが実際に使用するのは longitudinal.acceleration です。

しかし現状のMPCでは、入力ベクトルの第1要素を速度として

umin << 0.0, ...
umax << cfg.v_max, ...

とし、QPが速度u[0]を直接最適化しています。

その後、実際の加速度指令は

acc = 100.0 * (u[0] - actual_v);
acc = clip(acc, a_min, a_max);

という非常に高いゲインのP制御へ変換されています。

つまり実際には、

経路速度
  ↓
速度入力型MPC
  ↓
高ゲインP変換
  ↓
加速度飽和
  ↓
AWSIM

となっており、MPCが予測した速度応答と、AWSIMへ入る加速度応答が一致しません。この不一致はACC、AEB、コーナー進入速度、追い越し時のclosing speedのすべてへ波及します。

改善案

縦方向を少なくとも次のモデルへ変更します。

state:
  v
  a_cmd_filtered       # 必要ならアクチュエータ遅れ

control:
  acceleration
  または jerk

constraints:
  a_brake_eff <= a <= a_accel_eff
  |jerk| <= jerk_max
  v <= v_path_limit

横MPCと縦制御を分ける場合でも、縦制御器は次を同じ物理量で処理すべきです。

v_path_limit
v_acc_limit
v_aeb_limit
v_overtake_limit
v_race_strategy_limit

最終目標速度は最小値を採用し、そこから実現可能な加速度列を求めます。

1.2 CSVの速度・加速度情報が実行時の単一情報源になっていない

strict loaderはpsi_rad、kappa_radpm、vx_mps、ax_mps2を含む7列を検証しています。これは良い設計です。

ところが実行時のload_ref_path()は、読み込んだ点からx_mとy_mだけを取り出しています。CSVに保存されたheading、curvature、velocity、accelerationはそのままReferencePathへ渡されません。

さらに制御ループでは毎周期、

reference_path_->set_v_ref(
  std::vector<double>(
    reference_path_->waypoints.size(),
    effective_v_max));

と全waypointの速度を一定値へ上書きしています。

このため、trajectory editorで作った速度プロファイルが、レース実行時の制御に反映されていない可能性が高いです。

改善方針

次のどちらかに統一してください。

方式A: CSVを正式な走行プロファイルとする

x, y, psi, kappa, v_ref, a_ref

をそのまま読み込み、オンラインでは安全・ACC・戦略上限のみを掛ける。

方式B: CSVはgeometryだけとする

s, x, y

だけを保存し、起動時に曲率と速度プロファイルを一意に生成する。

現状の「7列を厳密に検証するが、実行時はXY中心」という中間状態は、調整結果の追跡を難しくします。

1.3 compute_speed_profile() の式は次元が合っていない

現在の加速度制約は実質、

(v[i + 1] - v[i]) / (2 * delta_s)

をa_minとa_maxの間へ制限しています。

速度がm/s、距離がmなので、この式の単位は1/sであり、m/s²ではありません。正しい空間領域の関係は、

a=
2Δs
v
i+1
2
	​

−v
i
2
	​

	​


です。

実装はz=v²で行うと安定します。

z[i] = v[i]^2

z[i+1] <= z[i] + 2 * a_accel * ds
z[i]   <= z[i+1] + 2 * a_brake * ds

z[i] <= ay_max / max(abs(kappa[i]), epsilon)
v[i] = sqrt(z[i])

前向きpassで加速制約、後ろ向きpassで制動制約を入れれば、QPを使わなくても高速に生成できます。

2. ACC・AEB
ACC

現状には、前車速度、closing margin、目標距離、カーブ内guard、required decelerationによる速度capなど、ACCに必要な部品はかなり揃っています。コース進行方向上で前車を判定しているのも適切です。

問題は、前車速度がV2Xの直近2点から推定される一方、その信頼度がACC目標へ明示的に入っていないことです。V2Xメッセージ自体には速度がなく、異常ジャンプ時は静止物として扱う設計です。

ACC目標は次の形へ整理するとよいです。

desired_gap =
    body_length_clearance
  + standstill_gap
  + ego_speed * time_headway
  + source_age_uncertainty
  + relative_speed_uncertainty

v_target =
    front_speed_estimate
  + distance_error_gain * (gap - desired_gap)

front_speed_estimateの信頼度が低い場合は、単純に0 m/sとみなすのではなく、占有範囲を広げます。

AEBはMPCから独立させるべき

現在のSafetyBrakeはV2X behavior FSMとMPC問題生成の中に組み込まれています。これは通常走行では機能しますが、次の状況で弱くなります。

QP生成・solveが遅い
追い越しFSMが古いtargetをholdしている
gap plannerの境界が不整合になる
Recoveryとの状態競合が起こる
V2Xのsource ageとreceipt ageが食い違う

AEBは、MPC出力後に最終指令を調停する独立Safety Supervisorへ置くべきです。

nominal controller command
             │
             ▼
┌──────────────────────────┐
│ Independent Safety Arbiter│
│ - AEB                     │
│ - lane boundary invariant │
│ - stale state stop        │
│ - command finite/rate     │
└──────────────────────────┘
             │
             ▼
       AWSIM / vehicle

AEBの安全距離は、例えば次で統一できます。

d
safe
	​

=
2
L
ego
	​

+L
target
	​

	​

+v
ego
	​

τ
total
	​

+max(0,
2a
brake,eff
	​

v
ego
2
	​

−v
target
2
	​

	​

)+k
σ
	​

σ
s
	​

+d
margin
	​


ここでa_brake_effは設定値ではなく、AWSIM／実車ログから得た確実に出せる減速度の下限にします。

現在の閾値は攻めすぎ

設定では、moving frontのhard distanceが中心間2.05 mです。車両全長は2.0 mなので、同じ向きならバンパー間は約0.05 mしかありません。これは遅延、姿勢差、V2X誤差を吸収できるhard floorではありません。

また、front-riskのEmergency判定はrequired deceleration 10 m/s²ですが、通常指令の制動下限は-1.35 m/s²です。別途停止距離gateもありますが、risk levelと実際の制動能力が異なる物理モデルになっています。

Comfort / Hard / Emergencyは、固定値ではなく、測定したa_brake_effに対する割合で設定するのが安全です。

3. 車線維持・横制御
現状
40 Hz制御
N=20
kinematic bicycle model
steering gain 1.5
state prediction delay 0
accel／steer low-pass gain 1.0
waypoint preview offset 2
safety margin scale 0

という攻撃的な設定です。

AWSIMは四輪のwheel colliderによる物理モデルであり、等価二輪モデルそのものではありません。したがって、速度が上がると操舵遅れ、飽和、yaw response、横滑りにモデル誤差が出ます。

課題
固定WP数で物理horizonが変動する

内部補間は現状legacy floor方式で、固定Nを維持するためにceil化を保留しています。つまり点間隔によってMPCが見る実距離が変わります。

追い越し、カーブ進入、AEBは「何点先」ではなく、「何m先」「何秒先」で判断すべきです。

state predictionとinput previewが別々

state_prediction_delay_sec=0.0のまま、入力だけwaypoint offset 2で先読みしています。これは調整上は速く見えても、状態と入力参照の位相が一致しません。

改善案
現在位置を最近傍点ではなく、最近傍segmentへ射影
進捗を連続Frenet座標sとして管理
horizonを30 mまたは2.0 sなどの距離・時間基準にする
操舵step responseから以下を同定
command-to-steering delay
steering gain
steering rate limit
yaw-rate gain
速度別横加速度限界
コース境界に対するSafety Supervisorを追加

最低限、横制御は次のログを毎周期残すべきです。

e_y
e_psi
kappa_ref
delta_ff
delta_fb
delta_command
delta_measured
yaw_rate_predicted
yaw_rate_measured
distance_to_left_wall
distance_to_right_wall
4. 自己位置推定
良い点

GNSS poseのcovarianceを補正し、orientationが無効ならIMUをfallbackとして使い、基準経路の向きから初期yawを生成しています。MPC側でもOdometryの受信停止とsource stamp停止を分けて監視しています。

修正すべき点
Covarianceの入力検証が不足

現在は値がthreshold以下ならgood covarianceへ変換します。負のcovarianceもgood扱いになり、NaNやInfを明示的にrejectしていません。

if (!std::isfinite(v) || v < 0.0) {
    reject_or_mark_degraded();
}

が必要です。

Quaternionの検証が不足

現在はNaNまたは全要素0のみをfallback条件にしています。Inf、norm異常、古いIMU、GNSSとIMUの時刻差は確認されていません。

最低限、

all finite
norm within [0.95, 1.05]
imu age < threshold
abs(gnss_stamp - imu_stamp) < threshold

を必要条件にします。

EKF triggerが失敗しても再試行されない

非同期serviceを投げた直後にekf_triggered_=trueにしているため、responseがsuccess=falseでも、その後のGNSS callbackでは再試行されません。callback内のfuture.get()も例外処理されていません。

次の状態機械に変更すべきです。

WAIT_SENSOR
  → INITIAL_POSE_READY
  → TRIGGER_REQUESTED
  → TRIGGER_CONFIRMED
  → LOCALIZATION_HEALTHY

success=trueを受け取るまでTRIGGER_CONFIRMEDへ進めません。

最新GNSSにfreshness条件がない

/set_initial_poseは保存済みの最新GNSSが存在するかだけを確認し、その測定時刻が古くないかを確認していません。

ヘアピンで初期方向を誤る可能性

初期yawは最近傍点を選び、その点以降または以前のsegment tangentを使用します。segmentへの射影、最大距離、候補branch比較、車両の初期向きとの整合性はありません。

ヘアピンの近接する往路・復路で間違ったbranchを選ぶ可能性があります。候補segmentごとに、

position distance
heading difference
expected start grid range
course continuity

をscore化してください。

固定0.3秒のsimulation delay

launchではsimulation pose delayが暫定0.3秒です。コメントでも実測値ではないと明記されています。

GNSS pose stampとEKF output、vehicle ground truth相当のログを使い、相互相関から遅延を同定するべきです。

5. V2Xと追い越し
V2X tracker自体はかなり良い

source stampとreceipt stampを分離し、header stampがある場合はsource ageを使って外挿し、位置jumpや不正速度を検出しています。freshnessはreceipt ageで管理しています。

課題は、これらが最終的に一つの健全性契約になっていない点です。

enum class TrackHealth {
  Healthy,
  DegradedNoSourceStamp,
  DegradedVelocityUnknown,
  Stale,
  PositionJump,
  IncompleteRoster,
  Invalid
};

のように統合し、各plannerが同じ判断を使うべきです。

現状の追い越し設定は速いが脆い

現在の設定には次の攻撃的な値があります。

膨張後の残余gap: 0.2 m
lateral acceleration guard: 6.0 m/s²
active gap loss hold: 2.0 s
hard curve内での継続許可
safety margin scale: 0
wall clearance: 0.8 m
追い越しline offset: 1.2 m

この設定は、シナリオに合えば速いです。ただし、gapが消えた後も2秒間locked lineを維持するため、V2X dropoutと実際の相手の横移動を区別できない場合に危険です。

推奨する追い越し判定

横方向の一点gapではなく、Frenet座標上で相手の占有tubeを作ります。

target_s(t)  ± longitudinal_uncertainty(t)
target_ey(t) ± lateral_uncertainty(t)

不確実性は時間とともに拡大させます。

sigma_s(t)  = sigma_s0  + velocity_sigma * t + source_age_margin
sigma_ey(t) = sigma_ey0 + lateral_motion_bound * t

各候補軌道に対して、

track corridor feasible
vehicle separation feasible
lateral acceleration feasible
steering rate feasible
required longitudinal deceleration feasible
hard-curve completion feasible

を判定します。

abort条件を明文化する

追い越しFSMに「開始条件」は大量にありますが、勝負で重要なのは、コミット後にどこで諦めるかです。

少なくとも次を不変条件にします。

V2X health is Healthy or explicitly tolerated Degraded
predicted clearance >= minimum
required deceleration <= achievable braking reserve
track corridor remains feasible
solver deadline is met
target does not intrude selected-side ordering

1つでも破れたら、

Pass → AbortHold → Return

へ遷移し、反対側へ即座に切り返さないようにします。

6. 完全自動運転・復帰

現在のRecoveryは非常に充実していますが、制御ノードの中へ多くの責務が集中しています。mpc_controller_cpp.cppがReferencePath、MPC、V2X、追い越し、Recovery、ROS I/O、Boostを持ち、最後はSingleThreadedExecutorで実行されています。

完全自動運転としては、上位にLifecycle Supervisorを置くべきです。

BOOT
 → WAIT_SENSORS
 → LOCALIZING
 → READY
 → RACING
 → FINISHING
 → STOPPED

RACING
 → DEGRADED
 → RECOVERY
 → RACING

any state
 → SAFE_STOP

各状態のentry条件とtimeoutを明示します。

V2X台数hard-code

Recoveryの期待V2X台数は現在2で、READMEでも4台構成では3へ変更する必要があるとされています。

これはprofile切替忘れでRecoveryが永久停止するタイプの不具合です。次のいずれかへ変更してください。

セッション設定から参加台数を取得
一定時間観測したvehicle ID集合をrosterとして確定
qualifying／final profileを起動時に明示し、矛盾なら起動失敗

通常走行の安全判断と、後退時の完全情報要求は分けても構いません。

7. Race Strategist

現在の実装にはrace strategyの材料はありますが、戦略層としてまとまっていません。

AWSIM statusからは、残り時間、周回数、現在lap time、section、残りBoost、Boost中フラグを取得できます。Boostは0.5 m/s²が10秒継続します。

Boost設定にdomain依存の非対称がある

現在は、

awsim_boost:
  enabled: true
  domain_enabled:
    1: false
    2: false
    3: false

です。未列挙domainはdefaultのenabled: trueを引き継ぐため、明示的overrideがなければDomain 4だけBoostが有効になる可能性があります。

4台決勝でこれはかなり危険な設定差です。ROS_DOMAIN_IDは戦略やグリッド位置ではなく、単なる車両通信domainとして扱うべきです。

推奨戦略状態
Launch
Cruise
Attack
FollowAndPrepare
Defend
Conserve
LastLapAttack
FinishSafe

Race Strategistが変更してよいのは、あくまで次のような目的関数とsoft制約です。

preferred pass side
desired time headway
overtake benefit threshold
boost request
maximum tactical risk
desired speed bias

AEB、lane boundary、最大加速度などのhard safety constraintは変更できないようにします。

Boost使用条件

単純なfirst_forward_motionではなく、予測利益で決定します。

boost_gain =
    expected_distance_gain
  + overtake_completion_gain
  - braking_loss_before_next_corner
  - collision_risk_penalty

候補は、

低速コーナー立ち上がり
長い全開区間
ShiftOut完了後のPass
最終周の順位逆転
前車との相対速度が足りない場面

です。コーナー直前やAEB余裕が小さい場合は使用しません。

また、公式ルールでは順位による加速度・速度handicapが入る可能性があります。オンラインで実加速度を推定し、現在の車両能力を更新するCapabilityEstimatorがあると戦略精度が上がります。

8. リアルタイム性能
現状のリスク

制御周期は40 Hzなので、1周期は25 msです。

一方、solve_osqp()は呼び出しごとに、

Eigen sparse matrixをCSC配列へコピー
OSQPSettings作成
osqp_setup
osqp_solve
osqp_cleanup

を行っています。

さらにSingleThreadedExecutor上でOdometry、V2X、status、gear、marker、trajectory、Recovery評価を処理しています。周期超過が起きると、Odometry処理遅延、V2X遅延、solver遅延が同じスレッド内で連鎖します。

改善案
OSQP workspaceを保持
osqp_update_*でq、bounds、matrix valuesのみ更新
前周期解でwarm start
matrix sparsity patternを固定
marker生成を低優先executorへ分離
V2X tracker更新とcontrolをcallback group分離
Safety Supervisorは制御ループと独立
毎周期のstage timingをpublish

記録すべき時間は次です。

association_us
world_model_us
behavior_us
trajectory_us
qp_build_us
qp_solve_us
safety_us
publish_us
total_us
deadline_missed
仮の受入基準

40 Hzの場合、チーム内基準として次が妥当です。

p95 total       < 12 ms
p99 total       < 17 ms
p99.9 total     < 20 ms
hard maximum    < 25 ms
deadline miss   = 0 in acceptance race suite
9. デバッグ・評価環境
現在のmake evalだけではレース性能を評価できない

公式ガイドも、ローカル評価は単独タイムアタックであり、本大会は複数台レースであると明記しています。

リポジトリのeval.shも1台、NPCなし、handicapなし、rankingなしです。

一方、dev3は3つのAutoware domainを起動し、dev.shではNPCを0にしています。これは予選の「自チーム＋他チーム＋NPC」とは異なるシナリオです。

作るべきシナリオ行列

公式シミュレータはscenario YAML、vehicle poses、過去結果のreplayをサポートしています。これをCIの中心に据えるべきです。

Suite	シナリオ
Qualifying	2チーム車両＋NPC、全スタート位置
Final	4チーム、全Domain、内外グリッド
Safety Gate	障害物停止、NPC追い越し、車線維持
ACC/AEB	前車急減速、停止車、低速車、cut-in相当
Overtake	直線、soft curve、hairpin手前、side-by-side
V2X Fault	dropout、遅延、out-of-order、同一stamp、position jump
Localization	GNSS遅延、covariance異常、IMU stale、誤branch初期化
Recovery	前壁、後壁、横壁、車列接触、V2X欠損
Runtime	6周、4台、ログ負荷あり／なし、連続solver stress
Strategy	Boost残数、handicap変動、最終周順位争い

同じシナリオをDomain 1～4で実行し、domain依存設定を検出してください。

性能計測とデバッグ走行を分ける

現在の評価launchはcapture、rosbag、RVizをすべて有効にしています。

これはデバッグには便利ですが、WCET測定にはノイズになります。

debug mode:
  RViz on
  capture on
  full rosbag
  verbose diagnostics

performance mode:
  headless
  RViz off
  capture off
  selected-topic rosbag
  CPU affinity fixed
  release build

の2系統を用意してください。

自動判定メトリクス

公式出力にはresult JSON、rosbag、motion analyticsがあります。

チーム内の仮基準として、次を自動判定すると有効です。

項目	仮の合格基準
Safety Gate	全決定論シナリオで衝突・逸脱0
AEB	定義した速度・距離範囲で停止成功100%
V2X stale	stale情報をclear corridorとして使用0回
Overtake feasible	成功率95%以上
Overtake infeasible	安全abort率100%
Localization	横誤差p95 < 0.15 m、yaw誤差p95 < 1.5°
Runtime	p99.9 < 20 ms、deadline miss 0
Race	DNF率、衝突率、中央値順位、最悪順位を集計
Recovery	復帰可能ケース成功率、復帰不能ケース誤作動0

ラップタイムだけでなく、DNF率と順位分布を主KPIにすべきです。

10. ビルド・CI

CMakeには多くのテストがあり良い一方、configure/build中に仮想環境を作り、pip install -r requirements.txtを実行しています。ネットワーク制限、PyPI更新、依存削除によって提出ビルドが壊れる可能性があります。

次へ変更するのが安全です。

requirements.lock
  - exact version
  - hash pinned

vendor/wheels/
  - evaluation imageで必要なwheelを同梱

CI:
  - network disabled build
  - clean Docker build
  - colcon test
  - reference path validation
  - scenario regression
  - runtime budget check

また、CMakeコメントは「C++23」ですが、実際にはC++17を設定しています。小さい点ですが、提出環境との差分を減らすため修正しておくべきです。

推奨アーキテクチャ
GNSS / IMU / Wheel / Steer
             │
             ▼
 Localization + LocalizationHealth
             │
             ├──────────────────────┐
             ▼                      │
       Frenet World Model           │
             ▲                      │
             │                      │
 V2X Tracker + V2XHealth            │
             │                      │
             ▼                      │
        Race Strategist             │
  lap / rank / boost / risk budget  │
             │                      │
             ▼                      │
        Behavior Planner            │
 Cruise / Follow / Pass / Abort     │
             │                      │
             ▼                      │
       Trajectory Planner           │
 corridor + speed profile + tube    │
             │                      │
        ┌────┴────┐                 │
        ▼         ▼                 │
  Lateral MPC  Longitudinal MPC/PID │
        └────┬────┘                 │
             ▼                      │
       Nominal command              │
             │                      │
             ▼                      │
 Independent Safety Supervisor ◄────┘
 AEB / lane invariant / stale stop
             │
             ▼
       Command Publisher

重要なのは、Race StrategistやOvertake PlannerがSafety Supervisorを迂回できないことです。

実装順序
第1段階: 事故と不安定性を止める
acceleration入力に一致する縦制御へ変更
独立AEB／command arbiter追加
V2X Health統一
OSQP時間計測とworkspace再利用
localization初期化状態機械
qualifying／finalのscenario runner作成
第2段階: 追い越し成功率を上げる
segment projectionによる連続s
距離・時間ベースhorizon
不確実性付き他車占有tube
overtake commit／abort invariant
lateral／longitudinal trajectoryの同時実現可能性判定
AWSIMで操舵・制動能力を同定
第3段階: 順位を取りに行く
Race Strategist
Boost最適化
Lap／順位別risk budget
handicap下でのonline capability推定
opponent replayを使ったMonte Carlo評価
最悪順位・DNF率を含むパラメータ最適化
最終評価

現状は、機能量だけなら十分に戦えるコードです。むしろ、これ以上FSM条件を増やすと、相互作用と調整不能性が勝ち筋を潰します。

最優先で直すべきなのは次の順です。

1. 縦制御の物理的一貫性
2. 独立AEB／Safety Supervisor
3. 3台・4台の自動回帰試験
4. V2Xの不確実性管理
5. 40 Hz deadline保証
6. 戦略層と制御層の分離

特にP0-01の縦制御を直すと、単独ラップ、ACC、AEB、コーナー進入、追い越しclosing speed、Boost判断が同じ物理モデルに揃います。最初に切るGitHub Issueは、**「CSV速度契約の統一とacceleration-native longitudinal controller導入」**が最も費用対効果の高い一手です。

ちなみにいまは予選なのでシミュレータで動かしています、実車のような厳しい環境は考えずawsim環境と相手v2xだけ考えればいいです、上位の対戦相手も横gはちゃめちゃでたぶん通してます、繰り返しますがこれは予選用コードです
7m 46s考えました

了解です。前回は実車・本戦まで見越して保守側へ振りすぎました。予選のAWSIM専用コードなら、評価軸を次へ切り替えるべきです。

完走できる最低限の衝突回避を残しつつ、AWSIMで通る最大速度・最大横加速度・最短追い越し時間を狙う。

したがって、前回挙げた「汎用Safety Supervisor」「実車向けセンサ健全性」「大規模な縦制御再設計」「汎用Lifecycle化」は、予選前の優先課題から外します。既存の攻撃的な設定も、AWSIM予選という前提なら合理的です。

修正版の結論

現在の勝ち筋は、次の順です。

自分で掛けている速度・Boost制限を外す
AWSIMの操舵モデルとMPC内部モデルの不一致を潰す
コーナー速度と追い越し横移動のソフトウェア上限を上げる
V2X約1 Hz前提で追い越し状態を安定させる
予選環境と完全一致した3台レースでパラメータ探索する
スタート、追い越し、Boostだけに戦略を絞る

ACCやAEBは「快適性」や「実車相当の安全余裕」ではなく、接触ペナルティやスタックでタイムを失わない最低限のガードとして扱えば十分です。

予選向けP0課題
P0-1. 自分で掛けているハンディキャップを確認する
Boostが既知Domainですべて無効

現在の設定では、トップレベルのenabled: trueに対して、Domain 1～3がすべてfalseです。

awsim_boost:
  enabled: true
  domain_enabled:
    1: false
    2: false
    3: false

ローカルのdev.shではBoost数が2に設定されています。

予選環境でBoostが使用可能で、オンライン実行Domainが1～3のどれかなら、現状はBoostを一度も使わずに走っています。相手が使っている場合、横G以前にここで負けます。

Domain 2だけスタート後15秒間37 km/h

現在は通常40 km/hに対して、Domain 2だけスタート後15秒間37 km/hです。

v_max: 40.0

domain_v_max:
  1: 40.0
  2: 40.0
  3: 40.0

domain_start_v_max_duration: 15.0
domain_start_v_max:
  2: 37.0

これは「内側グリッド有利を補正する」とコメントされていますが、勝つことだけを考えるなら自発的に公平化する理由はありません。オンライン予選で自車がDomain 2なら、最初の15秒を3 km/h落としているため、まず無効化候補です。

Issue案
[Qualifier][P0] Remove domain-specific self-handicap

- 予選実行時のROS_DOMAIN_IDを確定する
- Boost使用可否を確認し、使用可能なら該当Domainを有効化する
- domain_start_v_maxによる自己速度制限を廃止する
- 起動時ログに Domain / Boost enabled / effective v_max を必ず表示する
P0-2. AWSIM予選とローカル環境の差をなくす

現在の複数台devは、

--wall-recovery on
--ranking off
--handicap off
--laps unlimited

です。

一方、単独evalでは、

--wall-recovery off
--laps 6

です。

特にwall-recoveryの差は重大です。

dev3: AWSIM標準wall recoveryあり
eval: AWSIM標準wall recoveryなし
コード側: 独自stuck recoveryあり

これでは、ローカル3台走行でAWSIM標準復帰に助けられているのか、独自Recoveryが機能しているのか切り分けられません。

予選で使用される設定と同じフラグを、1つの専用スクリプトに固定してください。

simulator_scripts/qualifier.sh

に次を明示します。

vehicles
npcs
boosts
laps
timeout
wall-recovery
collisions
start-mode
V2X更新条件
完了条件
make qualifier

の1コマンドで、オンライン予選と同じ台数・周回数・復帰条件・Boost条件が起動することです。

P0-3. 操舵gain 1.5を「AWSIM向け性能ハック」として正しくモデルへ入れる

現在のコードでは、MPC内部の操舵とAWSIMへpublishする操舵に差があります。

起動時にsteer rateをgainで割り、

最終publish時には操舵角をgain倍しています。

final_command.lateral.steering_tire_angle *=
  mpc_cfg_.steering_tire_angle_gain_var;

一方、内部BicycleModel更新にはgain前のu[1]を使っています。

car_->drive(Eigen::Vector2d(actual_v, u[1]));

設定値は1.5です。

つまりコード上は概ね、

MPC予測操舵      = δ
AWSIM投入操舵    = 1.5 δ
内部状態更新操舵 = δ

となります。

これはAWSIMで曲がるための有効なハックかもしれませんが、予測より実車両が1.5倍強く旋回するモデル不一致になります。コーナーでうまく走れている間は速いものの、追い越しShiftOutや壁際ではoscillation、overshoot、solver failureの原因になります。

予選向けにはgainを1.0に戻す必要はありません。むしろ次のどちらかに統一するべきです。

案A: MPCモデルへgainを含める
const double effective_delta =
  steering_tire_angle_gain_var * delta_command;

をBicycleModelと予測に使う。

案B: MPC出力側へ吸収する

MPCがAWSIMへ入る最終tire angleを直接最適化し、publish時には倍率を掛けない。

最小の実験

20、30、40 km/hで一定操舵を入れ、

command steering
actual yaw rate
actual path curvature = yaw_rate / speed
predicted curvature = tan(delta_model) / wheelbase

を比較します。

予測曲率とAWSIM実測曲率の誤差を10%以下にすることを受入条件にすれば十分です。実車用の厳密な車両同定までは不要です。

予選で最も効く横G・追い越し調整
現在の通常走行はかなり攻めている

設定はすでに、

ay_max: 30.0
safety_margin_scale: 0.0
v2x_front_risk_emergency_decel: 10.0
v2x_overtake_front_velocity_limit_enabled: false
v2x_overtake_hard_curve_entry_enabled: true

です。

この方向性は予選AWSIM用として正しいです。前回のように安全側へ戻す必要はありません。

ただし、通常走行のay_max=30に対して、追い越し専用lineは横加速度6.0で制限されています。

v2x_overtake_line_max_lateral_accel: 6.0
v2x_overtake_guard_max_lateral_accel: 6.0

つまり単独走行は30 m/s²を許すのに、追い越し横移動は6 m/s²までです。相手がはちゃめちゃな横Gで通しているなら、ここが明確な予選用ボトルネック候補です。

横Gの探索範囲

一気に30へ固定するより、次をA/Bします。

パラメータ	現在	探索候補
overtake_line_max_lateral_accel	6.0	10 / 15 / 20 / 30
overtake_guard_max_lateral_accel	6.0	line値と同じ
overtake_line_shift_distance	4.0 m	3.0 / 2.5 / 2.0
overtake_line_max_target_change	0.04 m/cycle	0.06 / 0.08 / 0.12
overtake_guard_min_front_distance	5.0 m	4.0 / 3.5 / 3.0
overtake_line_min_wall_clearance	0.8 m	0.65 / 0.55 / 0.45

現在の追い越しline設定はこちらです。

特に効く組み合わせ

まずは、

shift_distance = 3.0
max_lateral_accel = 12.0
guard_max_lateral_accel = 12.0
max_target_change = 0.08

を第1候補にします。

これで壁接触やsolver failureが増えなければ、

2.5 m / 20.0 m/s² / 0.12 m per cycle

へ進めます。

max_target_change=0.04 m/cycleは40 Hzなら目標値だけで1.6 m/s相当の横移動速度です。追い越しlineの立ち上がりを早くするには、横加速度上限だけでなく、このtarget slew制限も同時に上げる必要があります。

通常コーナー速度

40 km/hは約11.1 m/sです。ay_max=30 m/s²が40 km/hで非拘束になるのは、

κ≤
11.1
2
30
	​

≈0.243 m
−1

半径では約4.1 m以上のコーナーです。

したがって、trajectory上の最大曲率が0.30 m⁻¹なら、

v
max
	​

=
30/0.30
	​

=10.0 m/s

なので約36 km/hまで落ちます。

上位相手が40 km/h近くで通しているなら、必要なay_maxは、

a
y,required
	​

=v
max
2
	​

max∣κ∣

で求められます。

予選用なら、trajectory CSVから最大曲率を出し、

ay_max = 40 km/hを最大曲率で通す値

まで上げて、AWSIMの物理衝突や壁接触だけを実質的な上限にする案があります。

ただし、現在は操舵gain不一致があるため、先にgainをモデルへ反映してからay_maxを上げるほうが、パラメータの意味が明確になります。

経路計画は「正しい速度プロファイル」より「早く曲がれる先読み」

前回は速度CSVを正式に扱うべきと書きましたが、予選限定なら話は変わります。

現在、制御ループで全waypointの速度を毎周期effective_v_maxへ上書きしています。

reference_path_->set_v_ref(
  std::vector<double>(
    reference_path_->waypoints.size(),
    effective_v_max));

これはAWSIM予選では必ずしも悪くありません。

基本は常時最高速
必要な場所だけ動的に減速

という攻撃的な戦略だからです。

問題は、コーナー速度制限が前周期の操舵列から得たkappa_predを使うことです。

まだ操舵を切り始めていないコーナー手前では、kappa_predが小さく、減速開始が遅れる可能性があります。

大規模な縦MPCへの作り直しより、予選前には次の小変更が効果的です。

max_kappa =
  max(
    predicted_control_curvature,
    upcoming_reference_path_curvature);

つまり、

前周期の操舵予測
＋
経路上の先読み曲率

の大きい方で速度上限を決めます。

これなら、

直線では40 km/h
コーナー直前まで速度維持
必要な位置だけ減速
コーナー出口で即加速

という予選向け挙動になります。

固定WP数ではなく、例えば次の距離窓を使うとよいです。

現在から10～20 m先までのmax curvature
ACCは「追走」ではなく「追い越し準備」に限定する

現状はすでに、

前車検出は24 m
汎用Follow速度capは5 m以内
Overtake中のfront velocity limitは無効
Follow時に0.9 m横へpreposition
moving hard distanceは中心間2.05 m

となっています。

予選用途としてはかなり正しいです。

中心間2.05 mは、車長2.0 m前提なら前後クリアランス約5 cmなので、これ以上詰める必要はほぼありません。詰めすぎて接触・減速・Recoveryになるほうが遅いです。

ACCで追加するならcomfort制御ではなく、

Pass sideが見つからない:
  前車速度 + 小さいclosing margin

Pass sideが確定:
  横移動完了までclosing speed制限

横分離完了:
  ただちにfront cap解除

だけで十分です。この構造はすでにかなり実装されています。

AEB・衝突被害軽減ブレーキ

予選用なら独立した重厚なAEBは不要です。既存のSafetyBrakeを、接触直前の最後の保険として残せばよいです。

現在の設定は、

v2x_safety_brake_distance: 2.2
v2x_moving_safety_brake_distance: 2.2
v2x_moving_follow_hard_distance: 2.05

です。

これらはかなり攻めています。実車向けの余裕へ増やす必要はありません。

ただし次だけは残すべきです。

・前方車と横方向に重なっている
・距離が停止可能距離以下
・追い越しlineによる横分離がまだ完了していない

ならSafetyBrakeです。

横分離完了後は、現在のfront-overlap exclusion latchのように解除します。これにより「横に並んだのにブレーキを踏む」損失を避けられます。

V2Xはfail-closedではなく短時間fail-operational

予選ではLiDARやカメラを使わず、相手情報がV2Xだけなら、V2Xが一瞬欠けるたびに停止する戦略は負けます。

現在は、

v2x_timeout_sec: 1.0
v2x_overtake_target_hold_sec: 0.75
v2x_overtake_active_gap_loss_hold_sec: 2.0

です。

リポジトリコメントも、V2Xが約1 Hzで更新される前提を置いています。

timeout_sec=1.0は、1 Hzに少しでもジッタがあると車両がactive setから消えるため、境界値として脆いです。

予選用の第一候補は、

v2x_timeout_sec: 1.3

です。

さらに、

0～1.3 s:
  前回位置＋推定速度で外挿

1.3～2.0 s:
  新規追い越しは禁止
  進行中のPassは同一sideを維持してReturn可能位置を探す

2.0 s超:
  staleとして追い越し中止

とすると、短いdropoutでFollowへ落ちたり左右へ振れたりしません。

active_gap_loss_hold=2.0 sは、実測されたヘアピンgap dropout対策としてコメントされているため、ログなしで短くする必要はありません。むしろ、

position jumpなし
target intrusionなし
前方緊急リスクなし
wall corridor成立

なら2秒維持する現在の方向が、予選では有利です。

スタート戦略

現在はスタート時に、

最長1.25秒観測
0.4秒peer motion観測
0.2秒candidate安定待ち
その間はbase line上で加速継続

という構造です。

AWSIMでV2Xが安定しているなら、ここはもう少し速くできます。

パラメータ	現在	予選探索
motion observation	0.40 s	0.15 / 0.25 / 0.40
max observation	1.25 s	0.50 / 0.75 / 1.00
candidate stable	0.20 s	0.05 / 0.10 / 0.20

スタート直後の1秒は順位への影響が大きいため、予選では追い越し通常部より先にA/Bする価値があります。

特にBoostを使う場合、

Boost投入
→ 0.2秒観測
→ corridor lock
→ ShiftOut

の順序を固定するか、

corridor確定後にBoost

とするかで接触率が変わります。

低速・停止車回避

現在の低速回避は、

shift_velocity: 3.0 m/s
pass_control_velocity: 6.0 m/s
rejoin_control_velocity: 4.0 m/s

です。

停止車回避用としてはかなり保守的です。通常最高速は約11.1 m/sなので、停止車列を見つけた瞬間に3 m/sまで落ちると大幅に損します。

予選向けには次を探索対象にします。

shift: 3 → 5 → 7 m/s
pass:  6 → 8 → 10 m/s
rejoin:4 → 6 → 8 m/s

ただし、これはlow_speed_shift_control()のP制御を直接使うため、速度だけを上げると横振動しやすくなります。

したがって、

shift_velocity
lateral_gain
heading_gain
steering rate

をセットで探索します。

予選用に優先度を落とす課題

以下は、少なくとも予選コードでは後回しでよいです。

前回の提案	予選での扱い
実車対応の独立Safety Supervisor	後回し
実車用GNSS／IMU異常網羅	後回し
汎用的な完全自動運転Lifecycle	後回し
動的障害物全般への対応	不要、相手V2Xだけでよい
カメラ／LiDAR fallback	不要
comfort ACC	不要
jerk最適化	タイムに効かなければ不要
大規模MPCC化	予選前はリスクが高い
実車向け横加速度制約	不要
4台決勝用V2X roster一般化	予選後でよい

縦制御も、現在の

acc = clip(100 * speed_error, a_min, a_max)

が実質bang-bang制御として速く、コーナー進入で破綻していないなら、予選前に全面改修する必要はありません。優先すべきは、どこで加速を止めるかという曲率先読みです。

改訂版・予選課題リスト
## Q-P0-01 Qualifier simulator parity
オンライン予選と同じvehicles/NPC/Boost/laps/wall-recovery設定を
qualifier.shへ固定する。

## Q-P0-02 Remove self-handicap
BoostのDomain 1～3 falseと、Domain 2の15秒37km/h制限を見直す。

## Q-P0-03 Steering model consistency
steering_tire_angle_gain_var=1.5をBicycleModel予測にも反映し、
予測曲率とAWSIM実測曲率の誤差を10%以下にする。

## Q-P0-04 Aggressive corner speed
trajectory最大曲率から40km/h通過に必要なay_maxを算出し、
30/40/50/無制限相当をA/Bする。

## Q-P0-05 Aggressive OvertakeLine
shift_distance、max_lateral_accel、max_target_change、
min_front_distance、wall_clearanceを一括探索する。

## Q-P0-06 Curvature-preview braking
前周期の操舵予測だけでなく、10～20m先の経路曲率を速度上限へ使う。

## Q-P0-07 V2X jitter tolerance
timeoutを1.0/1.2/1.3/1.5秒で比較し、
Follow/Overtakeのチャタリングと接触率を評価する。

## Q-P0-08 Start-grid optimization
観測時間とcandidate安定時間を短縮し、
スタート5秒後の順位または進行距離を最大化する。

## Q-P1-01 Faster stopped-vehicle bypass
Shift/Pass/Rejoin速度を上げ、停止車列通過時間を短縮する。

## Q-P1-02 Boost strategy
スタート使用、追い越し使用、コーナー出口使用の3方式を比較する。
評価指標

単独lap timeだけではなく、各パラメータ候補について次を出してください。

finish_position
total_race_time
best_lap
median_lap
distance_after_start_5s
overtake_attempt_count
overtake_success_count
overtake_completion_time
time_spent_in_follow
time_spent_in_safety_brake
collision_count
wall_contact_count
solver_failure_count
recovery_count
minimum_vehicle_clearance
minimum_wall_clearance
control_deadline_miss

予選用の採用基準は、かなり単純でよいです。

1. 衝突またはDNFがない
2. 順位が良い
3. 同順位なら総時間が短い
4. 同程度なら追い越し成功率が高い
予選版の最終優先順位
順位	観点	重要度
1	追い越し支援	最重要
2	車線維持・高速横制御	最重要
3	経路・コーナー速度計画	最重要
4	スタート／Boost戦略	最重要
5	ACC・追い越し準備	高
6	V2X欠損耐性	高
7	接触後Recovery	中～高
8	自己位置推定	現行が安定していれば低
9	AEB	最低限だけ
10	汎用完全自動運転設計	予選後

この前提なら、最初に見るべきコードはAEBではなく、Boost無効化、Domain 2速度制限、操舵gain 1.5のモデル不一致、追い越しlineの6 m/s²制限です。ここは構造改修なしでも順位へ直接効く可能性が高いです。