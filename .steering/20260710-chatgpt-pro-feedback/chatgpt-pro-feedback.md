以下のC++ソースコードとtrajectory CSVを、自動運転AIチャレンジ2026のルールベース部門向けに修正してください。

目的は、約348.7mの周回コースにおいて、経路追従、急カーブ、V2X追い越し、低速車回避を安定させることです。

現在のtrajectory CSVは約1m間隔で、最大曲率は約0.39 1/mです。1mごとに最大約22度向きが変わるため、現在の空間離散MPCには粗すぎます。

変更は以下の優先順位で実施してください。

# 1. 角度正規化の修正

現在の実装:

```cpp
double wrap_to_pi(const double angle)
{
  return std::fmod(angle + kPi, 2.0 * kPi) - kPi;
}
```

は、負方向にπをまたぐケースを正しく処理できません。

trajectoryには、例えば以下のような区間があります。

```text
psi = 3.1121347
psi = -3.0034554
```

本来の角度差は約+0.168radですが、現在の実装では約-6.116radとして扱われる可能性があります。

以下のような実装へ変更してください。

```cpp
double wrap_to_pi(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
```

または同等に正しく[-π, π]へ正規化できる実装にしてください。

この修正には単体テストを追加してください。

テストケース:

```text
wrap_to_pi(0) = 0
wrap_to_pi(π) は ±π
wrap_to_pi(-π) は ±π
wrap_to_pi(3.112 - (-3.003)相当)
wrap_to_pi(-3.003 - 3.112) ≈ +0.168
wrap_to_pi(5π)
wrap_to_pi(-5π)
```

# 2. 周回trajectoryの重複終点除去

CSVの先頭と末尾は同一地点です。

```text
先頭:
89656.8036175, 43128.8719252

末尾:
89656.8036175, 43128.8719252
```

さらにcircular=trueの場合、construct_path()内で先頭点を末尾へ追加しているため、ゼロ長区間が生成されます。

construct_path()の開始時に、circularかつ先頭・末尾間距離が十分小さい場合は末尾を削除してください。

例:

```cpp
if (
  circular && wp_x.size() >= 2 &&
  std::hypot(wp_x.front() - wp_x.back(), wp_y.front() - wp_y.back()) < 1e-3)
{
  wp_x.pop_back();
  wp_y.pop_back();
}
```

削除後も最低限必要な点数が残っていることを確認してください。

# 3. waypoint補間数の修正

現在は以下の実装です。

```cpp
static_cast<int>(d / resolution)
```

これは切り捨てになるため、元データが約0.999m間隔の場合、resolution=0.25としても3分割しかされず、実際には約0.333m間隔になります。

以下へ変更してください。

```cpp
const int subdivisions = std::max(
  1,
  static_cast<int>(std::ceil(d / resolution)));
```

生成された隣接waypoint間距離が原則としてresolution以下になるようにしてください。

ただし極端に短い区間、ゼロ長区間、数値誤差を適切に処理してください。

# 4. trajectory CSVの全列を読み込む構造へ変更

現在のload_ref_path()はx_mとy_mしか読み込んでいません。

CSV列:

```text
s_m
x_m
y_m
psi_rad
kappa_radpm
vx_mps
ax_mps2
```

これを専用構造体で読み込んでください。

例:

```cpp
struct ReferencePathPoint
{
  double s;
  double x;
  double y;
  double psi;
  double kappa;
  double velocity;
  double acceleration;
};
```

CSV値が欠落、不正、非有限値の場合は黙って0へ変換せず、行番号と列名を含むエラーまたは警告を出してください。

現在のread_csv_columns()ではstod失敗時に0.0を挿入していますが、経路データでは危険なため修正してください。

必須列が存在しない場合は起動時に例外としてください。

# 5. 0.25m間隔での周期的な再サンプリング

元CSVは約1m間隔です。

内部MPC用ReferencePathは以下を目標に再生成してください。

```text
内部空間間隔: 0.25m
周回長: 約348.7m
周回境界で位置、向き、曲率が連続
```

単純な移動平均だけでなく、可能であればs_mを独立変数とした周期スプラインでx(s), y(s)を補間してください。

そこから以下を再計算してください。

```text
psi(s) = atan2(y'(s), x'(s))

kappa(s) =
  (x'(s) * y''(s) - y'(s) * x''(s)) /
  pow(x'(s)^2 + y'(s)^2, 1.5)
```

周期スプライン導入が難しい場合は、まず既存線形補間を改善しても構いませんが、以下を満たしてください。

```text
・π境界でpsiが破綻しない
・周回終端でゼロ長区間がない
・曲率に極端なスパイクが発生しない
・最大waypoint間隔が設定resolution以下
```

元CSVのpsi_rad、kappa_radpmは、検証用または初期値として利用してください。

# 6. smoothing_distanceの意味を明確化

現在のsmoothing_distanceはメートルではなくwaypoint個数として使用されています。

resolutionを1mから0.25mへ変更すると、同じ値でも物理的な平滑化距離が4分の1になります。

以下のどちらかへ変更してください。

推奨:

```text
smoothing_distance_m
```

としてメートル単位で設定し、内部でwaypoint数へ変換する。

または最低限:

```text
実際の平滑化幅 = resolution × waypoint数
```

がログに出るようにしてください。

初期値は、0.25m間隔に対して片側3点程度を候補としてください。

ただし急カーブを過度に丸めないようにしてください。

# 7. MPC horizonを距離ベースで管理

現在のMPCは空間離散型なので、Nだけでなく以下を明示的に管理してください。

```text
MPC予測距離 = waypoint間隔 × N
```

初期設定候補:

```yaml
reference_path:
  resolution: 0.25

mpc:
  N: 64
  control_rate: 30.0
```

これで約16m先まで予測します。

高速走行時の候補:

```yaml
reference_path:
  resolution: 0.25

mpc:
  N: 80
  control_rate: 40.0
```

これで約20m先まで予測します。

ただし、単に固定Nを読むのではなく、可能であれば設定値として以下を導入してください。

```yaml
mpc:
  horizon_distance: 16.0
```

そして、

```cpp
N = ceil(horizon_distance / reference_path_resolution);
```

で決めてください。

Nには上下限を設定してください。

# 8. model->wp_idの破壊的変更を廃止

init_problem()内にある以下の処理を削除してください。

```cpp
model->wp_id += effective_wp_id_offset();
```

init_problem()はOSQP再試行時に複数回呼ばれるため、現在の実装ではwp_id_offsetが累積する可能性があります。

以下のようなローカル変数を使用してください。

```cpp
const int base_wp_id = model->wp_id;
const int planning_wp_id = base_wp_id + effective_wp_id_offset();
```

MPC参照、曲率参照、path constraint参照、prediction生成ではplanning_wp_idを使用してください。

model->wp_idは現在位置に最も近いwaypoint IDとして維持してください。

# 9. waypoint ID依存設定を距離sへ変更

以下の設定は、resolution変更により意味が変わります。

```text
wp_id_offset
wp_id_low_offset
v2x_overtake_forbidden_wp_ranges
ReferenceVelocityConfigulatorのwp_id
その他wp IDで指定される区間
```

可能なものはメートル単位へ変更してください。

例:

```yaml
mpc:
  lookahead_offset_m: 1.5
  low_speed_lookahead_offset_m: 0.5

v2x:
  overtake_forbidden_s_ranges:
    - [68.0, 76.0]
    - [114.0, 126.0]
```

内部でReferencePathの累積sからwaypoint IDへ変換してください。

周回境界をまたぐ区間にも対応してください。

# 10. 速度プロファイルの修正

現在は起動時にcompute_speed_profile()を呼んだ後、control()内で毎周期以下を実行しています。

```cpp
reference_path_->set_v_ref(
  std::vector<double>(
    reference_path_->waypoints.size(), effective_v_max));
```

このため曲率ベース速度プロファイルが毎周期失われています。

この全点一定速度上書きを廃止してください。

速度プロファイルは以下から生成してください。

```text
・全体速度上限
・曲率による横加速度制限
・前進方向の加速度制限
・後退方向の減速度制限
・区間別速度上限
・V2Xによる一時的な速度制限
```

基本式:

```cpp
v_curve[i] =
  std::sqrt(ay_max / (std::abs(kappa[i]) + epsilon));
```

前進パス:

```cpp
v[i + 1] = std::min(
  v[i + 1],
  std::sqrt(v[i] * v[i] + 2.0 * a_max * ds));
```

後退パス:

```cpp
v[i] = std::min(
  v[i],
  std::sqrt(v[i + 1] * v[i + 1] +
            2.0 * std::abs(a_min) * ds));
```

周回コースなので、開始・終了境界も連続するよう複数回反復するか、経路を複数周分展開して中央1周分を取得してください。

CSVのvx_mpsは参考値として読み込んでください。ただしそのまま使用しないでください。

提供CSVには以下のような値があります。

```text
最高速度: 約18.43m/s
最大正加速度: 約5.16m/s²
最大負加速度: 約-30.78m/s²
```

2026用の制約とは合わないため、現在設定のa_max、a_min、ay_max、v_maxで再生成してください。

# 11. compute_speed_profile()の数式確認

現在は速度vを変数として、

```cpp
(v[i + 1] - v[i]) / (2 * ds)
```

のような制約を作っています。

物理的な加速度制約は概ね、

```text
(v[i+1]^2 - v[i]^2) / (2ds)
```

です。

したがって、現在のOSQP速度プロファイルは使用せず、まず前進・後退パス方式へ置き換えてください。

OSQPで実装する場合はv²を変数とするなど、次元的に正しい定式化にしてください。

# 12. MPC内または出力側に速度変化制限を追加

MPCの速度入力には、現状明示的な加減速制約がありません。

最低限、制御出力側で以下を実施してください。

```cpp
const double max_dv_up = a_max * dt;
const double max_dv_down = std::abs(a_min) * dt;

u[0] = clip(
  u[0],
  last_u_[0] - max_dv_down,
  last_u_[0] + max_dv_up);
```

可能であればMPCの入力列に以下を追加してください。

```text
-a_decel * dt_i <= v[i+1] - v[i] <= a_accel * dt_i
```

空間離散なので、

```cpp
dt_i = ds_i / max(v_ref_i, minimum_speed);
```

を使用してください。

# 13. 低速線形化の保護

現在は、

```cpp
if (v_ref == 0.0)
```

のみ特別処理しています。

v_refが非常に小さい場合、

```text
1 / v_ref
1 / v_ref²
```

が大きくなります。

以下のような閾値を導入してください。

```cpp
constexpr double kMinLinearizationSpeed = 0.5;
```

```cpp
if (std::abs(v_ref) < kMinLinearizationSpeed) {
  // 安定した低速モデル
}
```

低速車回避、停止、SafetyBrake時にNaNや巨大係数が発生しないようにしてください。

# 14. V2X予測時間をwaypoint index × Tsから距離÷速度へ変更

現在は以下です。

```cpp
const double horizon_t =
  std::min(
    static_cast<double>(i + 1) * model.Ts,
    cfg.prediction_time);
```

しかしiは空間waypoint indexであり、Tsは制御周期なので整合していません。

制御周期を変更すると、同じ空間位置に対する他車予測時間が変わってしまいます。

以下へ変更してください。

```cpp
double cumulative_distance = 0.0;

for (int i = 0; i < N; ++i) {
  const auto & p0 =
    model.reference_path->get_waypoint(ref_wp_id + i);
  const auto & p1 =
    model.reference_path->get_waypoint(ref_wp_id + i + 1);

  cumulative_distance += p0.distance_to(p1);

  const double prediction_speed = std::max(
    0.5,
    std::max(current_speed_mps, p0.v_ref));

  const double horizon_t = std::min(
    cumulative_distance / prediction_speed,
    cfg.prediction_time);
}
```

current_speed_mpsをV2XGapPlannerへ渡せるようにAPIを整理してください。

# 15. V2X前方車判定をコースs/d座標へ統一

現在のevaluate_v2x_behavior()では、現在waypointの接線だけを使ってlongitudinal/lateralを計算しています。

急カーブやヘアピンでは誤判定するため、相手車両をReferencePathへ投影してください。

以下の構造を導入してください。

```cpp
struct ProjectedOpponent
{
  std::string id;
  int waypoint_id;
  double relative_s;
  double lateral;
  double longitudinal_velocity;
  double euclidean_distance;
};
```

判定は原則として以下を使用してください。

```text
relative_s > 0 かつ lateral overlap → 前方車
abs(relative_s)が小さく横方向が近い → 側方車
relative_s < 0 → 後方車
```

速度は、

```cpp
std::hypot(vx, vy)
```

ではなく、投影先waypoint接線方向の速度を使用してください。

```cpp
v_s = cos(psi) * vx + sin(psi) * vy;
```

# 16. V2X自車除外を距離だけに依存しない

現在は、

```cpp
if (self_distance < self_filter_radius) {
  continue;
}
```

で自車候補を除外しています。

横並びの他車を自車として除外する可能性があります。

優先順位を以下にしてください。

```text
1. vehicle_idで自車を除外
2. 自車IDが取得できない場合のみ、小さい距離閾値を補助的に使用
```

vehicle_idが空の場合、全車を"**unknown**"として同じエントリへ上書きしないでください。

少なくとも配列indexを付与するか、簡易最近傍追跡を導入してください。

# 17. 追い越し復帰条件の改善

OvertakeLineConfigにある、

```cpp
return_clear_distance
```

が実質使用されていません。

以下を満たした場合のみReturnへ移行してください。

```text
・追い越し対象よりreturn_clear_distance以上前に出た
・側方車がいない
・戻り先の後方安全距離が確保されている
```

V2Xメッセージが一瞬途切れたことだけでReturnへ移行しないようにしてください。

追い越し対象vehicle_idを状態として保持してください。

# 18. max_target_changeを時間単位へ変更

現在のmax_target_changeは1制御周期あたりの変化量なのでcontrol_rate依存です。

以下へ変更してください。

```yaml
v2x_overtake_line_max_target_change_rate: 1.0
```

単位はm/sとしてください。

```cpp
const double max_change =
  max_target_change_rate * model->Ts;
```

# 19. OSQP成功判定の強化

現在はsolution->xの有無しか確認していません。

以下を確認してください。

```text
OSQP_SOLVED
OSQP_SOLVED_INACCURATE
```

以外は失敗として扱ってください。

また、以下の判定を削除してください。

```cpp
control_signals[i] == 0.0
```

直線ではステアゼロが正常です。

解の妥当性は以下で確認してください。

```text
・OSQP status
・allFinite()
・入力上限内
・状態制約違反量
・NaN/Infなし
```

# 20. OSQP再初期化コストの計測

現在は毎周期、

```cpp
osqp_setup
osqp_solve
osqp_cleanup
```

しています。

以下の処理時間を計測し、throttled logまたはdiagnosticへ出してください。

```text
ReferencePath更新時間
MPC問題生成時間
OSQP setup時間
OSQP solve時間
control()全体時間
```

可能であればOSQP workspaceを再利用し、P、A、q、l、uの更新APIを使用してください。

ただし、まず正しさを優先してください。

# 21. MultiThreadedExecutorの安全性

現在は2スレッドのMultiThreadedExecutorですが、odom、trajectory、reference_path、parameter更新、path constraintsにデータ競合の可能性があります。

最初はSingleThreadedExecutorへ変更してください。

```cpp
rclcpp::executors::SingleThreadedExecutor executor;
```

複数スレッドを維持する場合は、共有データのコピーまたはmutex保護を実装してください。

特に以下を保護してください。

```text
odom_
trajectory_
reference_path_
path_constraints
border_cells
mpc_cfg_
mpc_->cfg
```

# 22. 古い2024依存コードの整理

以下は2024由来または現仕様と異なる可能性が高いため、提出用コードから分離してください。

```text
/aichallenge/pitstop/condition
use_bug_acc_
acc = 500.0
/boost_commander/command
AckermannControlBoostCommand
```

通常制御は常に以下へ送ってください。

```text
/control/command/control_cmd
```

加速度にはコード側ハード上限を設けてください。

```cpp
constexpr double kRuleMaxAcceleration = 1.0;
acc = std::min(acc, kRuleMaxAcceleration);
```

ブーストを使用する場合は、競技の正式なAWSIMブーストインターフェースを独立したクラスへ実装してください。

通常加速度とブーストを混同しないでください。

# 23. 出力加速度制御の改善

現在は以下です。

```cpp
acc = 100.0 * (u[0] - actual_v);
```

ゲイン100は実質的にほぼ常時clampされるため、速度制御として粗いです。

最低限、設定値化してください。

```yaml
speed_control:
  kp: ...
  ki: ...
  feedforward_enabled: true
```

可能であれば、

```text
目標速度差P制御
目標速度プロファイル由来のfeedforward加速度
加速度上限
加速度変化率jerk上限
```

を実装してください。

例:

```cpp
double acceleration_command =
  feedforward_acceleration +
  kp * (target_velocity - actual_velocity);
```

その後、

```cpp
acceleration_command =
  clip(acceleration_command, a_min, a_max);
```

さらにjerk制限をかけてください。

# 24. trajectory検証ツールを追加

CSVまたは再サンプリング後ReferencePathについて、以下を出力する検証ツールを追加してください。

```text
総距離
点数
最小・最大waypoint間隔
平均waypoint間隔
最大|kappa|
最小曲率半径
最大psi差
周回境界の位置差
周回境界のpsi差
周回境界のkappa差
速度最小値・最大値
加速度最小値・最大値
横加速度最大値 v²|kappa|
非有限値の有無
ゼロ長区間の数
```

以下を異常として扱ってください。

```text
waypoint間隔 <= 1e-6
waypoint間隔 > resolution * 1.05
|psi差|が不自然に大きい
非有限値
曲率の極端な単点スパイク
周回境界不連続
```

# 25. 必要なテスト

少なくとも以下のテストを追加してください。

```text
・wrap_to_piのπ境界
・重複終点除去
・0.999m区間をresolution=0.25で4分割できる
・再サンプリング後の最大間隔が0.25m以下
・周回境界でpsiが連続
・速度プロファイルがa_max、a_min、ay_maxを満たす
・init_problem()を複数回呼んでもmodel->wp_idが変化しない
・control_rateを変えてもV2Xの同一点予測時間が大きく変わらない
・V2X vehicle_id空文字が複数車両を上書きしない
・直線でステア0のOSQP解を成功として扱う
```

# 26. 初期パラメータ

まず以下を基準としてください。

```yaml
reference_path:
  resolution: 0.25
  circular: true
  smoothing_distance_m: 0.75

mpc:
  horizon_distance: 16.0
  control_rate: 30.0
  max_acceleration: 1.0

v2x:
  overtake_gap_lookahead_distance: 25.0
```

高速域でOSQP計算時間に余裕がある場合のみ、次を比較してください。

```yaml
reference_path:
  resolution: 0.25

mpc:
  horizon_distance: 20.0
  control_rate: 40.0

v2x:
  overtake_gap_lookahead_distance: 30.0
```

# 27. 完了条件

以下を満たした状態を完成としてください。

```text
・trajectoryが約0.25m間隔で再サンプリングされる
・周回境界にゼロ長区間がない
・π境界で曲率スパイクが発生しない
・MPC予測距離が設定値と一致する
・init_problem()再試行でwp_idが進まない
・速度プロファイルが毎周期一定速度で上書きされない
・速度プロファイルがa_max、a_min、ay_maxを満たす
・V2X予測時間が距離÷速度で算出される
・前方、側方、後方車がコースs/d座標で分類される
・OSQPステータスを正しく確認する
・加速度指令が1.0m/s²を超えない
・古いbug acceleration処理が提出コードから除去される
・全テストが通る
・変更内容とパラメータ移行方法をREADMEへ記載する
```

既存動作を一度に壊さないよう、変更は可能な限り小さい単位へ分割してください。

推奨コミット単位:

```text
1. angle wrapping and circular path fixes
2. path resampling and trajectory validation
3. distance-based horizon and waypoint configuration migration
4. speed profile correction
5. MPC state mutation and OSQP status fixes
6. V2X time prediction and s/d projection
7. race safety cleanup and legacy removal
8. tests and documentation
```
