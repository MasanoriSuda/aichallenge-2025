# ChatGPT Pro 指摘対応 Design

作成日: 2026-07-10  
更新日: 2026-07-10  
状態: First Implementation Slice Complete / Simulator Verification Pending

## 結論

27項目を一括変更せず、次の順に依存関係を固定する。

1. 現状計測、validator、C++ test 基盤を用意する。
2. `wp_id` 破壊、OSQP 誤判定、data race、角度・周回 seam、CSV 読込を直す。
3. 周期 ReferencePath、距離 horizon、距離 `s` 設定へ移行する。
4. 次元的に正しい速度プロファイルと縦制御を同時に導入する。
5. V2X の識別、`s/d` 投影、時間予測、追い越し復帰を共通座標上へ統合する。
6. 計算時間と gate 結果を見て、0.25 m / 16 m / 30 Hz の候補値を既定化する。
7. 公式 boost interface が確認できるまで、通常制御から legacy boost を分離するだけに留める。

特に以下は同じ変更単位で扱う。

- #8 `wp_id` 非破壊化と #19 OSQP 成功判定。直線の steering 0 による再試行で参照点を累積させない。
- #10 速度プロファイル保持と #11 数式修正。一定速度上書きだけを外して、次元の誤った旧プロファイルを有効化しない。
- #12 速度変化制限、#22 legacy 整理、#23 longitudinal control。最終出力の安全層を一貫させる。
- #15 `s/d` 投影、#16 車両識別、#17 Return 条件。異なる車両や座標基準を混ぜない。

## 設計原則

- `/control/command/control_cmd` と既存 topic / service / Domain 契約を変えない。
- `aichallenge_submit/` 内の変更に閉じる。
- pure algorithm を ROS node から分離し、単体テストと validator で同じコードを使う。
- 経路の添字ではなくメートル、制御周期回数ではなく秒を設定契約にする。
- static path / speed と runtime overlay を分け、制御周期中に基準データを破壊しない。
- まず正しさを確立し、OSQP workspace 再利用などの最適化は計測後に行う。
- behavior change は config で比較可能にし、対応 phase の受け入れ後に既定値を切り替える。
- 公式2026仕様とローカル候補値を分ける。

## 目標アーキテクチャ

```text
trajectory CSV
  -> strict ReferencePathPoint loader
  -> duplicate/seam normalization
  -> canonical arc-length s
  -> periodic resampling [0, L)
  -> optional periodic smoothing
  -> psi/kappa recomputation
  -> trajectory validation
  -> static base speed profile
  -> immutable ReferencePath snapshot

odom + V2X snapshot + runtime parameters
  -> base_wp_id / ego s
  -> planning_wp_id + distance horizon
  -> projected opponents in course s/d
  -> static speed profile + dynamic limits
  -> MPC problem
  -> OSQP status/finite/constraint validation
  -> velocity slew limiter
  -> P + feedforward longitudinal controller
  -> acceleration/jerk safety limiter
  -> /control/command/control_cmd
```

## コンポーネント分割

現在は主要ロジックが `mpc_controller_cpp.cpp` の匿名 namespace に集中し、C++ 単体テストを作りにくい。段階的に次へ分離する。

| コンポーネント | 主な責務 |
|---|---|
| `reference_path_core` | angle、strict CSV、重複除去、arc length、resampling、psi/kappa、speed profile |
| `reference_path_validator` | raw / internal path の統計、異常判定、CLI exit code |
| `mpc_problem_builder` | base/planning waypoint 分離、horizon、行列、制約 |
| `osqp_solver_adapter` | setup/solve、status、残差・境界検査、timing |
| `v2x_course_projection` | vehicle tracking、path projection、front/side/rear 共通表現 |
| `longitudinal_controller` | velocity slew、P+FF、acceleration、jerk、fail-safe |
| `mpc_controller_cpp` | ROS I/O、snapshot、component orchestration、診断 publish |

初期 refactor では `multi_purpose_mpc_ros_core` のような内部 library target を作り、既存 executable と gtest から link する。topic、node 名、launch entry は変えない。

## ReferencePath 設計

### データモデル

raw CSV は構造体単位で保持する。

```cpp
struct ReferencePathPoint
{
  double s_m{};
  double x_m{};
  double y_m{};
  double psi_rad{};
  double kappa_radpm{};
  double vx_mps{};
  double ax_mps2{};
};
```

内部 waypoint には最低限次を持たせる。

```cpp
struct Waypoint
{
  double s_m{};
  double x{};
  double y{};
  double psi{};
  double kappa{};
  double base_v_ref{};
  double base_a_ref{};
  // existing path bounds and cells
};
```

`base_v_ref` は static profile、MPC 問題内の `ur` / `umax_dyn` は runtime copy とする。V2X や domain 制限で `Waypoint` の全点を毎周期書き換えない。

### Strict CSV loader

1. header の BOM、前後空白を除去する。
2. 必須7列の存在、header 重複、各 data row の列数を検証する。
3. `std::stod` の消費文字数まで確認し、部分数値を拒否する。
4. 全値へ `std::isfinite` を適用する。
5. `s_m` の単調増加、点数、座標のゼロ長候補を検証する。
6. エラーにはファイル、1始まり行番号、列名、入力値、理由を含める。

`load_waypoints()` が読む別形式 CSV と trajectory CSV を同じ暗黙 parser で処理しない。trajectory 用 loader を明示的に分離する。

### Circular 入力の正規化

circular path は次の順で正規化する。

1. 先頭と末尾の位置差を `closure_duplicate_tolerance_m` で比較する。
2. 同一点の場合、末尾の全列を構造体単位で1点削除する。
3. 連続するゼロ長点を検出し、許容する重複以外は error とする。
4. 残存 unique point が3点以上であることを確認する。
5. `s_m` の最終値と座標から求めた閉路幾何長を比較し、差を診断する。

先頭点を末尾へ実データとして再追加しない。閉路 edge は index modulo で仮想的に扱う。

### Canonical arc length

- unique point `i` から `(i+1)%M` のユークリッド距離を積算する。
- circular path の長さ `L` は closing edge を含む幾何長とする。
- CSV `s_m` は入力品質比較と速度・加速度 metadata の補間に使うが、幾何処理の唯一の正本にはしない。
- CSV `s_m` と幾何長の差が閾値を超えた場合は、strict mode で error、diagnostic mode で warning とする。

### 周期再サンプリング

最低実装は、依存追加のない周期線形補間とする。

```text
M  = ceil(L / requested_resolution_m)
ds = L / M
s_k = k * ds, k = 0 .. M-1
```

これにより `ds <= requested_resolution_m` となり、末尾重複なしで closing edge も同じ `ds` になる。

各 `s_k` は canonical polyline の該当 segment 上で補間する。`psi/kappa` を角度の線形補間で生成せず、再生成した `x/y` から求める。raw `vx/ax` は診断用に周期補間するが、制御用速度は後段で再計算する。

周期 spline は `periodic_cubic` mode として後続比較候補にする。採用条件は次のすべて。

- 外部 runtime 依存を不必要に増やさない。
- seam の位置・1次・2次微分が連続する。
- raw / periodic linear と比較して単点曲率 spike が減る。
- path 長、壁 clearance、gate3 の lane keeping を悪化させない。

### 姿勢と曲率

uniform periodic points では wrapped neighbor を使う。

```text
x'  = (x[i+1] - x[i-1]) / (2 ds)
y'  = (y[i+1] - y[i-1]) / (2 ds)
x'' = (x[i+1] - 2x[i] + x[i-1]) / ds^2
y'' = (y[i+1] - 2y[i] + y[i-1]) / ds^2

psi = atan2(y', x')
kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)
```

分母が閾値未満なら入力 path を異常として扱い、0へ黙って置換しない。角度差はすべて正しい `wrap_to_pi()` を使う。

### Smoothing

`smoothing_distance_m` は片側距離とする。

```text
half_window_points = round(smoothing_distance_m / actual_ds)
full_width_m       = 2 * half_window_points * actual_ds
```

- circular path では window index を wrap する。
- `0.0` は無効。
- smoothing 後に `s/psi/kappa` を再計算する。
- 起動ログへ requested distance、actual half-window、point count、full width を出す。
- 現行 `smoothing_distance` は migration 中だけ読み、新 key がなければ `old_count * actual_ds` として警告する。
- 0.75 m は初期候補であり、最大曲率、path length、wall clearance、lane keeping の比較前に固定しない。

## Trajectory Validator 設計

同じ core library を使う CLI executable を追加する。

```text
reference_path_validator --config <config.yaml> [--all-domain-paths] [--json <path>]
```

出力する指標:

- 入力ファイル、mode、circular、点数、総距離。
- 最小・最大・平均 spacing。
- 最大 `abs(kappa)`、最小曲率半径、最大 wrapped psi difference。
- seam の位置、psi、kappa 差。
- 速度・加速度の最小・最大。
- 最大 `v^2*abs(kappa)`。
- 非有限値、ゼロ長、過大間隔、曲率 spike の件数と index / `s_m`。
- raw と resampled の path length 差。

異常時は非0終了する。人向け table と machine-readable JSON を同じ計算結果から生成する。生成物は検証ログであり、`output/` の既存契約を変えない。

## 距離ベース Horizon と `s` 設定

### Horizon

ReferencePath 作成後の `actual_ds` を使う。

```text
requested_N = ceil(horizon_distance_m / actual_ds)
N = clamp(requested_N, horizon_steps_min, horizon_steps_max)
actual_horizon_distance = N * actual_ds
```

起動時に requested/actual distance、N、QP variables、constraints、control period をログへ出す。clamp が発生したら warning とする。

既存 `N` は新 key 未指定時だけ fallback とする。`current_control`、path constraints、border cells、V2X planner bounds は算出後の同じ N を使う。

### Waypoint ID から `s` への移行

ReferencePath に次の API を置く。

```cpp
int waypoint_id_at_s(double s_m) const;
double wrapped_s(double s_m) const;
double forward_distance_s(double from_s, double to_s) const;
double signed_shortest_distance_s(double from_s, double to_s) const;
bool circular_range_contains(double s_m, double start_s, double end_s) const;
```

新しい meter key を canonical とし、旧 index key は migration adapter で現在 path 上の距離へ変換する。新旧両方がある場合は新 key を優先して警告する。

`ReferenceVelocityConfigulator` は `std::map<double, velocity>` の `s_m` 区間へ移行し、周回 wrap を明示的に処理する。順序が保証されない入力 map の iteration に依存しない。

## 速度プロファイル設計

### Static base profile

各点の初期上限を次で作る。

```text
v_limit[i] = min(
  global_v_max,
  section_v_max[i],
  sqrt(ay_max / max(abs(kappa[i]), kappa_epsilon)))
```

`v_limit^2` を使う周期 forward / backward relaxation で縦加減速制約を適用する。

```text
forward:
v[j]^2 <= v[i]^2 + 2 * a_max * ds_ij

backward:
v[i]^2 <= v[j]^2 + 2 * abs(a_min) * ds_ij
```

`j=(i+1)%M` とし、seam を含め、変化量が tolerance 未満になるまで有限回反復する。最大 iteration 到達は error とする。旧 speed-profile OSQP は使用しない。

`base_a_ref` は最終 `v_ref` から `(v[j]^2-v[i]^2)/(2ds)` で計算し、feedforward と validator に使う。

### Runtime overlay

毎 control cycle は base profile を変更せず、horizon local copy に次を `min` 合成する。

1. domain `v_max`。
2. start duration `v_max`。
3. `s_m` 区間速度。
4. curve / MPC dynamic limit。
5. V2X Follow / SafetyBrake / gap planner limit。

`v_max`、`a_max/a_min`、`ay_max` の parameter 更新で static 条件が変わる場合は、control cycle 外で新 profile を構築し、検証後に snapshot を atomic swap する。

## MPC 問題と OSQP 設計

### Waypoint state の非破壊化

```cpp
const int base_wp_id = model->wp_id;
const int planning_wp_id = reference_path.wrap_id(
  base_wp_id + effective_wp_id_offset());
```

`init_problem()` は `planning_wp_id` を引数または local immutable value とし、次を同じ基準にする。

- kappa / v reference。
- path constraints / border cells。
- V2X behavior / gap planner。
- prediction marker。
- last solved waypoint diagnostic。

OSQP retry は safety margin だけを変え、base/planning ID を変えない。

### 低速線形化

初期実装は `abs(v_ref) < min_linearization_speed_mps` で明示的な低速 branch を使い、`1/v`、`1/v^2` を生成しない。通常 branch との境界、0、閾値直下・直上で finite と command continuity をテストする。

最低速度 clamp を使う別案は、停止中にも時間状態が進む影響を評価してから採用する。

### OSQP adapter

solver は solution だけでなく診断を返す。

```cpp
struct OsqpSolveResult
{
  enum class Status { Solved, SolvedInaccurate, Rejected, SetupFailed };
  Status status;
  Eigen::VectorXd x;
  double setup_ms{};
  double solve_ms{};
  double max_constraint_violation{};
};
```

受理条件:

- OSQP status が `SOLVED` または `SOLVED_INACCURATE`。
- `x.allFinite()`。
- 入力部分が設定 bounds 内。
- `A*x` が `l/u` を tolerance 内で満たす。
- dimension が期待値と一致する。

steering が厳密に `0.0` であることを失敗理由にしない。`SOLVED_INACCURATE` と constraint tolerance は warning log に残す。

失敗時は古い horizon の任意要素へ進まず、最後の valid command を acceleration / steering rate limit 付きで減速方向へ更新する。連続失敗閾値を超えたら速度0を目標にする。

### Performance

次を `std::chrono::steady_clock` で計測する。

- ReferencePath 更新。
- MPC problem build。
- OSQP setup。
- OSQP solve。
- solution validation。
- control total。

平均だけでなく max / p95 / p99 相当を一定窓で集計する。workspace 再利用は、matrix dimension と sparsity pattern が安定し、setup が有意なボトルネックであることを確認した後の別 phase とする。

## Executor と状態 snapshot

第一段階は `SingleThreadedExecutor` へ変更する。control callback の先頭で次を local snapshot にする。

- `Odometry::SharedPtr`。
- `Trajectory::SharedPtr`。
- validated ReferencePath shared snapshot。
- V2X active tracks snapshot。
- MPC config / runtime parameter snapshot。
- path constraints / border cells snapshot。

parameter callback や trajectory callback は live object を途中まで変更せず、新 object の構築・検証後に pointer を置換する。

SingleThreaded で callback age / deadline を満たせない場合だけ、次の条件で MultiThreaded を再導入する。

- callback group の責務を分離する。
- mutable state ごとの owner を決める。
- lock 中に OSQP solve や publish を行わない。
- control cycle は immutable snapshot だけを参照する。
- ThreadSanitizer 相当を通常 Docker で使えない場合も、stress test と generation counter で途中更新を検出する。

## V2X 設計

### 共通投影表現

```cpp
struct ProjectedOpponent
{
  std::string track_id;
  std::string source_vehicle_id;
  int waypoint_id{};
  double s_m{};
  double relative_s_m{};
  double lateral_m{};
  double longitudinal_velocity_mps{};
  double euclidean_distance_m{};
  double projection_error_m{};
  double last_seen_sec{};
};
```

point-to-waypoint ではなく、近傍 path segment へ直交投影し、segment 内比率から `s_m` と符号付き lateral を求める。前回投影付近を先に探索し、position jump または不成立時に global search へ fallback する。

circular path では local behavior 範囲が半周未満であることを前提に、`relative_s_m` を符号付き最短距離へ wrap する。hairpin の空間的近接を前方車と誤認しないよう、path distance、projection error、Euclidean distance、lateral overlap を併用する。

相手速度は投影 segment の接線へ射影する。

```text
v_s = cos(psi) * vx + sin(psi) * vy
```

### Vehicle identity と self filter

1. `ego_vehicle_id` が利用可能で source ID と一致した場合だけ確実に self として除外する。
2. self ID が利用できない場合のみ、非常に小さい `self_filter_radius_m` を補助条件にする。
3. 非空 ID は `id:<vehicle_id>` track とする。
4. 空 ID は message 内 index で上書きせず、位置・時刻・最大 jump の nearest-neighbor association から `anon:<counter>` track を作る。
5. association が曖昧な場合は track を統合せず、新 track と warning を生成する。
6. stale track は timeout で planning 対象から外すが、追い越し target state には dropout hold を適用する。

公式 V2X で ID 保証と自車 ID 取得方法が確定したら、この fallback を簡略化する。

### Prediction time

horizon point ごとに path の実距離を積算する。

```text
prediction_speed = max(
  min_prediction_speed_mps,
  abs(current_speed_mps),
  base_v_ref[i])

horizon_t = min(cumulative_distance / prediction_speed, prediction_time)
```

同じ spatial point の `horizon_t` が control rate 変更で変わらないことをテストする。速度の選び方は過大・過小予測を log 比較し、必要なら opponent longitudinal speed を組み込む。

### Overtake target と Return

Overtake state に次を追加する。

- target `track_id`。
- target last seen time / last projected `s/d`。
- pass side。
- phase start `s` と time。
- return-side rear safety candidate。

Return への遷移条件:

```text
ego_ahead_of_target >= return_clear_distance_m
AND no side overlap
AND rear gap on return side >= return_rear_clear_distance_m
AND target observation is fresh enough
```

V2X が一瞬消えた場合は clear とみなさず、`overtake_target_dropout_hold_sec` の間は target state を保持する。hold を超え、安全確認できない場合は速度を制限し、新規 lateral target を増やさず Recovery / Follow へ移る。

### Target change rate

```text
bounded_dt = clamp(actual_dt, 0.0, max_target_change_dt_sec)
max_change = max_target_change_rate_mps * bounded_dt
```

長時間 pause 後の1周期で大きく動かないよう `dt` に上限を設ける。legacy `m/cycle` key は自動変換せず、元 control rate を確認して手動移行する。

## Longitudinal Control と Legacy 分離

### 通常制御

MPC の目標速度へ最初に slew limit を掛ける。

```text
v_cmd_limited = clamp(
  v_cmd,
  last_v_cmd - abs(a_min) * dt,
  last_v_cmd + a_max * dt)
```

次に P + feedforward を計算する。

```text
a_raw = base_a_ref + kp * (v_cmd_limited - actual_v)
a_limited = clamp(a_raw, a_min, effective_a_max)
a_command = jerk_limit(a_limited, last_acc, dt)
a_command = final_acceleration_safety_clamp(a_command)
```

最終 safety clamp は low-pass / jerk 処理後にも適用する。`dt<=0`、大きな time jump、初期化、control disable、solver failure は専用 branch とする。

I 項は定常誤差の実測後に追加し、anti-windup、reset 条件、saturation log を同時に実装する。

### Legacy / boost

- main MPC node は常に通常 `AckermannControlCommand` を生成する。
- `acc=500.0`、`use_bug_acc_` 命名、旧 pitstop condition による通常 control 分岐を除去する。
- `/control/command/control_cmd` の最終 publisher を維持する。
- custom `AckermannControlBoostCommand`、`boost_commander` executable、launch include は依存調査後に通常提出経路から外す。
- 2026 の正式 boost interface が公開された場合だけ、通常加速度とは別の adapter / arbitration として設計し、`docs/interface/` を先に更新する。

## Config 案

最終 key 名は実装時に既存 YAML との重複を再確認する。

```yaml
reference_path:
  processing_mode: periodic_linear  # legacy, periodic_linear, periodic_cubic
  resolution: 0.25                  # [m]
  circular: true
  closure_duplicate_tolerance_m: 0.001
  smoothing_distance_m: 0.75        # one-sided [m]
  strict_validation: true

mpc:
  horizon_distance_m: 16.0
  horizon_steps_min: 8
  horizon_steps_max: 96
  min_linearization_speed_mps: 0.5
  lookahead_offset_m: 1.5
  low_speed_lookahead_offset_m: 0.5

  speed_profile_mode: forward_backward

  speed_control:
    kp: 1.0
    feedforward_enabled: true
    acceleration_safety_limit_mps2: 1.0
    jerk_up_limit_mps3: TBD
    jerk_down_limit_mps3: TBD

  v2x_overtake_forbidden_s_ranges_m: []
  v2x_overtake_line_max_target_change_rate_mps: 1.0
  v2x_overtake_line_return_clear_distance_m: 4.0
  v2x_overtake_line_return_rear_clear_distance_m: TBD
  v2x_overtake_target_dropout_hold_sec: TBD
```

0.25 m、0.75 m、16 m、30 Hz、1.0 m/s²、target rate 1.0 m/s は初期比較候補であり、公式確定値ではない。

## 段階導入と rollback

### Phase 0: Baseline と test seam

- 現行 CSV、current config、control timing、gate 結果を保存する。
- core library / gtest / validator の骨格を作る。
- production behavior は変更しない。

### Phase 1: Deterministic correctness

- #8、#19、#21を先に修正する。
- #1〜#4を strict path processing として追加する。
- solver fallback と invalid data の fail-fast を確認する。

Rollback: config ではなく小コミット単位で戻せるよう、path、solver、executor を分ける。

### Phase 2: Periodic path と距離設定

- `processing_mode=periodic_linear` を opt-in で追加する。
- #5〜#7、#9、#24を実装する。
- 全 domain CSV、lane keeping、QP timing を確認後に既定化する。

Rollback: `processing_mode=legacy`、旧 N / wp key fallback。

### Phase 3: Speed / acceleration

- #10と#11を同時に `speed_profile_mode=forward_backward` として追加する。
- #12、#13、#23の output safety layer を有効化する。
- #22の危険 legacy branch を通常経路から外す。

Rollback: base profile / controller mode を比較できる期間を設ける。ただし `acc=500` は rollback 対象にしない。

### Phase 4: V2X

- #16 identity、#15 projection、#14 time、#18 rate、#17 Return の順に実装する。
- 既存 `use_v2x_gap_planner` / `use_v2x_behavior_fsm` 配下で dev2、dev3/4、gate2 を確認する。
- dropout 中は安全側へ倒す。

Rollback: V2X behavior feature flags を無効化し、base trajectory tracking を維持する。

### Phase 5: Performance と既定値

- #20 timing から 0.25 m / 16 m / 30 Hz を評価する。
- p99 budget を満たす場合だけ 20 m / 40 Hz と workspace reuse を比較する。
- #26 の既定値を決める。

### Phase 6: Integration

- #25全テスト、gate、dev multi-vehicle、submit/evalを実行する。
- #27 DoD を確認する。
- package README と `docs/spec/mpc-integration.md` を更新する。

## ログ・診断

起動時:

- 選択 CSV と domain override。
- raw / normalized / resampled 点数と path length。
- requested / actual resolution、smoothing 幅、horizon distance / N。
- speed profile の min/max、最大縦加減速、最大横加速度。
- legacy config key 使用と換算結果。
- active processing / speed / V2X mode。

throttled runtime:

- odom / V2X / trajectory age。
- problem/setup/solve/validate/control timing。
- OSQP status、iteration、最大制約違反。
- solver fallback count と安全停止遷移。
- effective velocity / acceleration limits と制限理由。
- projected opponent の track ID、relative `s/d`、state classification。
- Overtake target、dropout age、Return block reason。

## 影響範囲

| 領域 | 影響 |
|---|---|
| `multi_purpose_mpc_ros` | 主変更。path、MPC、V2X、縦制御、test、validator |
| `multi_purpose_mpc_ros_msgs` | 正式 boost 方針確定まで変更保留 |
| launch | executable / topic 名は維持。legacy boost include の扱いだけ要確認 |
| config | meter / second key を追加し旧 key を段階移行 |
| `docs/spec/mpc-integration.md` | algorithm、config、検証、migration を更新 |
| `docs/interface/*` | 現契約は変更しない。正式 boost 等で変更が必要な場合のみ先行更新 |
| `aichallenge_system` | 変更なし |
| 実車 `vehicle/` / `remote/` | 本作業では変更なし。sim完了後に別検証 |

## 決定記録

- 2026-07-10: 指摘27項目のうち、正しさ・安全性に関する項目は採用した。
- 2026-07-10: 指摘内のコース長348.7 m、最大曲率0.39 1/mを固定要件にせず、選択CSVの validator 結果を正本とした。
- 2026-07-10: 周期 spline を初手の必須実装にせず、正しい周期線形 resampling を最低実装とした。
- 2026-07-10: 速度プロファイル上書きの削除と旧 OSQP speed profile の有効化を分離せず、前進・後退 pass への置換と同時変更にした。
- 2026-07-10: OSQP workspace 再利用は correctness と timing 測定後へ延期した。
- 2026-07-10: 1.0 m/s² は公式ページ上も「約」であり、設定可能なローカル safety candidate とした。
- 2026-07-10: 2026 boost の存在は確認できるが正式 interface は未確認のため、legacy 危険経路の分離だけを採用し、新 adapter は TBD とした。
- 2026-07-10: `ceil` 分割 helper と単体テストは先行追加するが、本番 ReferencePath は距離ベース horizon と waypoint 設定の距離化が完了するまで legacy `floor` 分割を維持する。選択中の `final_ver3` で即時に `ceil` を有効化すると内部点数が約358点から約709点へ増え、固定 `N=15` の実距離 horizon が約11.2〜15.3 mから約5.7〜8.1 mへ縮むためである。
- 2026-07-10: 周回重複終点の除去により C++ と Python `path_constraints_provider` の waypoint ID がずれないよう、provider 側も同じ closure 正規化へ揃えてから topic constraint mode を検証する。
- 2026-07-10: SingleThreaded化と同時に `odom_timeout_sec: 0.5` のsteady receipt/source-stamp監視を追加した。この値は公式値ではなく、実走行のcallback age計測後に再調整する。
- 2026-07-10: solver失敗とcontrol disableはlegacy boost判定より優先し、boost=false、負加速度、単調減速とする。NaN / Inf、filter後の加速度・操舵境界違反も最終publish前に拒否する。
