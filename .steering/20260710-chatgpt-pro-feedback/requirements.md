# ChatGPT Pro 指摘対応 Requirements

作成日: 2026-07-10  
更新日: 2026-07-10  
状態: First Implementation Slice Complete / Simulator Verification Pending

## 目的

ChatGPT Pro の27項目の指摘を、現行の C++ MPC、trajectory CSV、V2X、OSQP、ROS 2 インターフェースに照らして整理し、安全に実装できる要求へ変換する。

目標は、周回経路の数値品質、MPC の再現性、速度・加速度制約、V2X 追い越し判断、制御出力の安全性を改善しつつ、Automotive AI Challenge 2026 向けベースリポジトリとして既存の起動・評価・提出契約を維持することである。

## 入力と正本

- 指摘原文: `chatgpt-pro-feedback.md`
- 実装対象: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`
- MPC 統合仕様: `docs/spec/mpc-integration.md`
- 安全検証方針: `docs/spec/safety-gates.md`
- 参加者契約: `docs/interface/participant-interface.md`
- 評価契約: `docs/interface/evaluation-interface.md`
- 2026 公式ルール: <https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html>

公式ページは 2026-07-10 時点でも WIP である。加速度上限は「約 1.0 m/s²」、SIM boost は予定として記載されているが、正式な boost topic、message 型、利用条件は確認できない。このため、加速度の安全制限は設定可能なローカル基準として導入し、boost インターフェースは `TBD` とする。

## 調査結果

### 現行実装で確認できた問題

- `wrap_to_pi()` は負側の周期正規化に失敗し得る `fmod` 実装である。`mpc_controller_cpp.cpp:89`
- 周回 CSV の重複終点を残したまま先頭点列を追加し、補間数を `floor` 相当で計算している。`mpc_controller_cpp.cpp:518`
- trajectory loader は実行時に `x_m` / `y_m` しか使わず、数値変換失敗を `0.0` に置換する。`mpc_controller_cpp.cpp:132`, `:257`
- 起動時に作成した曲率速度プロファイルを、制御周期ごとに全点一定速度へ上書きしている。`mpc_controller_cpp.cpp:5575`, `:5929`
- `init_problem()` が `model->wp_id` を破壊的に加算し、OSQP 再試行で offset が累積する。`mpc_controller_cpp.cpp:3035`, `:3411`
- OSQP は `solution->x` の存在だけを確認し、正常な直線ステア `0.0` を失敗扱いして再試行する。`mpc_controller_cpp.cpp:229`, `:3402`
- 低速線形化は `v_ref == 0.0` だけを保護し、微小速度を保護しない。`mpc_controller_cpp.cpp:1015`
- V2X 予測時間は空間 waypoint index に制御周期 `Ts` を掛けている。`mpc_controller_cpp.cpp:1868`
- V2X 主 FSM は現在接線基準の分類が残り、空 ID の全車を `__unknown__` へ上書きする。`mpc_controller_cpp.cpp:1452`, `:2496`
- `return_clear_distance` は読み込まれるが追い越し復帰条件に使われていない。`mpc_controller_cpp.cpp:1089`, `:3664`
- 通常系にはゲイン100の速度 P 制御、legacy 系には `acc = 500.0` と独自 boost message が残る。`mpc_controller_cpp.cpp:5659`, `:5965`, `:5972`
- 2 thread の `MultiThreadedExecutor` に対して、node の odom、trajectory、reference path、parameter 更新は同期されていない。`mpc_controller_cpp.cpp:5676`, `:6076`
- C++ 単体テスト基盤と trajectory validator は存在しない。

### 現在選択される trajectory の実測値

対象は `config.yaml` が選ぶ `env/final_ver3/traj_mincurv.csv`。CSV 値の静的集計結果は次のとおり。

| 指標 | 実測値 |
|---|---:|
| データ点数 | 350 |
| 末尾 `s_m` | 342.0066572 m |
| 最小隣接 `s_m` 差 | 0.4559603 m |
| 最大隣接 `s_m` 差 | 1.5454952 m |
| 先頭・終点位置差 | 0 m |
| 最大 `abs(kappa_radpm)` | 0.6825166 1/m |
| `vx_mps` 範囲 | 5.5241613〜18.4260400 m/s |
| `ax_mps2` 範囲 | -30.7812180〜5.1569546 m/s² |

指摘原文の「約348.7 m」「約1 m間隔」「最大曲率約0.39 1/m」は、現在選択される CSV と一致しない。これらを固定受け入れ値にせず、対象 CSV と再サンプリング結果を validator で都度計測する。

## 採否と実装優先度

優先度は実装順を表し、`P0` が最優先。`条件付き採用` は正しさ、性能、公式仕様の確認後に有効化する。

| # | 指摘 | 採否 | 優先度 | 要点 |
|---:|---|---|---|---|
| 1 | 角度正規化 | 採用 | P0 | π境界修正と単体テスト |
| 2 | 周回重複終点 | 採用 | P0 | 全列同期削除、残存点数検証 |
| 3 | `ceil` 補間 | 採用 | P0 | 周回 seam を含む最大間隔保証 |
| 4 | CSV 全列・厳密読込 | 採用 | P0 | 7必須列、有限値、行・列付き診断 |
| 5 | 0.25 m 周期再サンプリング | 条件付き採用 | P1 | 厳密な周期線形方式を先行し、周期 spline は品質比較後 |
| 6 | smoothing のメートル化 | 採用 | P1 | 片側物理幅として定義し旧 key を移行 |
| 7 | 距離ベース horizon | 採用 | P1 | N 上下限、実距離・計算時間ログ |
| 8 | `wp_id` 非破壊化 | 採用 | P0 | 再試行可能な `planning_wp_id` |
| 9 | waypoint ID 設定の `s` 化 | 条件付き採用 | P1 | 新 key 優先、旧 key 警告付き互換 |
| 10 | 速度プロファイル保持 | 採用 | P0 | 静的基準と動的制限を分離 |
| 11 | 速度プロファイル数式 | 採用 | P0 | #10 と同時に前進・後退 pass へ置換 |
| 12 | 速度変化制限 | 採用 | P0 | 出力側を先行し、MPC 内制約は次段 |
| 13 | 低速線形化保護 | 採用 | P0 | 微小速度でも有限なモデル |
| 14 | V2X 距離÷速度予測 | 採用 | P2 | control rate 非依存化 |
| 15 | V2X `s/d` 投影 | 採用 | P2 | 主 FSM と gap planner の座標判定統一 |
| 16 | V2X self / unknown ID | 条件付き採用 | P2 | ID保証は未確定。明示ID優先＋匿名track |
| 17 | 追い越し復帰条件 | 採用 | P2 | 対象保持、前方・側方・後方安全確認 |
| 18 | target 変化率の時間単位化 | 採用 | P2 | 実 `dt` による m/s 制限 |
| 19 | OSQP 成功判定 | 採用 | P0 | status、finite、境界・制約違反を確認 |
| 20 | OSQP 時間計測 | 一部採用 | P1/P3 | 計測は P1、workspace 再利用は正しさ確立後 |
| 21 | Executor 安全化 | 採用 | P0 | 初期は single thread、並列化は snapshot 設計後 |
| 22 | 2024 legacy 整理 | 一部採用 | P0/TBD | 危険加速度と通常系混在を除去。正式 boost adapter は保留 |
| 23 | 出力加速度制御 | 採用 | P0 | P+FF、clamp、jerk、安全上限 |
| 24 | trajectory validator | 採用 | P0 | raw / resampled の同一基準検証 |
| 25 | 必須テスト | 採用 | 横断 | 各変更と同じ単位で追加 |
| 26 | 初期パラメータ | 条件付き採用 | P3 | 0.25 m / 16 m / 30 Hz は実験候補。計測後に既定化 |
| 27 | 完了条件 | 採用・補正 | 統合 | 固定コース長ではなく validator 基準に補正 |

## 対象範囲

### 対象

- `multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp`
- 分離する path / solver / V2X / longitudinal utility と公開 header
- `multi_purpose_mpc_ros/config/config.yaml`
- `multi_purpose_mpc_ros/config/ref_vel.yaml`
- `multi_purpose_mpc_ros/CMakeLists.txt`、`package.xml`
- `multi_purpose_mpc_ros/test/`
- trajectory validator と必要な起動・利用説明
- `docs/spec/mpc-integration.md`
- 必要な場合のみ `docs/interface/participant-interface.md`

### 対象外

- `/v2x/vehicle_positions` の topic 名・message 型変更。
- `/control/command/control_cmd` の topic 名・message 型変更。
- Domain 0 / Domain 1..N、`v2x_msgs`、AWSIM 管理責務の変更。
- `aichallenge_system/`、result JSON、`output/latest/` の変更。
- 公式仕様が未確認な boost インターフェースの確定実装。
- OSQP workspace 再利用を、正しさ・計測より先に行うこと。
- シミュレータ未検証のまま実車向け既定値を変更すること。

## 機能要求

### R-IF: インターフェースと安全境界

- `R-IF-01`: 最終通常制御は `/control/command/control_cmd` の `AckermannControlCommand` とする。
- `R-IF-02`: `/localization/kinematic_state`、`/planning/scenario_planning/trajectory`、`/set_initial_pose`、`/v2x/vehicle_positions` の現行契約を変えない。
- `R-IF-03`: 変更は `aichallenge_submit/` 内に閉じ、Domain・評価 FSM・成果物 schema を変更しない。
- `R-IF-04`: NaN / Inf、無効 OSQP 解、stale V2X、異常 `dt` を制御出力へ通さない。
- `R-IF-05`: 判定不能時は、新しい横回避へ入らず、最後の安全な状態から rate limit 付き減速へ移る。

### R-PATH: trajectory と ReferencePath

- `R-PATH-01`: `wrap_to_pi()` は任意の有限角を `[-pi, pi]` 相当へ正規化し、負方向の跨ぎを正しく扱う。
- `R-PATH-02`: circular 入力の先頭・終点が許容誤差内で同一点なら、全列を同期して末尾を除去する。除去後3点未満はエラーとする。
- `R-PATH-03`: strict loader は `s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2` を必須とし、欠落列、列数不一致、部分数値、NaN / Inf、非単調 `s_m` を行番号・列名付きで拒否する。
- `R-PATH-04`: circular path の内部点は `[0,L)` に置き、末尾重複を持たない。`M=ceil(L/resolution)`、`ds=L/M` 相当で seam を含む全間隔を resolution 以下にする。
- `R-PATH-05`: 最初の実装は依存追加を避けた周期線形補間と周期中央差分を最低基準とする。周期 spline は validator と gate で曲率品質が改善し、退行しない場合に採用する。
- `R-PATH-06`: `psi` と `kappa` は再サンプリング後の `x/y` から周期的に再計算し、CSV 値は比較・診断用に保持する。
- `R-PATH-07`: `smoothing_distance_m` は片側の物理距離と定義する。実際の点数、片側距離、全幅を起動ログへ出す。
- `R-PATH-08`: validator は raw CSV と内部 ReferencePath の点数、総距離、間隔、曲率、姿勢差、seam、速度、加速度、横加速度、非有限値、ゼロ長を報告し、異常時は非0終了する。
- `R-PATH-09`: コース長や最大曲率をハードコードせず、対象 CSV の validator 結果を基準にする。

### R-MPC: MPC、horizon、solver

- `R-MPC-01`: `model->wp_id` は現在位置に最も近い基準 waypoint とし、問題生成で変更しない。offset 適用後はローカルな `planning_wp_id` として全参照へ明示的に渡す。
- `R-MPC-02`: horizon は `horizon_distance_m` と実際の内部 `ds` から算出し、`N` の上下限、実予測距離、変数・制約数をログへ出す。
- `R-MPC-03`: `abs(v_ref) < min_linearization_speed_mps` の低速モデルを持ち、生成行列と出力が有限であることを検証する。
- `R-MPC-04`: OSQP は `OSQP_SOLVED` または `OSQP_SOLVED_INACCURATE` だけを候補とし、solution finite、入力境界、最大制約違反を確認する。直線の steering `0.0` は正常解として扱う。
- `R-MPC-05`: OSQP 失敗時の再試行は同一 `base_wp_id` で行い、失敗回数が閾値を超えた場合は安全停止へ移る。
- `R-MPC-06`: reference path 更新、問題生成、OSQP setup、solve、control 全体を `steady_clock` で計測し、throttled log または diagnostics へ出す。
- `R-MPC-07`: 初期実装は `SingleThreadedExecutor` とする。将来 multi thread に戻す場合は、制御周期先頭で immutable snapshot を取得し、共有状態の所有権と mutex / callback group を定義する。

### R-SPEED: 速度プロファイルと縦制御

- `R-SPEED-01`: static base speed profile と、domain/start/section/V2X による runtime limit を分離し、毎周期 base profile を全点一定値で上書きしない。
- `R-SPEED-02`: base profile は `v_max` と `sqrt(ay_max/(abs(kappa)+epsilon))` の小さい方を初期上限とし、周期前進・後退 pass で `a_max` / `a_min` を満たす。
- `R-SPEED-03`: 旧 `(v[i+1]-v[i])/(2ds)` を使う OSQP speed profile は使用しない。再度 QP 化する場合は `v^2` など次元的に正しい変数を使う。
- `R-SPEED-04`: `u[0]` は実測 `dt` と `last_u_[0]` から、上昇 `a_max*dt`、下降 `abs(a_min)*dt` の範囲に制限する。初期化、停止、時刻 jump を明示処理する。
- `R-SPEED-05`: longitudinal command は最低限 `feedforward_acceleration + kp * velocity_error` とし、設定された加減速度と jerk の最終制限を適用する。I 項は anti-windup 設計後の追加とする。
- `R-SPEED-06`: 通常出力 acceleration は独立した safety limit を超えない。初期ローカル候補は 1.0 m/s² だが、2026 公式の厳密値としてコード定数化しない。
- `R-SPEED-07`: `acc=500.0` と通常加速度の混在を除去する。boost は通常 command と別責務とし、正式 interface 確認まで無効・未実装とする。

### R-V2X: V2X 時間・座標・識別・復帰

- `R-V2X-01`: 各 horizon 点の予測時刻は累積経路距離を予測速度で割り、上限 `prediction_time` と最低予測速度を適用する。control rate を時間軸として使わない。
- `R-V2X-02`: 他車を ReferencePath へ投影し、track ID、waypoint、`s`、周回補正した `relative_s`、lateral `d`、接線方向速度、ユークリッド距離、観測時刻を保持する。
- `R-V2X-03`: front / side / rear、gap planner、front risk、overtake return は共通の投影結果を使用する。
- `R-V2X-04`: 自車 ID が利用可能な場合は ID 一致で除外する。ID が取得できない場合だけ、小さい距離閾値を補助利用する。
- `R-V2X-05`: 空 ID の複数車両を同一 key に上書きしない。message index だけの一時識別より、距離 gate 付き匿名 track ID を優先する。
- `R-V2X-06`: Overtake 開始時に対象 track ID と最終観測を保持し、対象より `return_clear_distance_m` 以上前、側方車なし、戻り先の後方 gap 安全、の全条件成立後だけ Return へ移る。
- `R-V2X-07`: 短時間の V2X 欠損は clear 判定に使わない。dropout hold 後も安全確認できない場合は新規横移動を止め、速度を制限して Recovery / Follow へ移る。
- `R-V2X-08`: 横 target 変化は `max_target_change_rate_mps * bounded_dt` とし、control rate を変えても1秒当たりの変化量を維持する。
- `R-V2X-09`: topic、message、Domain、`aichallenge_system` は変更せず、既存 V2X feature flags の配下で段階導入する。

### R-TEST: テストと文書

- `R-TEST-01`: pure function を ROS node の巨大 translation unit から分離し、C++ gtest で検証できる構造にする。
- `R-TEST-02`: #1〜#25 の各修正と同じ変更単位で対応テストを追加し、最後にまとめて後付けしない。
- `R-TEST-03`: 全 `csv_path` と `domain_csv_path` を validator に通す。
- `R-TEST-04`: `docs/spec/mpc-integration.md` と package README に新 config、単位、移行、検証手順を反映する。
- `R-TEST-05`: interface 契約に実変更がない限り `docs/interface/` の契約値は変えない。実変更が必要な場合はコードより先に migration note を書く。

## 設定移行要求

| 現行 | 新設定候補 | 移行ルール |
|---|---|---|
| `reference_path.resolution` | 同 key、単位 `[m]` を明記 | 値変更は resampler 有効化と同時 |
| `reference_path.smoothing_distance` | `smoothing_distance_m` | 新 key 優先。旧 key は現 resolution との積で解釈し警告 |
| `mpc.N` | `horizon_distance_m` + `N_min/N_max` | 新 key 優先。旧 `N` は fallback と警告 |
| `wp_id_offset` | `lookahead_offset_m` | 新 key 優先、旧値は現 path 上の距離へ変換 |
| `wp_id_low_offset` | `low_speed_lookahead_offset_m` | 同上 |
| `v2x_overtake_forbidden_wp_ranges` | `v2x_overtake_forbidden_s_ranges_m` | 周回境界 range 対応。旧 range は移行ツールで変換 |
| `ref_vel_configulator.*.wp_id` | `s_m` | 物理位置が一致することを validator で確認 |
| `v2x_overtake_line_max_target_change` | `v2x_overtake_line_max_target_change_rate_mps` | 旧値は自動換算せず警告。control rate 依存のため手動移行 |
| 固定 gain 100 | `speed_control.kp` 等 | legacy mode と比較後に新 mode を既定化 |

新旧 key が同時指定された場合は新 key を優先して警告する。異なる意味の値を黙って混ぜない。リポジトリ内 YAML の移行とテスト完了後、旧 key 削除は別変更として行う。

## 初期パラメータの扱い

次は公式値ではなく、性能・安全性を比較するための候補である。

```yaml
reference_path:
  resolution: 0.25              # [m], candidate
  circular: true
  smoothing_distance_m: 0.75    # [m], one-sided candidate

mpc:
  horizon_distance_m: 16.0      # [m], candidate
  control_rate: 30.0            # [Hz], candidate
  min_linearization_speed_mps: 0.5
  a_max: 1.0                    # [m/s^2], local WIP safety candidate
```

- 0.25 m / 16 m では N は概ね64になるが、実際の `ds` と上下限から算出する。
- 20 m / 40 Hz は、OSQP を含む control 全体の p99 が周期の80%未満で、deadline miss がない場合だけ比較する。
- 現行の 0.6 m / N=15 / 100 Hz を baseline として同じログ・シナリオを保存する。
- V2X lookahead 25〜30 m は horizon とは別の behavior lookahead として扱い、MPC の変数数を自動で増やさない。

## 受け入れ条件

### 自動テスト

- `wrap_to_pi` が 0、±pi、±5pi、実 trajectory の正負跨ぎを正しく処理する。
- circular 重複終点だけを削除し、非 circular または異なる終点を保持する。
- 0.999 m 区間を 0.25 m 以下の4区間へ分割できる。
- resampled path の seam を含む全区間が `0 < ds <= resolution * (1 + tolerance)` である。
- path の全 `s/x/y/psi/kappa/v/a` と MPC 行列・solver 解が finite である。
- path 総距離が重複終点除去後の canonical path 長から許容誤差を超えて変化しない。
- 周回 seam の wrapped psi 差と kappa 差が validator の異常閾値内である。
- base speed profile が `v^2*abs(kappa) <= ay_max` と縦加減速制約を seam を含めて満たす。
- `init_problem()` を複数回呼んでも `model->wp_id` が変化しない。
- `v_ref=0`、閾値未満、閾値境界で MPC 行列が有限である。
- OSQP の失敗 status / NaN / 制約違反解を拒否し、直線の steering 0 解を受理する。
- control rate を変えても同じ path 距離の V2X prediction time と横 target 変化が許容差内で一致する。
- front / side / rear、周回 seam、ヘアピン近接別区間を `s/d` で分類できる。
- 空 vehicle ID の複数車両が別 track として残り、自車 ID 一致だけが除外される。
- 追い越し復帰の3条件と V2X dropout hold を個別に検証できる。
- 速度、加速度、jerk の step response が設定上限を守る。
- 旧 key、新 key、競合指定、単位変換のテストが通る。

### ビルド・実行

- `make autoware-build` が成功する。
- 対象 package の `colcon test` と `colcon test-result --verbose` が成功する。
- validator が全 domain trajectory を正常終了する。
- `make dev` / `make gate1` / `make gate3` で control topic 周期、NaN / Inf、wall / over penalty の退行がない。
- `make gate2` / `make dev2` 以上で停止車回避、front / side / rear、V2X delay / dropout、追い越し Return を確認できる。
- control p99 が選択した control period の80%未満で、継続的 deadline miss がない。
- `/control/command/control_cmd`、`/localization/kinematic_state`、`/planning/scenario_planning/trajectory` の接続を維持する。
- 提出 tar を作成し、eval build / `make eval`、result JSON、penalty、`output/latest/` を確認する。

## Definition of Done

- 指摘 #1〜#25 の採用部分が実装・自動テスト済みである。
- #20 の workspace 再利用は、実装するか、計測上不要として根拠付きで保留できている。
- #22 の legacy 危険加速度は通常制御から除去され、正式 boost interface の未確定事項が記録されている。
- #26 の既定値は baseline 比較と計算時間測定に基づいて選択されている。
- trajectory は設定 resolution で周期再サンプリングされ、重複終点、ゼロ長、非有限値、異常 seam がない。
- MPC 予測距離、速度プロファイル、solver 判定、V2X 予測・分類、縦制御が各要求を満たす。
- 最終 acceleration command が設定された安全上限を超えない。
- ROS 2 / 評価 / 提出契約を壊していない。
- package README と `docs/spec/mpc-integration.md` に config migration と検証結果が反映されている。
- 実車確認は上記シミュレータ検証後に別途実施する。

## 未確定事項

- 2026 公式 boost topic、message 型、通常加速度との arbitration。
- `/v2x/vehicle_positions` に自車が含まれるか、`vehicle_id` の一意性・自車 ID の取得方法。
- 0.25 m 周期線形補間で実コースの曲率品質を満たせるか、周期 spline が必要か。
- `s_m` と座標から再計算した幾何学的 path 長の差をどの閾値で error / warning に分けるか。
- `SOLVED_INACCURATE` の最大許容制約違反。
- 16 m / 30 Hz と 20 m / 40 Hz の実測計算余裕。
- SingleThreadedExecutor で odom / V2X callback age と control deadline を両立できるか。
- 2026 公式 safety gate と `make gate1/2/3` の完全な対応関係。
