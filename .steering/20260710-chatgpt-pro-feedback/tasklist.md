# ChatGPT Pro 指摘対応 Tasklist

作成日: 2026-07-10  
更新日: 2026-07-10  
状態: Phase 0〜2 First Slice Complete / Simulator Verification Pending

## Definition of Done

- 指摘 #1〜#25 の採用範囲が実装され、対応する自動テストが通る。
- trajectory の重複終点、ゼロ長、過大間隔、非有限値、周回 seam 異常が validator で検出されない。
- path length、resolution、smoothing 幅、MPC horizon、速度・加速度制約が物理単位で確認できる。
- `init_problem()` の再試行で `model->wp_id` が変化しない。
- OSQP status、finite、bounds、constraint violation を確認し、steering 0 の正常解を受理する。
- static speed profile が毎周期一定速度で消されず、縦・横加速度制約を満たす。
- V2X の front / side / rear、空 ID、dropout、追い越し Return を共通 `s/d` 表現で扱える。
- 通常経路から `acc=500.0` と legacy boost / pitstop 分岐が除去され、最終 acceleration command が安全上限を守る。
- control timing の p99 が採用 control period の80%未満で、継続的 deadline miss がない。
- `/control/command/control_cmd` などの ROS 2 契約、Domain、提出物、result JSON、`output/latest/` を壊していない。
- build、package test、validator、gate/dev、submit/eval の証跡を残す。
- package README と `docs/spec/mpc-integration.md` に config migration と最終値を反映する。

## 完了済み: Input / Analysis / Planning

- [x] `chatgpt-pro-feedback.md` に指摘全文を保存する。
- [x] 指摘を27項目へ分割し、優先度、依存関係、受け入れ条件を整理する。
- [x] `mpc_controller_cpp.cpp`、`config.yaml`、CSV、CMake、既存 test を照合する。
- [x] `docs/interface/participant-interface.md` と `evaluation-interface.md` の契約を確認する。
- [x] `docs/spec/mpc-integration.md`、`safety-gates.md`、`competition-rules.md`、`open-questions.md` を確認する。
- [x] 2026-07-10 時点の公式ルールページで「加速度約1.0 m/s²」「SIM boost予定」「ページWIP」を再確認する。
- [x] 現在選択される `final_ver3/traj_mincurv.csv` の基準統計を取得する。
- [x] 指摘内の348.7 m / 最大曲率0.39を固定要件にしないと決定する。
- [x] 各指摘を採用、条件付き採用、一部採用へ分類する。
- [x] `requirements.md` を具体化する。
- [x] `design.md` に architecture、migration、段階導入を記録する。
- [x] 実装・検証 task を本ファイルへ分解する。

## 実装記録: 2026-07-10 First Slice

- C++ path core、strict 7列CSV loader、周回重複終点正規化、validator、C++/Python test登録を追加した。
- `base_wp_id` / `planning_wp_id` を分離し、旧 safety-margin retry を削除した。
- OSQP status、solution finite、全 `A*x` 制約違反を検証し、workspace/CSCをRAII化した。
- solver失敗、stop request、stale/non-finite odometry、non-finite commandを減速・boost無効のfail-safeへ接続した。
- odometryはsteady receipt ageに加え、非ゼロsource stampが更新されない再送も監視する。
- `ceil` helperは実装・テスト済みだが、固定 `N` の実距離退行を避けるためproductionはlegacy `floor`を維持した。
- `make autoware-build`: 25 packages成功。
- package test: C++ 20件、validator contract 1件、Python 17件、合計52 assertions/testsで error/failure 0。
- trajectory CSV 13本: strict circular validatorすべて終了コード0。
- node smoke: odometry欠損時に speed 0、acceleration -1.6 m/s²、steering 0をpublishすることを確認した。
- `pre-commit` はhost・autoware-build containerとも未導入のため未実行。代替として `git diff --check`、XML/YAML parse、Python compileを実行した。
- 未実施: `make dev`、`make gate1/2/3`、多車両、submit/eval。Phase 3以降の周期path、距離horizon、speed/V2X再設計も未実装。

## Phase 0: Baseline と Test Seam

### Baseline evidence

- [x] 作業開始時の `git status --short` と対象ファイル差分を記録する。
- [x] 現行 `resolution=0.6`、`N=15`、`control_rate=100Hz` の実予測距離を計測する。
- [ ] `make dev` で control topic hz、MPC solve 時間、NaN / Inf、wall / over penalty の baseline を保存する。
- [ ] `make gate1`、`make gate2`、`make gate3` の現行結果とログ時刻を保存する。
- [ ] `make dev2` 以上で現行 V2X state transition と追い越し Return の baseline を保存する。

### C++ test infrastructure

- [x] `CMakeLists.txt` に `BUILD_TESTING` と `ament_cmake_gtest` を追加する。
- [x] `package.xml` に test dependency を追加する。
- [ ] pure path / solver / longitudinal / V2X utility を test 可能な内部 library target へ段階分離する。
- [x] 既存 `test_v2x_vehicle_tracker.py` が実際に test 登録されているか確認し、未登録なら追加する。
- [x] package 単位の build/test コマンドを README または本 task の検証記録へ残す。

### Validator skeleton — #24

- [x] strict loader と同じ core を使う `reference_path_validator` CLI を追加する。
- [ ] human-readable table と machine-readable JSON の出力構造を決める。
- [x] 異常時に非0終了する contract test を追加する。
- [ ] `csv_path` と全 `domain_csv_path` を列挙できるようにする。

## Phase 1: 即時の決定性・Solver・Thread 安全化

### Non-mutating waypoint — #8

- [x] `base_wp_id` と `planning_wp_id` を分離する。
- [x] `model->wp_id += effective_wp_id_offset()` を削除する。
- [x] kappa、v_ref、path bounds、V2X、prediction が同じ `planning_wp_id` を使うよう整理する。
- [x] 副作用を複数回進める OSQP safety-margin retry を削除し、1周期1problem/solveへ固定する。
- [ ] `init_problem()` を複数回呼んでも `model->wp_id` が不変なテストを追加する。

### OSQP result validation — #19

- [x] solver 戻り値を solution だけでなく status / violation を持つ構造体へ変更する（timingはPhase 7）。
- [x] `OSQP_SOLVED` と `OSQP_SOLVED_INACCURATE` 以外を拒否する。
- [x] dimension、`allFinite()`、input bounds、`A*x` constraint violation を検証する。
- [x] steering `0.0` を失敗扱いする再試行判定を削除する。
- [ ] setup failure、unsolved、NaN / Inf、bounds 違反、constraint 違反の test を追加する。
- [ ] 直線で steering 0 の正常解を受理する test を追加する。
- [x] 連続 solver failure 時に旧horizonを再生せず、単調減速して速度0へ移る。

### Low-speed model — #13

- [x] `min_linearization_speed_mps` を config に追加し、範囲を検証する。
- [x] 閾値未満で `1/v` と `1/v^2` を生成しない低速 branch を実装する。
- [ ] `v_ref=0`、閾値直下、閾値、直上で行列・解が finite な test を追加する。
- [ ] 停止、SafetyBrake、低速車回避で command spike がないことを確認する。

### Executor / snapshot — #21

- [x] C++ main を `SingleThreadedExecutor` へ変更する。
- [ ] control callback 冒頭で odom / trajectory / config / path / constraints の local snapshot を取る。
- [ ] parameter / trajectory 更新は新 object の検証後に pointer swap する。
- [ ] callback age と control deadline miss のログを追加する。
- [ ] SingleThreaded で topic hz と control period を維持できることを確認する。
- [ ] MultiThreaded が必要になった場合だけ callback group / mutex / immutable snapshot 設計を追加する。

## Phase 2: Angle、CSV、Circular Path の基礎修正

### Angle normalization — #1

- [x] `wrap_to_pi()` を `atan2(sin, cos)` または同等の実装へ変更する。
- [x] angle utility を C++ test から呼べる場所へ分離する。
- [x] 0、±pi、±5pi の test を追加する。
- [x] `3.112 - (-3.003)` と逆方向の wrapped difference test を追加する。
- [x] 全 `wrap_to_pi()` call site の角度向きを確認する。

### Strict trajectory loader — #4

- [x] `ReferencePathPoint` を追加する。
- [x] 必須7列、header 重複、row 列数、完全数値変換を検証する。
- [x] NaN / Inf、非単調 `s_m`、不足点数を拒否する。
- [x] error に file / row / column / value / reason を含める。
- [x] missing header、bad number、partial number、NaN / Inf、short row の test を追加する。
- [x] `load_waypoints()` 用 legacy CSV parser と trajectory strict loader を分ける。

### Duplicate endpoint — #2

- [x] circular かつ closure tolerance 内の末尾を全列同期で削除する。
- [x] 非 circular path と異なる終点は削除しない。
- [x] 重複除去後3点未満を error にする。
- [x] 連続ゼロ長点を検出し、許容する closure duplicate 以外を拒否する。
- [x] configured `final_ver3` CSV で末尾重複が1点除去されることを test する。

### Ceil subdivision — #3

- [ ] `ceil(distance / resolution)` と下限1を使用する。
- [x] resolution <= 0、ゼロ長、極短 segment の扱いを pure helper / strict loader で定義する。
- [x] 0.999 m / 0.25 m が4区間になる test を追加する。
- [ ] closing edge を含む全間隔が resolution 以下になる test を追加する。

## Phase 3: Periodic ReferencePath と Validator

### Canonical arc length — #5

- [ ] unique point の全 edge と closing edge から canonical `s` / loop length を計算する。
- [ ] CSV `s_m` 最終値と幾何長の差を warning / error に分ける閾値を決める。
- [ ] circular helper `wrapped_s`、forward / signed distance、range contains を追加する。
- [ ] seam を跨ぐ synthetic path の unit test を追加する。

### Periodic resampling — #5

- [ ] `M=ceil(L/resolution)`、`ds=L/M`、`s=[0,L)` を実装する。
- [ ] 最初は `periodic_linear` mode を追加する。
- [ ] periodic central difference で `psi/kappa` を再計算する。
- [ ] derivative denominator の異常を0へ黙って置換せず error にする。
- [ ] raw `psi/kappa/vx/ax` を比較用 metadata として保持する。
- [ ] seam を含む spacing、psi、kappa、finite の test を追加する。
- [ ] path length が canonical length から許容誤差を超えない test を追加する。
- [ ] `periodic_cubic` は linear の validator / gate 結果が不足する場合だけ別 task として実装する。

### Smoothing in meters — #6

- [ ] `smoothing_distance_m` を片側距離として追加する。
- [ ] wrapped window と `actual_ds` から half-window point 数を計算する。
- [ ] requested / actual half-width / full-width を起動ログへ出す。
- [ ] 旧 `smoothing_distance` fallback と非推奨警告を実装する。
- [ ] 新旧同時指定時は新 key を優先して警告する。
- [ ] resolution 変更でも物理 smoothing 幅を維持する test を追加する。
- [ ] 0.75 m 候補で最大曲率、path length、wall clearance を baseline 比較する。

### Validator completion — #24

- [x] 点数、総距離、min/max/mean spacing を出力する。
- [x] 最大 `abs(kappa)`、最小半径、最大 wrapped psi difference を出力する。
- [x] seam position / psi / kappa difference を出力する。
- [x] velocity / acceleration range、最大 lateral acceleration を出力する。
- [ ] non-finite、zero-length、over-resolution、curvature spike の場所を出力する。
- [ ] raw / normalized / resampled の比較を出力する。
- [x] 全13 trajectory CSV と domain override 選択先を実行し、終了コード0を確認する。

## Phase 4: Distance-based Horizon と Config Migration

### Horizon — #7

- [ ] `horizon_distance_m`、`horizon_steps_min/max` を config に追加する。
- [ ] ReferencePath の `actual_ds` から N を算出する。
- [ ] clamp 時に warning を出す。
- [ ] path constraints、border cells、current control、prediction、V2X bounds が同じ N を使うよう整理する。
- [ ] requested / actual horizon、N、QP size を起動ログへ出す。
- [ ] 旧 `N` fallback と非推奨警告を追加する。
- [ ] resolution を変えても actual horizon が1区間以内で一致する test を追加する。

### `wp_id` config to `s_m` — #9

- [ ] `lookahead_offset_m` と `low_speed_lookahead_offset_m` を追加する。
- [ ] `v2x_overtake_forbidden_s_ranges_m` を追加し、seam crossing range を扱う。
- [ ] `ref_vel.yaml` の `wp_id` 区間を `s_m` 区間へ移行する。
- [ ] 既存 `wp_id_offset` / low offset / forbidden ranges / ref_vel を migration adapter で読み込む。
- [ ] 全旧設定を物理位置へ変換した migration table を生成する。
- [ ] resolution 変更前後で同じコース位置が選ばれる test を追加する。
- [ ] 周回境界を跨ぐ速度区間・禁止区間 test を追加する。

## Phase 5: Speed Profile、Output Safety、Legacy Cleanup

### Correct periodic speed profile — #10 / #11

- [ ] `base_v_ref` / `base_a_ref` と runtime limit を分離する。
- [ ] global / section / curvature の static upper limit を合成する。
- [ ] `v^2` の周期 forward / backward relaxation を実装する。
- [ ] seam を含む convergence と最大 iteration を実装する。
- [ ] 旧 `(v[i+1]-v[i])/(2ds)` の speed-profile OSQP を production path から除外する。
- [ ] control cycle の全点 `set_v_ref(effective_v_max)` を削除する。
- [ ] runtime domain/start/section/V2X limit を horizon local copy に合成する。
- [ ] v_max / a_max / a_min / ay_max 更新時に profile を再構築・検証・swap する。
- [ ] 直線、単一カーブ、複合カーブ、周回 seam の速度 profile test を追加する。
- [ ] `v^2*abs(kappa)<=ay_max`、前進 `a<=a_max`、後退 `a>=a_min` を全 edge で test する。

### Velocity slew — #12

- [ ] 通常制御でも `last_u_[0]` と実 `dt` による上昇・下降制限を追加する。
- [ ] first cycle、control disable、solver failure、time jump の初期化規則を実装する。
- [ ] 異常 `dt` を bounded `dt` または安全停止へ処理する。
- [ ] step velocity command の rate limit test を追加する。
- [ ] MPC 内 velocity difference constraint は output limiter 安定後の別小変更で追加する。

### Longitudinal controller — #23

- [ ] `speed_control.kp` と `feedforward_enabled` を config 化する。
- [ ] `base_a_ref + kp * velocity_error` を実装する。
- [ ] acceleration / deceleration clamp を適用する。
- [ ] jerk up / down limit と final safety clamp を実装する。
- [ ] filter / jerk 後にも final acceleration が bounds 内であることを test する。
- [ ] saturation / jerk limiting reason を throttled log へ出す。
- [ ] I 項は定常誤差の必要性を計測し、追加する場合は anti-windup / reset test を同時に実装する。

### Legacy cleanup — #22

- [ ] `use_bug_acc_` と `acc=500.0` を通常 MPC control path から除去する。
- [ ] `/aichallenge/pitstop/condition` が現在の2026通常制御に必要か依存調査する。
- [ ] 不要な pitstop subscription と collision-only legacy state を main node から除去する。
- [ ] main MPC node が常に標準 `AckermannControlCommand` を生成するようにする。
- [ ] `/control/command/control_cmd` の最終 publisher が維持されることを launch / topic で確認する。
- [ ] `boost_commander`、custom message、CMake / launch include の依存を一覧化する。
- [ ] 正式2026 boost interface が未確認の間は legacy boost を既定無効かつ通常出力から分離する。
- [ ] 正式仕様確認後に adapter を作る場合、先に `docs/interface/` と migration note を更新する。

## Phase 6: V2X Course Coordinates と Safe Overtake

### Identity / self filter — #16

- [ ] `ego_vehicle_id` の取得元と未設定時の挙動を定義する。
- [ ] 非空 ID を stable track key にする。
- [ ] 空 ID 用の anonymous track counter と nearest-neighbor association を実装する。
- [ ] association の distance / time / jump gate を config 化する。
- [ ] ID 一致 self filter を優先し、距離 filter は fallback のみにする。
- [ ] 空 ID 複数、近接横並び、self ID 一致、track timeout の test を追加する。
- [ ] Python 比較 tracker の空 ID 上書きも必要に応じて同期修正する。

### Shared `s/d` projection — #15

- [ ] `ProjectedOpponent` と segment projection utility を追加する。
- [ ] `s_m`、wrapped `relative_s_m`、lateral、projection error を計算する。
- [ ] 相手速度を path tangent へ射影する。
- [ ] front / side / rear の分類を共通 helper へ統一する。
- [ ] FSM、front risk、gap planner、multi-front、low-speed path が共通投影結果を使うよう移行する。
- [ ] straight、curve、hairpin adjacent segment、seam の projection / classification test を追加する。

### Distance-based prediction — #14

- [ ] horizon の cumulative path distance を計算する。
- [ ] current speed と base reference speed から prediction speed を作る。
- [ ] minimum prediction speed と maximum prediction time を適用する。
- [ ] `index * model.Ts` の予測を削除する。
- [ ] 30 / 40 / 100 Hz で同一 spatial point の prediction time が一致する test を追加する。

### Time-based target change — #18

- [ ] `v2x_overtake_line_max_target_change_rate_mps` を追加する。
- [ ] actual bounded `dt` を用いて max change を計算する。
- [ ] pause / time jump 後の横 target jump を防ぐ。
- [ ] 旧 m/cycle key を警告付きで読み、手動 migration を案内する。
- [ ] 異なる control rate で1秒後の target が一致する test を追加する。

### Return safety / dropout — #17

- [ ] Overtake 開始時に target track ID、pass side、last seen を保持する。
- [ ] `return_clear_distance_m` を対象との `relative_s` 判定へ接続する。
- [ ] side overlap clear 条件を実装する。
- [ ] return-side rear clear distance 条件を実装する。
- [ ] V2X dropout hold time と stale target behavior を実装する。
- [ ] 短時間欠損だけで Return へ入らないようにする。
- [ ] 前方 clear のみ、側方 clear のみ、rear clear のみ、全条件成立の transition test を追加する。
- [ ] dropout 中の Pass / Recovery / Follow と速度制限を test する。

## Phase 7: Timing、Parameter Selection、Optional Optimization

### Timing — #20

- [ ] ReferencePath update、problem build、OSQP setup、solve、validation、control total を計測する。
- [ ] rolling max / p95 / p99 と deadline miss count を出す。
- [ ] `use_stats` の未使用状態を整理し、診断有効化 key を一つにする。
- [ ] timing log 自体の overhead を確認する。

### Candidate comparison — #26

- [ ] Baseline A: 0.6 m / N=15 / 100 Hz を記録する。
- [ ] Candidate B: 0.25 m / 16 m / 30 Hz を記録する。
- [ ] B の path quality、lane keeping、gate、lap / penalty、p99 を比較する。
- [ ] Candidate C: 0.25 m / 20 m / 40 Hz は B に十分な余裕がある場合だけ実行する。
- [ ] smoothing 0.75 m と代替値を curve / wall clearance で比較する。
- [ ] V2X lookahead 25〜30 m を MPC horizon と独立に比較する。
- [ ] 採用値、却下値、根拠を `design.md` 決定記録へ追記する。

### OSQP workspace reuse — #20 optional

- [ ] setup time が有意な bottleneck か確認する。
- [ ] N と sparsity pattern が cycle 間で安定するか確認する。
- [ ] 必要な場合だけ P/A/q/l/u update API を使う prototype を作る。
- [ ] fresh setup と同じ解・status・constraint violation になる regression test を追加する。
- [ ] 改善が小さい、または複雑性が高い場合は根拠付きで保留する。

## Phase 8: Required Automated Test Matrix — #25

- [ ] π境界 angle tests。
- [ ] duplicate endpoint tests。
- [ ] 0.999 m / 0.25 m subdivision test。
- [ ] periodic max spacing test。
- [ ] seam psi / kappa continuity test。
- [ ] strict CSV error diagnostics tests。
- [ ] periodic speed profile a_max / a_min / ay_max tests。
- [ ] repeated `init_problem()` wp_id invariance test。
- [ ] low-speed finite matrix / output tests。
- [ ] OSQP status / finite / bounds / straight zero-steer tests。
- [ ] control-rate-independent V2X prediction test。
- [ ] empty-ID multi-vehicle retention test。
- [ ] `s/d` front / side / rear / seam / hairpin tests。
- [ ] Overtake Return three-condition and dropout tests。
- [ ] time-based target-rate tests。
- [ ] velocity / acceleration / jerk limiter tests。
- [ ] legacy / new config migration tests。
- [ ] all-domain trajectory validator test。
- [ ] `colcon test-result --verbose` で failure がないことを確認する。

## Phase 9: Simulator / Interface Verification

### Build and static checks

- [x] `make autoware-build`。
- [x] 対象 package の `colcon test`。
- [ ] `pre-commit run -a`。
- [x] 全 config / domain trajectory の validator。

### Single vehicle / safety gates

- [ ] `make dev` で `/control/command/control_cmd` の hz と型を確認する。
- [ ] `/localization/kinematic_state` と `/planning/scenario_planning/trajectory` の接続を確認する。
- [ ] `make gate1` で停止時の finite / decel / re-acceleration を確認する。
- [ ] `make gate2` で低速・停止車回避と V2X dropout を確認する。
- [ ] `make gate3` で急カーブ、seam、lane keeping、wall / over を確認する。
- [ ] solver fallback を意図的に発生させ、安全停止とログ理由を確認する。

### Multi vehicle

- [ ] `make dev2` で front / side / rear と追い越し Return を確認する。
- [ ] `make dev3` 以上で anonymous / stable ID、multi-front gap、dropout を確認する。
- [ ] Domain 1..N の分離と `/v2x/vehicle_positions` 以外の cross-domain 通信が増えていないことを確認する。
- [ ] crash / wall / over penalty と lap completion を baseline 比較する。

### Evidence

- [ ] 各実行コマンド、config、git commit、run ID を記録する。
- [ ] `output/latest/d<N>/autoware.log` を保存する。
- [ ] result summary/details、motion analytics、rosbag / mcap を確認する。
- [ ] 問題時刻、solver status、V2X state、control command を対応付ける。

## Phase 10: Documentation、Submission、Completion — #27

- [x] package README に現スライスのReferencePath境界と validator 使用法を書く。
- [ ] package README に新旧 config key と単位の migration table を書く。
- [ ] `docs/spec/mpc-integration.md` に final algorithm、既定値、timing、gate 結果を反映する。
- [ ] 2026未確定 boost / V2X ID 前提を `docs/spec/open-questions.md` と整合させる。
- [x] interface を変更していないことを `docs/interface/participant-interface.md` と再照合する。
- [ ] interface 変更が必要になった場合だけ、コードより先に契約と migration note を更新する。
- [ ] `./create_submit_file.bash` を実行する。
- [ ] tar 最上位が `aichallenge_submit/` であることを確認する。
- [ ] `./docker_build.sh eval --submit submit/aichallenge_submit.tar.gz` を実行する。
- [ ] `make eval` を実行する。
- [ ] result JSON schema、主要 key、`output/latest/`、成果物 owner を確認する。
- [ ] 本 tasklist の Definition of Done を全項目レビューする。
- [ ] 未実装・未検証・TBD を残す場合は、理由、リスク、次の確認条件を明記する。
- [ ] 実車検証は simulator DoD 完了後の別 steering task として開始する。

## 推奨コミット単位

1. `test infrastructure and baseline trajectory validator`
2. `non-mutating waypoint and strict OSQP result validation`
3. `single-threaded executor and immutable control snapshots`
4. `angle wrapping and circular path input fixes`
5. `strict trajectory CSV model and diagnostics`
6. `periodic resampling and metric smoothing`
7. `distance-based horizon and s-parameter migration`
8. `periodic speed profile and low-speed model`
9. `velocity acceleration and jerk safety control`
10. `V2X identity and shared course projection`
11. `V2X distance-time prediction and safe overtake return`
12. `legacy boost separation and standard command cleanup`
13. `timing parameter selection tests and documentation`

各コミットで関連 unit test を同時に追加し、少なくとも対象 package の build/test を実行する。

## Blocked / TBD

- [ ] 2026 正式 boost topic / message / arbitration の公開または運営回答。
- [ ] V2X self ID、vehicle ID 一意性、空 ID の公式保証。
- [ ] `make gate1/2/3` と2026公式 safety gate の対応・合否基準。
- [ ] periodic spline が必要かを決める validator / gate 比較結果。
- [ ] 16 m / 30 Hz、20 m / 40 Hz の p99 timing 結果。
- [ ] `SOLVED_INACCURATE`、path length 差、curvature spike の最終閾値。
