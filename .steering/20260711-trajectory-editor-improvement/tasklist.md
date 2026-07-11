# Trajectory Editor 改善 Tasklist

作成日: 2026-07-11  
更新日: 2026-07-11  
状態: Phase 0〜6 Implemented / Phase 7 Evidence Hold / Simulator Pending

## Definition of Done

このDefinition of Doneは全Phaseの完了条件である。2026-07-11にPhase 0〜6のoffline Editor実装と自動検証を完了した。候補CSVのruntime採用とシミュレータ回帰は、原本/configを自動変更せず後続確認とする。

- [x] `Validate Trajectory` がファイル、working data、Undo履歴を変更せず、構造化issueと統計を表示する。
- [x] `Normalize Geometry` がcandidate上で重複・退化区間・arc length・再サンプリング・幾何量を処理し、適用前に比較できる。
- [x] `Recompute Speed Profile` が速度・縦加減速・横加速度・seam制約を満たし、非収束・実現不能時は保存しない。
- [x] MPC出力はcanonical 7列順でC++ validatorを通る。
- [x] Pure Pursuitの8列形式、2つのEditorコマンド、V2X Editorの既存importが維持される。
- [x] 通常保存は別名を提示し、上書き確認、保存前検証、原子的置換、未保存変更確認が機能する。
- [x] before/afterの7表示、issue同期選択、補正レポートが処理結果と一致する。
- [x] 0.25 m、1.0 m/s²、16 mが公式値ではなくCandidateとして表示される。
- [x] Editor速度が現状のruntime MPC走行へ直接反映されない制限がUI・README・仕様に記載される。
- [ ] 自動テスト、C++ validator、シミュレータ回帰の証跡を本ファイルへ記録する。

## 完了済み: Input / Analysis / Steering

- [x] ChatGPT Proの指摘全文を `chatgpt-pro-feedback.md` に保存する。
- [x] 指摘をValidate、Normalize、Speed、Preview、Save、Presetへ分割する。
- [x] 現行Editor本体、2つのlauncher、CMake、READMEを確認する。
- [x] MPC 7列CSVとPure Pursuit 8列CSVを確認する。
- [x] C++ `path_core`、`reference_path_validator`、既存testを照合する。
- [x] Python EditorとC++のclosure tolerance差を特定する。
- [x] 現行Saveの直接上書きと暗黙geometry再計算を確認する。
- [x] `v2x_position_editor.py` のimport依存を確認する。
- [x] Editor生成 `vx/ax` が現行C++ MPC走行へ直接反映されないことを確認する。
- [x] 既存MPCステアリングPhase 3〜5との所有範囲を分離する。
- [x] participant/evaluation interfaceとの互換性を確認する。
- [x] 指摘を採用、条件付き採用、対象外へ分類する。
- [x] `requirements.md` に要求、受け入れ条件、未確定事項を記録する。
- [x] `design.md` にarchitecture、data flow、数値方式、保存、rollbackを記録する。
- [ ] 要求・設計内容を利用者レビューで確定する。

## Phase 0: Baseline と Test Seam

### Baseline

- [x] 現行EditorでMPC CSVをopen/save-asし、byte diffと変更列を記録する。
- [x] 現行EditorでPure Pursuit CSVをopen/save-asし、byte diffと変更列を記録する。
- [ ] manual drag、insert、delete、smooth、undo、saveの現行挙動を記録する。
- [x] active `final_ver3/traj_mincurv.csv` のPython/C++ validator基準統計を保存する。
- [x] 元CSVのSHA-256を記録し、テストで原本を更新しないようにする。

### Test infrastructure

- [x] Editor pure core用のtest module構成を決める。
- [x] `CMakeLists.txt` へ新pytestを登録する。
- [x] temporary directoryだけへ保存するfixtureを用意する。
- [ ] C++/Pythonで共有するgolden CSV fixture directoryを作る。
- [x] valid/invalid fixtureごとのaccept/reject matrixをtest parameterとして記録する。
- [x] GUIなしのhost Pythonとpackage/container Pythonの両方でpure testsを実行できるか確認する。

## Phase 1: Data Model と Validate Core — P0

### Document / format model

- [ ] `TrajectoryFormat`、typed record、dataset、document、revisionを追加する。
- [x] original / working / candidateを分離する。
- [x] geometry / speed dirty flagを追加する。
- [x] circularをduplicate endpointと別の明示stateにする。
- [x] `--circular` / `--open` とpreset既定の扱いを実装する。
- [x] stale candidateをrevision比較で拒否する。

### Format adapters

- [x] raw CSVのheader、row、1始まりline numberを扱う非破壊parserを追加する。
- [x] MPC adapterへcanonical 7列schemaを実装する。
- [x] Pure Pursuit adapterへ既存8列schemaを実装する。
- [x] canonical serializeと既存Pure Pursuit列順維持を実装する。
- [x] existing `load_trajectory` 呼出互換を維持するadapterを用意する。

### Structured validation

- [x] `ValidationIssue` のcode、severity、line、point/segment、`s_m`、column、valueを実装する。
- [x] header欠落・余分・重複、row列数、空値、部分数値を検出する。
- [x] NaN / Infとderived metric overflowを検出する。
- [x] 厳密増加 `s_m`、最低点数、minimum segmentを検出する。
- [x] closure duplicateとinternal duplicate/degenerate segmentを区別する。
- [x] min/max/mean spacingとclosing-edge spacingを計算する。
- [x] wrapped `delta psi`、`abs(kappa)`、`delta kappa`を計算する。
- [x] velocity、acceleration、lateral accelerationを計算する。
- [x] 任意の `v_max/a_max/a_min/ay_max` 違反を検出する。
- [x] validationが入力objectを変更しないtestを追加する。
- [x] issue順序とmetricsのdeterminism testを追加する。

### Cross-language parity

- [x] canonical headerをPythonとC++で照合する。
- [x] closure tolerance `<= 1e-3 m` の境界testを追加する。
- [x] minimum segment `1e-6 m` の境界testを追加する。
- [x] pi seamを両実装で同じ向きにwrapするtestを追加する。
- [ ] golden fixtureをPython validatorとC++ loader/validatorへ通す。

## Phase 2: Validate UI と Safe Save — P0

### Validate UI

- [x] toolbarへ独立した `Validate` 操作を追加する。
- [x] issue一覧を `ttk.Treeview` で表示する。
- [x] code、severity、line、`s_m`、value、message列を表示する。
- [x] summary metricsとerror/warning/info件数を表示する。
- [x] schema error時も可能なissueを一覧化し、現在のdocumentを置換しない。
- [x] Validate前後でworking、Undo、dirty state、file hashが不変なtestを追加する。

### Dirty-state safety

- [x] drag / insert / delete / smoothでgeometry dirtyを設定する。
- [x] 点数またはopen/circular topology変更でspeed dirtyを設定する。
- [x] dirty derived fieldsをstatus barへ表示する。
- [x] dirty errorを解消せずにMPC strict saveできないようにする。
- [x] Pure Pursuit orientationの明示再計算フローを実装する。

### Save safety

- [x] `Save As` を通常動線にし、Phase 2では `_edited` suffixを提案する。
- [x] `Save` または既存file選択時にpathと変更概要を含む上書き確認を出す。
- [x] 保存前にformat別final validationを行う。
- [x] temporary fileへの完全書込とatomic replaceを実装する。
- [x] temporary fileを再parseしてからreplaceする。
- [x] write/validation/replace失敗時に既存fileを不変にするtestを追加する。
- [x] overwrite cancelで元fileがbyte不変なtestを追加する。
- [x] open / exit時の未保存変更確認を追加する。
- [x] 保存処理から暗黙の `recompute_on_save` を除去する。

## Phase 3: Normalize Geometry Core — P1

### Circular topology / cleanup

- [x] closure duplicateを `<= 1e-3 m` で検出する。
- [x] 選択された場合だけ末尾レコード全体を削除する。
- [x] internal degenerate segmentを列挙し、選択された場合だけ決定的に削除する。
- [x] cleanup後のopen/circular最低点数を確認する。
- [x] duplicate削除後もdocument.circularを維持する。

### Canonical arc length

- [x] unique pointの全edgeとclosing edgeからpath lengthを計算する。
- [x] open/circularのwrapped `s` helperを実装する。
- [ ] CSV `s_m` とcanonical geometry長の差をreportする。
- [x] seamを跨ぐsynthetic path testを追加する。

### Uniform resampling

- [x] circularの `M=ceil(L/resolution)`、`ds=L/M` を実装する。
- [x] openの `segments=ceil(L/resolution)`、endpoint保持を実装する。
- [x] polyline上のlinear interpolationを実装する。
- [x] seamを含む全spacingがresolution以下になるtestを追加する。
- [x] `s_m`厳密増加、finite、path length差のtestを追加する。

### Geometry regeneration

- [x] circular central differenceで `psi/kappa` を再計算する。
- [x] open endpointのone-sided方式を実装・文書化する。
- [x] derivative denominator異常をerrorにする。
- [x] 直線、既知半径円、pi跨ぎ閉路のtestを追加する。

### Velocity metadata policy

- [x] 点数不変時の `vx/ax` preserve modeを追加する。
- [x] 点数変更時のopen/periodic metadata interpolationを追加する。
- [x] metadata mode未選択時はcandidate生成を拒否する。
- [x] 既存rowの単純copyで新点を作らないtestを追加する。

### Candidate transaction

- [x] Normalize parameter dialogを追加する。
- [x] checkboxごとの実行項目をcandidateへ記録する。
- [x] candidate validation error時にApply/Saveをdisableする。
- [x] ApplyでUndo snapshot、revision increment、dirty更新を行う。
- [x] Cancelでdocumentが不変なtestを追加する。

## Phase 4: Before / After Preview と Report — P1

### Plot model

- [x] XY、spacing、psi、kappa、velocity、acceleration、lateral accelerationのplot seriesを生成する。
- [x] x-axisを原則 `s_m` とし、単位とlegendを付ける。
- [x] before/candidateで点数が違う場合のselection mappingを定義する。
- [x] issue、XY、scalar graphで共有するselection modelを追加する。

### Tk UI

- [x] XYへbefore/candidate overlayを追加する。
- [x] scalar graph tabsを追加する。
- [x] issue選択で該当point/segmentとgraph cursorを更新する。
- [x] graph選択でissue/XYを逆同期する。
- [x] 大きなcandidate生成時にUIをblockしない実行方式を確認する。
- [x] worker使用時はrevision照合とmain-thread UI更新を実装する。

### Transformation report

- [x] 元/補正後点数、総距離、削除数を表示する。
- [x] spacing、kappa、velocity、acceleration、lateral acceleration極値を表示する。
- [x] operation parameter、warning/error、constraint violation数を表示する。
- [x] 最大position displacementとpath length差を表示する。
- [x] report値とpure metricsが一致するtestを追加する。

## Phase 5: Recompute Speed Profile — P2

### Parameter contract

- [x] `v_max > 0`、`a_max > 0`、`a_min < 0`、`ay_max > 0` を検証する。
- [x] `minimum_speed >= 0` とcircular stateを検証する。
- [x] invalid / non-finite parameterをcandidate作成前に拒否する。
- [x] open start/end speedは仕様確定までoptionalとして扱う。

### Algorithm

- [x] curvature upper limitを生成する。
- [x] forward acceleration relaxationを実装する。
- [x] backward deceleration relaxationを実装する。
- [x] circular seamを含め収束まで反復する。
- [x] convergence toleranceとmaximum iterationを設定化する。
- [x] maximum iteration到達をerrorにする。
- [x] `minimum_speed` が上限と競合する場合をinfeasibleにする。
- [x] outgoing edge定義で `ax_mps2` を再計算する。
- [x] open path最終 `ax=0.0` を実装する。
- [x] 独立したpost validationを実行する。

### Tests / UI

- [x] 直線、一定曲率、複合曲率のvelocity cap testを追加する。
- [x] 全edgeのforward/backward制約testを追加する。
- [x] circular seam収束testを追加する。
- [x] infeasible / non-convergent testを追加する。
- [x] Speed parameter dialog、candidate、preview、reportを追加する。
- [x] `Offline profile; runtime consumption pending` をUIへ表示する。

## Phase 6: Candidate Preset — P2

- [x] `AI Challenge 2026 Candidate - Safe` を追加する。
- [x] `resolution=0.25 m` と `a_max=1.0 m/s²` を編集可能にする。
- [x] `horizon_distance=16 m` をread-only integration hintにする。
- [x] 値が公式仕様ではない旨を画面とREADMEへ表示する。
- [x] `v_max/a_min/ay_max/minimum_speed` を根拠なく固定しない。
- [x] Development / Race presetは実測根拠ができるまで追加しない。

## Phase 7: Optional Periodic Spline / Smoothing — P3

- [ ] periodic linear baselineのspacing、path length、curvature、最大変位を保存する。
- [ ] spline方式と依存ライブラリを決める。
- [ ] seamの位置・1次・2次微分連続性testを追加する。
- [ ] curve spike指標と閾値を決める。
- [ ] smoothing幅をpoint数でなく物理距離で定義する。
- [ ] smoothing後の最大position displacementを制限する。
- [ ] Lanelet rail / wall clearanceの比較方法を決める。
- [ ] periodic linearとC++ validator、`make gate3`結果を比較する。
- [ ] 改善証跡がない場合は実装または有効化を保留する。
- [ ] 採用時も既定OFFから開始する。

## Phase 8: Compatibility / Documentation / Integration

### Compatibility

- [x] `trajectory_editor` のdefault MPC pathとCLIを維持する。
- [x] `pure_pursuit_trajectory_editor` のpresetと8列保存を維持する。
- [x] `Point`、`_default_paths`、`load_osm_rails`、`load_trajectory` のimport互換を確認する。
- [x] package install後に両launcherのsmoke testを行う。
- [x] ROS topic/service/Domain、launch entry、result schemaに差分がないことを確認する。

### Documentation

- [x] package READMEへ3操作、UI、schema、safe saveを追記する。
- [x] Candidate値が公式値でないことをREADMEへ追記する。
- [x] Phase 0〜2のUI、schema、safe save、利用手順をpackage READMEへ追記する。
- [x] Editor速度がruntimeへ未反映であることをREADMEへ追記する。
- [x] `docs/spec/mpc-integration.md` へoffline/runtime境界を追記する。
- [x] interface契約を変更していないため `docs/interface/*` の値を変更しないことを確認する。

### Build / automated verification

- [x] `make autoware-build` を実行する。
- [x] package pytestを実行する。
- [x] `test_path_core` とPython/C++ validator contract testを実行する。
- [x] 全MPC trajectory CSVをC++ validatorへ通す。
- [x] Pure Pursuit regression testを実行する。
- [x] `git diff --check`、`py_compile`、host/container testを実行する。

### Simulator verification

- [x] 原本を変更せず、候補CSVを別名で作成する。
- [x] candidate validation reportとC++ validator結果を保存する。
- [ ] configで候補CSVを明示選択し、`make dev` を実行する。
- [ ] 最初のヘアピン、1周完了、NaN / Inf、MPC failureを確認する。
- [ ] 適切な `make gate*` でlane keepingとpenalty退行を確認する。
- [ ] runtime speed反映は既存MPC Phase 5完了前には合格判定しない。
- [ ] 実車確認はシミュレータ検証後の別作業とする。

## 推奨実装単位

1. `trajectory validation core and golden fixtures`
2. `non-destructive validation UI`
3. `safe save and dirty-state handling`
4. `periodic linear geometry normalization core`
5. `candidate preview, plots, and transformation report`
6. `offline periodic speed profile`
7. `candidate preset and documentation`
8. `optional periodic spline after evidence`

各単位で対応testを同時に追加し、数値処理を最後にまとめてテストしない。

## 未確定事項の解消タスク

- [ ] curve spikeの式、近傍幅、warning/error閾値を決める。
- [ ] seam psi/kappa、path length差、最大変位の閾値を決める。
- [x] metadata補間とspeed再計算のEditor内選択条件を決める。
- [x] speed convergence toleranceとmax iterationの編集可能なローカル既定を決める。
- [ ] open boundary speedの要否を決める。
- [x] Tk Canvas graph modelと双方向selectionをprototypeで確認する。
- [ ] sidecar report JSONの要否を決める。
- [ ] GUI保存時のC++ validator実行を必須にするか決める。

## 検証記録

### 2026-07-11 Steering作成

- 実施: 指摘原文、現行Editor、C++ contract、validator、interface docs、既存MPC steeringの照合。
- 結果: offline Editor変更に閉じる限りinterface互換。
- 結果: MPC 7列とPure Pursuit 8列のformat分離が必須。
- 結果: closure toleranceはPython `<1e-5` からC++ `<=1e-3` へ揃える必要あり。
- 結果: 現行の直接上書き・暗黙geometry再計算はsafe save要求と不一致。
- 結果: Editorのspeed profileは現状runtime走行へ直接反映されない。
- 未実施: 当時は実装、build、pytest、GUI、validator再実行、シミュレータ。

### 2026-07-11 Phase 0〜2 実装

- 実装: GUI非依存 `trajectory_contract.py`、MPC 7列 / Pure Pursuit 8列validator、構造化issue、統計、optional limit、atomic CSV writer。
- 実装: `Validate` UI、issue選択、明示circular state、dirty/revision/Undo、明示 `Recompute Geometry`、`*_edited.csv` Save As、上書き・未保存確認。
- 実装: raw CSVとtemporary再parseをC++ loader基準へ寄せ、raw末尾極短区間、subnormal、端点差overflowをPython/C++統合testで固定。
- Baseline: 旧MPC saveは `recompute=False` でbyte同一、`recompute=True` で76行相当の差分。旧Pure Pursuit saveは暗黙再計算・書式変更により264行相当の差分。
- 原本保護: active `final_ver3/traj_mincurv.csv` SHA-256は作業前後とも `a31753f75dfe9913ea017a9868228f766efe4b591d98e3be5e8caa63720824f2`。
- Python基準: active MPCは350 raw / 349 normalized、error 0、legacy duplicate warning 1、total 320.1873164 m、spacing 0.5656085〜1.5196783 m、seam 0.9646175 m。
- C++基準: 同じactive MPCを `reference_path_validator --circular` へ通しexit 0。全10本の `env/**/traj*.csv` もexit 0。
- Host test: `python3 -m pytest -q test/test_trajectory_contract.py test/test_trajectory_editor_compat.py` は59件成功。
- Build: `make autoware-build` は25 package成功。既存のheader install warningとsetuptools deprecation warningのみ。
- Package test: `colcon test --packages-select multi_purpose_mpc_ros` は105 tests、error 0、failure 0、skip 0。
- GUI smoke: install後のMPC presetはerror 0 / warning 1、Pure Pursuit presetはerror 0 / warning 0でTk UI初期化成功。
- Compatibility: 2 launcherの`--help`、V2X Editorの4 import、ROS topic/service/Domain/launch/result schema無変更を確認。
- Static check: `py_compile` と `git diff --check` 成功。
- Independent re-review: 初回指摘のPython/C++不一致とUI state不具合は解消済みで、Phase 0〜2のrelease blockerなし。
- Known safe difference: Editorは10進表記に限定しhex floatを安全側で拒否する。C++ validatorの任意 `--resolution` / 5%判定はPhase 3のresampling検証へ残す。
- 未実施: Phase 3以降のNormalize/Preview/Speed、候補CSVでの `make dev` / `make gate*`、実車確認。

### 2026-07-11 Phase 3〜6 実装

- 実装: `trajectory_processing.py` に重複・退化点cleanup、open/circular等間隔線形resampling、canonical `s/psi/kappa`、preserve/interpolate/recompute metadata policy、TransformationReportを追加。
- 実装: `trajectory_speed.py` に曲率速度上限、forward/backward relaxation、circular seam反復、minimum-speed infeasible、非収束、outgoing `ax`、独立post-validationを追加。
- 実装: `trajectory_plot.py` / `trajectory_preview.py` に7表示、同一sのpolyline補間による最大変位、等倍XY、双方向issue同期、比較表、parameter dialog、responsive workerを追加。
- Safety review対応: MPCの全XY変更でspeedをstale化、Apply直前のrevision・content signature・validation再確認、invalid candidate previewのApply無効化、repair可能な`s/psi/kappa`・重複点CSVのNormalize導線、周回spacing `>1e-3 m` guardを追加。
- Active dry-run: 原本350点から0.25m candidate 1281点を `/tmp/traj_mincurv_normalized.csv` へ生成。spacing 0.247682〜0.249951m、Python error/warning 0、処理約49ms。
- Speed dry-run: `v_max=5.0, a_max=1.0, a_min=-2.0, ay_max=2.0` で2反復、約41ms。velocity 1.36055〜5.0m/s、acceleration -2.0〜1.0m/s²、max lateral acceleration 2.0m/s²、error/warning 0。
- C++ validator: normalized / speed-profiledの両候補を `--circular --resolution 0.25` でexit 0。raw/normalized 1281、zero-length 0、over-resolution 0。
- 原本保護: active CSV SHA-256は生成前後とも `a31753f75dfe9913ea017a9868228f766efe4b591d98e3be5e8caa63720824f2`。
- Host test: trajectory関連5 moduleは122件成功。`py_compile` と `git diff --check` 成功。
- Build: `make autoware-build` は25 package成功。既知のheader install / setuptools warningのみ。
- Package test: `colcon test --packages-select multi_purpose_mpc_ros` は182 tests、error 0、failure 0、skip 0。
- Independent re-review: normalize、plot/preview、speed integrationの3観点で再確認し、release blockerなし。指摘された複合repair error、実seam chord、変位算出、speed stale連鎖、candidate改変、UI例外経路は回帰test付きで修正。
- Compatibility: 両Editor launcherの`--help`成功。ROS topic/service/Domain、launch entry、result schema、runtime config、active CSVは変更していない。
- GUI: Tk 8.6 importとGUI非依存model/integration testは成功。現セッションにはX server / `xvfb-run`がないため、Phase 3〜6 dialogの実window smokeは未実施。
- 未実施: candidateをconfigで選ぶ `make dev` / `make gate*`。offline candidateをactive runtime pathへ自動採用しない方針のため、利用者が候補を選んだ後の回帰確認とする。
