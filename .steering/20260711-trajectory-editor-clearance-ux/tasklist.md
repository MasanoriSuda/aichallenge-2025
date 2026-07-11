# Trajectory Editor Clearance / UX 改善 Tasklist

作成日: 2026-07-11  
更新日: 2026-07-11  
状態: Core Implementation Complete / Deferred Requirements・GUI・Simulator Verification Pending

## Definition of Done

- [x] `Validate Clearance` が非破壊で車体・margin・wall contactを検証する。
- [x] point footprintと連続pose間swept footprintの両方を検証する。
- [x] `Adjust Clearance` がdetached candidateを生成し、安全再検証後だけApplyできる。
- [x] infeasible区間を安全と誤表示せず、workingを変更しない。
- [x] loaded Original、Working、Candidateをmain canvasで独立表示できる。
- [x] Save後もloaded Originalが変わらない。
- [x] 水平・垂直scrollbarでzoom後も全コースへ移動・編集できる。
- [x] scroll後もpoint選択、drag、insert、issue navigationの座標がずれない。
- [x] active CSV、map、vehicle YAML、runtime config、ROS / 評価契約を変更しない。
- [ ] pure tests、package tests、C++ validator、GUI、sim / gateの証跡を記録する。

## Steering / Analysis

- [x] 改善提案を壁clearance、原本比較、navigationへ分割する。
- [x] 先行 `20260711-trajectory-editor-improvement` から別ステアリングへ切り分ける。
- [x] navigation方式を水平・垂直scrollbarとするユーザー決定を記録する。
- [x] 現行Editorにloaded original snapshotがあることを確認する。
- [x] 現行Editorに右・中drag panがあることを確認する。
- [x] final_ver3にoccupancy grid YAML / PGMがあることを確認する。
- [x] vehicle info YAMLにwheel base、tread、overhangがあることを確認する。
- [x] `requirements.md` を作成する。
- [x] `design.md` を作成する。
- [x] requirements / designをユーザーと確定する。

## Phase 0: Contract / Baseline

### Map and vehicle contract

- [x] final_ver3 trajectoryとoccupancy gridのworld座標範囲を比較する。
- [ ] Lanelet2 railとoccupancy occupied boundaryの差を可視化・計測する。
- [x] offline判定のwall sourceとしてoccupancy gridを採用する。
- [ ] MPC/AWSIM trajectory poseがrear axle、`base_link`、車体中心のどれか確認する。
- [ ] `vehicle_info.param.yaml` の寸法とAWSIM collider外形の対応を確認する。
- [x] unknown cellは既定occupied、map外は常にerrorとする。
- [x] margin既定値を0 mとし、公式値でないことを明記する。

### Baseline evidence

- [x] active CSVとver5 CSVのhash、点数、boundsを記録する。
- [x] existing zoom、Fit、右・中pan、選択、drag、insertの挙動を記録する。
- [x] Save / Save As前後の `original_trajectory` lifecycleをtestで固定する。
- [x] main canvasとpreview canvasのnavigation差を記録する。
- [x] active CSV、map、vehicle YAMLを変更しないtemporary fixture方針を決める。

### Test seam

- [x] `test_trajectory_clearance.py` を追加する。
- [x] synthetic PGM/YAML fixtureをtemporary directoryへ生成する。
- [x] canvas transformをGUIなしでtestできる構造にする。
- [x] CMakeへ新pytestを登録する。

## Phase 1: Horizontal / Vertical Scrollbars — P0

### Canvas structure

- [x] Canvasをgrid配置のframeへ移し、水平scrollbarを追加する。
- [x] 垂直scrollbarを追加する。
- [x] scrollbar commandを既存center/scale viewport controllerへ接続する。
- [x] visible layer boundsとpaddingからfinite scrollregionを作る。
- [x] empty / extremely large boundsを安全に処理する。

### Coordinate transform

- [x] world-to-screen / screen-to-world helperをscrollbar viewportと同期する。
- [x] custom viewport offsetをevent座標変換へ反映する。
- [x] point選択、segment選択、drag、insertを同じ変換で維持する。
- [x] OSM、Original、Working、Candidate、clearance overlayを同じ変換で描画する。

### Navigation

- [x] zoom前後でcursorまたはviewport centerのworld位置を保持する。
- [x] Fitでscrollregion全体を表示する。
- [x] `Center Selection` を追加する。
- [x] issue選択からCenter Selectionを呼べるようにする。
- [x] 右・中drag panをscrollbarと同期する。
- [ ] horizontal / vertical wheel policyを明記する。

### Tests

- [x] scroll後のworld-screen-world round trip test。
- [ ] viewport edge / clamp test。
- [x] zoom anchor test。
- [ ] Fit / Center Selection test。
- [ ] scroll後のselect / drag / insert回帰test。

## Phase 2: Original / Working / Candidate View — P0

### State separation

- [x] `loaded_original` をOpen時に作成し、次のOpenまで不変にする。
- [x] dirty lifecycleに既存stateを使い、別`saved_baseline`を不要とする。
- [x] Save / Save Asでloaded originalを更新しない。
- [x] Undoがloaded originalへ影響しないことを確認する。
- [x] Open失敗時に現Original / Workingを維持する。

### Layer UI

- [x] Original / Working / Candidate checkbuttonを追加する。
- [x] gray / blue / orangeと線種の凡例を追加する。
- [x] Originalを編集対象から除外する。
- [x] Candidateがない場合の表示状態を明確にする。
- [x] layer toggleをview-only stateとしrevision/dirtyを変えない。

### Difference model

- [x] same-s interpolationでOriginal / Working差分を作る。
- [x] point count差、path length差、最大・平均変位を表示する。
- [x] 連続変更区間を`s_m`範囲で一覧化する。
- [x] stale original geometry用のdetached display arcを作る。
- [ ] `Reset Working to Original` を実装する場合は確認・Undoを追加する。

### Tests

- [x] Save後もloaded originalがbyte/object不変。
- [ ] 別trajectory Open時だけloaded originalを置換。
- [x] 点数違いのOriginal / Working mapping。
- [x] layer toggleのread-only性。
- [ ] Reset cancel / apply / Undo。

## Phase 3: Clearance Validation — P1

### Occupancy adapter

- [x] YAML schema、relative image path、threshold、negateをparseする。
- [x] P2 / P5 PGMとcommentをparseする。
- [x] occupied / free / unknownを分類する。
- [x] final_ver3のbinary/yaw 0条件でC++ MPCの画像正規化、Y反転、threshold、小occupied component除去とparityを取り、一般mapとの差を明記する。
- [x] world-grid-image座標変換とY軸反転を実装する。
- [x] origin yawを扱う。
- [x] map content signatureを追加し、Apply時に再読込する。
- [x] malformed / unsupported mapを安全側の明示errorにする。
- [x] Clearance coreをOGM専用にし、OSM lon/lat fallbackを使用しない。

### Vehicle adapter

- [x] vehicle info YAMLから寸法を読む。
- [x] reference pointをrear-axle暫定値として明示設定する。
- [x] front/rear/left/right margin入力を追加する。
- [x] manual overrideと暫定値sourceを表示する。
- [x] finite、non-negative、外形整合を検証する。

### Geometry / validation

- [x] oriented footprint cornerを生成する。
- [x] rectangle edge / interiorとoccupied cellの交差を検出する。
- [x] minimum raw / conservative clearanceを計算する。
- [ ] 左右別clearanceを計算する。
- [x] translation / heading条件からsweep補間数を決める。
- [x] open/circularの全segmentをsweep検証する。
- [x] collision、margin、unknown、map外を区別する。
- [ ] frame mismatch専用issueを追加する（現状はmap外errorで安全側停止）。
- [x] map resolution由来の測定誤差をreportする。

### UI / report

- [x] `Vehicle / Margin Settings` dialogを追加する。
- [x] `Validate Clearance` buttonを追加する。
- [x] selected point / issueの車体矩形とmarginを描画する。
- [x] occupied/unknown危険cellを強調する。
- [x] clearance issue treeとCenter Selectionを接続する。
- [x] 最小raw/conservative clearance、危険point/segment、map resolutionをsummary表示する。
- [x] clearance stateをnot-run / running / safe / unsafe / failed / staleでstatus表示する。

### Tests

- [x] straight wallとrectangle collision。
- [x] corner / edge / interior contact。
- [x] margin violationとbody collisionの分離。
- [x] heading rotationとpi seam。
- [x] swept collisionのみ発生する区間。
- [x] unknown / map外。
- [x] pure validationが入力pose / mapを変更しないことを確認する。

## Phase 4: Clearance Adjustment Candidate — P1

### Parameters / feasible set

- [x] max lateral shiftを追加する。
- [x] offset sampling stepを追加する。
- [x] displacement / smoothness / curvature weightを追加する。
- [ ] 補正対象`s_m`範囲を選択可能にする。
- [x] 各点の左右safe offset候補を列挙する。
- [ ] map resolutionに対して探索stepが不適切な場合を拒否する。

### Optimizer

- [x] deterministic dynamic programmingを実装する。
- [x] original deviationとoffset discontinuityを抑制する。
- [x] circular seamでoffset continuityとduplicate endpointを維持する。
- [x] max shiftを超えない。
- [x] feasible候補がない区間をinfeasibleにする。
- [x] candidate XYとgeometryを再生成する。

### Candidate transaction

- [x] map、vehicle、options、revision signatureをcandidateへ記録する。
- [x] before / after clearance reportを作る。
- [x] Original / Working / Candidateをoverlayする。
- [x] 最大変位、最小clearance、曲率をPreview表示する。
- [x] unsafe / infeasible時はApply可能なcandidateを生成しない。
- [x] Apply直前に全signatureとcontentを再確認する。
- [x] Applyを1回のUndo snapshotにする。
- [x] Apply後はgeometry valid、MPC speed dirtyとして設定する。
- [x] guard拒否でworking / Undo / revision不変testを追加する。

## Phase 5: Compatibility / Documentation / Verification

### Compatibility

- [x] existing Validate / Normalize / Speed / Saveを回帰する。
- [x] Pure Pursuit 8列、Original view、scrollbarを回帰する。
- [x] V2X Editor import互換をpackage testで確認する。
- [x] ROS topic/service/Domain、launch、result schema無変更を確認する。
- [x] active CSV、map、vehicle YAML、configがGit差分なしであることとblob hashを確認する。

### Documentation

- [x] package READMEへclearance、3-layer表示、scrollbar操作を追記する。
- [x] `docs/spec/mpc-integration.md` へoffline wall-checkの責務と制限を書く。
- [x] vehicle / margin値が公式値でないことを明記する。
- [x] occupancy grid分解能とwall接触保証の限界を書く。
- [x] runtime collision avoidanceではないことを明記する。

### Automated verification

- [x] host pure tests（181 passed）。
- [x] `make autoware-build`。
- [x] `colcon test --packages-select multi_purpose_mpc_ros`（226 tests, 0 failures）。
- [x] `py_compile` / `git diff --check`。
- [x] Editor出力のPython/C++ validator contract。
- [x] 2つのEditor launcherとV2X import smoke。

### GUI / simulator

- [ ] 実Tk画面で2本のscrollbar、zoom、Fit、Center Selectionを確認する。
- [ ] Original / Working / Candidateの表示切替を確認する。
- [ ] clearance overlayとissue navigationを確認する。
- [ ] 別名candidateを明示選択して `make dev`。
- [ ] 最初のヘアピン、1周、wall contact、NaN / Inf、MPC failureを確認する。
- [ ] baselineと適切な `make gate*` 結果を比較する。

## 実装順序

1. Map / vehicle / pose referenceの契約確定。
2. Scrollable canvasと座標回帰。
3. loaded Originalと3-layer表示。
4. Clearance read-only validation。
5. Clearance adjustment candidate。
6. Documentation、GUI、sim / gate。

Clearance adjustmentより前にscrollbarとOriginal表示を完成させ、補正結果を確認できない状態で自動補正を公開しない。

## 未確定事項の解消タスク

- [ ] occupancy grid / Lanelet2 / AWSIM colliderの壁位置を比較する。
- [ ] trajectory pose referenceをコード・URDF・走行markerで確認する。
- [x] unknown / map外policyをsynthetic fixtureとfinal_ver3 mapで評価する。
- [x] ローカル暫定sweep stepを0.05mとし、連続sweepの保守的包含を実装する（公式・実機上限は未確定）。
- [ ] margin candidate値と根拠を決める。
- [ ] max shift / offset step / optimizer weightをsynthetic corridorで決める。
- [ ] curvature・最大変位のApply上限を決める。
- [ ] sidecar clearance reportの要否を決める。

## 検証記録

### 2026-07-11 Steering作成

- ユーザー提案をclearance、Original比較、scrollbar navigationへ整理した。
- zoom後の移動方法は水平・垂直scrollbarとすることを決定した。
- final_ver3 occupancy gridは751×759px、resolution 0.1m、originをYAMLで保持することを確認した。
- active `traj_mincurv.csv`は350点、Git blob `c0f804e853e796b4ed380a772e544777ffd9328f`、x=[89612.816738, 89679.451863]、y=[43120.955345, 43187.367968]だった。`traj_mincurv_ver5.csv`は350点、blob `af5b0b8090bc7c13ca29320bad99b76f05dbd6fe`、x=[89612.960464, 89678.991431]、y=[43122.301878, 43187.083163]だった。
- vehicle info YAMLにwheel base 1.087m、front overhang 0.467m、rear overhang 0.510m、wheel tread 1.12m、left/right overhang 0.09mがあることを確認した。
- rear-axle referenceを仮定した導出外形は全長2.064m、全幅1.30m、前方1.554m、後方0.510m、左右各0.650mである。
- config上のMPC `width=1.45` はmargin込み用途であり、vehicle footprintの正本として混用しない。
- 上記寸法はrepositoryの現行値であり、trajectory基準点とAWSIM colliderの対応確認前にclearance既定値として確定しない。
- この時点では実装、test、GUI、sim / gateは未実施だった。

### 2026-07-11 実装・自動検証

- `trajectory_clearance.py`へYAML、P2/P5 PGM、origin yaw、Y反転、unknown/map外、向き付き車体矩形、segment sweep、raw/conservative clearanceを実装した。
- final_ver3のbinary PGM、origin yaw=0、negate=0条件で現C++ map処理へ合わせ、画像内最大値での正規化と5 cell未満のoccupied component除去を実装した。一般map loader完全互換ではない。final_ver3 mapは751×759、occupied 320418 cell、free 249591 cell、unknown 0 cellだった。
- final_ver3の現行350 poseと暫定車体外形、margin 0m、sweep step 0.05mを検証し、point/sweep issue 0、raw minimum 0.437m、連続sweepを含むconservative minimum 0.252mだった。row-strip距離場とoccupied-row indexによる最適化後はhostで約4.8秒だった。これはAWSIM collider接触を保証しない。
- 四方向margin 0.5m、最大shift 0.5m、offset step 0.05mでは9件のmargin violationからsafe candidateを約41.0秒で生成した。探索はworkerで動き、progress dialogのCancelが協調停止を要求する。
- marginを四方向0.5 mとした確認では、point 4件・segment 5件の`CLEARANCE_MARGIN_VIOLATION`を検出した。
- `trajectory_clearance_dialog.py`へ暫定車体preset、manual extent/margin、unknown policy、sweep/offset設定、構造化reportとCenter Issueを実装した。
- Editorへ不変Original、Working/Candidate layer、水平・垂直scrollbar、Center Selection、clearance overlay、read-only Validate、detached Adjust、Apply guard、unsafe report時Save blockを統合した。
- safety review後、unknown warningによる後続wall見逃し、並進＋回転sweep、circular yaw seam、連続sweep距離下限、次善offset探索、曲率上限、失敗時SAFE無効化、Save前map再hash、hidden Working誤編集、危険cell boundsを修正した。
- 性能review後、occupied/unknown row index、距離場のconvex row-strip集計、transition broad phase、lazy beam、circular start beam、unsafe candidateのclearance計算省略、Adjustの協調Cancelを追加した。
- active CSV、occupancy YAML/PGM、vehicle YAML、configはGit差分なし。確認したblobは順に`c0f804e...`、`a18158d...`、`beba676...`、`01d1b9e...`、`d3b0241...`だった。
- host pure testsは181件成功した。
- `make autoware-build`は成功した。
- Docker内`colcon test --packages-select multi_purpose_mpc_ros`は226 tests、0 errors、0 failures、0 skippedだった。Editor出力をbuilt C++ validatorへ渡すcontract testも含む。
- Docker内で`trajectory_editor --help`、`pure_pursuit_trajectory_editor --help`、V2X Editor import smokeが成功した。
- `py_compile`、`package.xml` parse、`git diff --check`は成功した。
- `pre-commit`はホストにcommandがなく未実施。
- 実Tk GUI、`make dev`、最初のヘアピン、1周、gateは未実施。
