# Trajectory Editor Clearance / UX 改善 Requirements

作成日: 2026-07-11  
更新日: 2026-07-11  
状態: Core Implementation Complete / Deferred Requirements・GUI・Simulator Verification Pending

## 目的

既存のTrajectory Editorへ、次の3機能を安全に追加する。

1. 車体寸法と安全マージンを考慮した壁クリアランス検証とcandidate補正。
2. 読込時の原本、編集中データ、補正候補を常時比較できる表示。
3. 拡大後もコース全体へ移動できる水平・垂直スクロールバー。

本改善はoffline Editor内に閉じる。active trajectory、MPC runtime設定、ROS 2インターフェース、評価基盤を自動変更しない。

## 入力と関連文書

- 先行ステアリング: `../20260711-trajectory-editor-improvement/`
- Editor本体: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/tools/trajectory_editor.py`
- Preview: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/tools/trajectory_preview.py`
- MPC設定: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml`
- Occupancy grid: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/env/<course>/occupancy_grid_map.yaml` と `.pgm`
- 車両情報: `aichallenge/workspace/src/aichallenge_submit/racing_kart_description/config/vehicle_info.param.yaml`
- Lanelet2表示: `aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map/lanelet2_map.osm`
- 統合仕様: `docs/spec/mpc-integration.md`

## 現状

### 壁・車体

- EditorはLanelet2 OSMの左右railを線として表示するが、車体footprintや安全マージンを表示・検証しない。
- MPC runtimeはoccupancy grid mapを経路制約に利用するが、Editorは現在そのmapを読み込まない。
- `vehicle_info.param.yaml` にはwheel base、tread、前後左右overhangがあるが、Editorへ取り込まれていない。
- 現行trajectory座標が後輪軸、`base_link`、車体中心のどれを表すかをEditor側で確定していない。

### Original / Working / Candidate

- 内部に `original_trajectory` は存在するが、main canvasへ常時表示されない。
- candidate previewは「現在のworking」と「candidate」の比較であり、複数回Applyした後に読込時原本との差が分かりにくい。
- Save成功時に現在のoriginal snapshotが更新されるため、「セッション読込時の不変な原本」と「直近保存baseline」が分離されていない。

### 画面移動

- zoom、Fit、右・中ドラッグpanは存在する。
- pan操作は発見しにくく、タッチパッドやマウス設定によって操作しづらい。
- main canvasに水平・垂直スクロールバーがなく、拡大後のviewport位置とコース全体の関係を把握しづらい。

## 基本方針

1. `Validate Clearance` と `Adjust Clearance` を分離する。
2. clearance検証は常にread-onlyとし、working、Undo、revision、CSVを変更しない。
3. 自動補正はdetached candidate上だけで実行し、PreviewとApply確認を必須にする。
4. 壁の正本候補はoccupancy gridとし、Lanelet2 railを物理壁と断定しない。
5. 車体基準点、寸法、margin、map pathを画面へ明示し、暗黙値で安全を保証しない。
6. 読込時原本は次のtrajectoryをOpenするまで不変とする。
7. pan方式はユーザー決定どおり、水平・垂直スクロールバーを正規操作とする。既存drag panは互換操作として維持できる。

## 対象範囲

### 対象

- Occupancy grid YAML / PGMのoffline読込、座標変換、occupied/free/unknown分類。
- 車両寸法のYAML読込と手入力override。
- 車体footprint、margin envelope、区間sweepを用いたclearance検証。
- clearance不足箇所、最小距離、該当point/segmentの可視化。
- feasibleな範囲でのlateral candidate補正。
- Original / Working / Candidateの表示切替、凡例、差分統計。
- main canvasの水平・垂直スクロールバー。
- pure Python core、GUI adapter、回帰test、README /仕様更新。

### 対象外

- `config.yaml` のactive CSVやmap pathをEditorから自動更新すること。
- runtime MPCのconstraint、horizon、速度、操舵、OSQP変更。
- Lanelet2 map、occupancy grid、車両諸元ファイルそのものの自動修正。
- clearance補正だけでSafety Gate合格や実車非接触を保証すること。
- V2X車両、動的障害物、センサ誤差を含むruntime衝突回避。
- Pure PursuitへMPC専用occupancy-grid補正を暗黙適用すること。

### 今回の実装境界

ユーザーが求めたwall clearance、自動補正候補、原本比較、scrollbar navigationを
core範囲とする。左右別clearance、最長連続危険区間、補正対象`s_m`範囲、必要幅・
不足量は今回の安全な最小実装には含めず、後続要件として残す。したがって
`INFEASIBLE`は設定した有限samplingと探索上限内でcandidateを発見できなかったことを
意味し、連続空間に解が存在しない証明ではない。

## 機能要求

### R-CLR: Clearance検証

- `R-CLR-01`: `Validate Clearance` はworking dataと明示パラメータから新しいreportを返し、入力objectを変更しない。
- `R-CLR-02`: wall sourceとしてoccupancy grid YAML / PGMを明示選択し、resolution、origin、origin yaw、negate、occupied/free thresholdを解釈する。
- `R-CLR-03`: map座標とtrajectory座標の対応が確認できない場合、検証を成功扱いにしない。
- `R-CLR-04`: unknown cellの扱いをreportへ記録し、安全側既定ではoccupiedとして扱う。
- `R-CLR-05`: 車両寸法はwheel base、front/rear overhang、wheel tread、left/right overhangからfootprintを構成する。
- `R-CLR-06`: trajectory pointの車体基準位置を明示選択する。初期候補はrear-axle referenceとするが、repository実装との照合前に確定値と呼ばない。
- `R-CLR-07`: marginは前・後・左・右をメートル単位で入力でき、有限かつ0以上であることを検証する。
- `R-CLR-08`: 各poseの向き付き車体矩形だけでなく、連続pose間のswept footprintも検証する。
- `R-CLR-09`: collision、margin不足、unknown接触、map外を異なるissue codeで報告する。
- `R-CLR-10`: reportは最小clearance、衝突point/segment数、使用map・車体・margin条件を持つ。最長連続危険区間と左右別余裕は後続拡張とする。
- `R-CLR-11`: issue選択でmain canvasの該当位置へ移動し、車体footprintと危険cellを強調表示する。
- `R-CLR-12`: map resolutionより細かい精度を保証せず、測定分解能と数値許容差をreportへ表示する。
- `R-CLR-13`: occupied判定、画像Y反転、threshold、runtimeで有効な前処理をC++ MPCのoccupancy-grid処理と照合する。Clearance用途ではOSM lon/latをlocal XYとして使うfallbackを禁止する。

### R-ADJ: Clearance自動補正

- `R-ADJ-01`: `Adjust Clearance` は先に同じ条件でClearance検証を行い、既に安全な場合は不要なcandidateを作らない。
- `R-ADJ-02`: candidateはsource revisionと入力map・vehicle・margin条件へ結び、いずれかが変化した場合はstaleとしてApplyを拒否する。
- `R-ADJ-03`: 補正は原経路からの変位、offset変化、曲率変化を抑えながらfree corridor内の解を探索する。
- `R-ADJ-04`: 最大lateral shift、探索刻み、smoothness / curvature weightを明示設定できる。補正対象`s_m`範囲は後続拡張とする。
- `R-ADJ-05`: wallから離す単純な最近傍projectionだけで反対壁へ押し込まない。左右候補を同じclearance条件で評価する。
- `R-ADJ-06`: 補正後にpoint footprintとswept footprintを独立再検証し、違反が残るcandidateはApply不可とする。
- `R-ADJ-07`: feasible解がない区間は無理に補正せず、infeasible issueを報告する。必要幅・不足量の推定は後続拡張とする。
- `R-ADJ-08`: candidate内で`s_m/psi_rad/kappa_radpm`を再生成・検証する。Applyは1回のUndo snapshotとし、geometryをvalid、MPC speed metadataをstaleにする。
- `R-ADJ-09`: Apply後は`Recompute Speed`を既存フローで要求する。利用者がresolution等も変える場合だけNormalize Geometryを別操作で行い、保存時の暗黙再計算は行わない。
- `R-ADJ-10`: active CSV、map、runtime configを自動上書きしない。

### R-ORG: 原本比較

- `R-ORG-01`: `loaded_original` はOpen直後のdeep copyとし、Save / Save As / candidate Applyでは変更しない。
- `R-ORG-02`: `saved_baseline` が必要な場合はloaded originalと別stateにし、名称をUIで区別する。
- `R-ORG-03`: main canvasでOriginal、Working、Candidateを独立checkbuttonで表示・非表示にできる。
- `R-ORG-04`: 既定色はOriginal=gray、Working=blue、Candidate=orangeとし、線種と凡例でも識別可能にする。
- `R-ORG-05`: 点数が異なる場合はsame-s interpolationでOriginal / Workingを比較し、index一致だけに依存しない。
- `R-ORG-06`: 最大変位、path length差、変更区間、点数差を表示する。
- `R-ORG-07`: Original表示はread-onlyであり、選択・drag・insert・deleteの対象にしない。
- `R-ORG-08`: `Reset Working to Original` を提供する場合は確認とUndo snapshotを必須にする。

### R-SCR: スクロールバー

- `R-SCR-01`: main canvasへ水平・垂直 `ttk.Scrollbar` を追加する。
- `R-SCR-02`: scrollbar thumbは現在viewportとscrollregionの関係を反映し、zoom後も同期する。
- `R-SCR-03`: scrollregionは表示中のmap rail、Original、Working、Candidate、clearance overlayを包含し、適切なpaddingを持つ。
- `R-SCR-04`: `world_to_screen` / `screen_to_world` はCanvasのscroll offsetを考慮し、scroll後のdrag・insert・選択位置をずらさない。
- `R-SCR-05`: zoom時はcursor位置またはviewport中心のworld座標を維持し、scrollbarが先頭へ飛ばない。
- `R-SCR-06`: `Fit` はscaleと両scrollbarをコース全体表示へ戻す。
- `R-SCR-07`: issueまたはpoint選択から `Center Selection` を実行でき、選択位置がviewport中央へ入る。
- `R-SCR-08`: 既存の右・中drag panを維持する場合もscrollbar位置と常に同期させる。

### R-SAFE: State / Save安全性

- `R-SAFE-01`: Clearance Validate、Original表示切替、scroll操作はdirty、Undo、revisionを変更しない。
- `R-SAFE-02`: Adjust candidateのCancelでworking、Undo、revision、file hashを不変にする。
- `R-SAFE-03`: Apply直前にsource revision、map signature、vehicle parameters、candidate content、clearance reportを再確認する。
- `R-SAFE-04`: clearance stateは`not_run / running / safe / unsafe / failed / stale`を区別する。設定後のclearance error、map error、failed/running/stale、stale derived fieldがある状態ではstrict saveを許可せず、SAFE保存時もmap signatureを再確認する。
- `R-SAFE-05`: 原本比較用snapshotやclearance cacheをCSVへ追加列として保存しない。

### R-COMPAT: 互換性

- `R-COMPAT-01`: `trajectory_editor` / `pure_pursuit_trajectory_editor` の実行名とCLIを維持する。
- `R-COMPAT-02`: Pure PursuitではOriginal表示とscrollbarを利用可能とし、occupancy-grid自動補正は明示的にunsupportedまたは別policyとする。
- `R-COMPAT-03`: `v2x_position_editor.py` の既存import APIを壊さない。
- `R-COMPAT-04`: ROS topic/service/Domain、launch entry、result JSON、提出物契約を変更しない。

## 非機能要求

- map読込、footprint、sweep、candidate探索はTk / ROS 2から分離し、headless pytest可能にする。
- 同じmap、trajectory、車体、margin、探索設定から同じreportとcandidateを生成する。
- 大きなmapと0.25m trajectory処理はworkerで実行し、Tk main threadを長時間停止させない。
- mapは検証・Apply・SAFE保存時に再読込してcontent signatureへ結ぶ。将来cacheする場合も別mapの結果を再利用しない。
- 離散poseのraw clearanceと、point量子化およびsegment sweep距離場・回転膨張を含む保守的clearanceを区別する。
- 数値単位は画面とreportで明示し、車両値やmarginを2026公式値と呼ばない。
- source CSV、occupancy grid、vehicle info、runtime config、`output/`をtest中に書き換えない。

## 受け入れ条件

### 自動テスト

- YAML / PGM座標変換、Y軸反転、origin yaw、threshold、unknown分類のfixture testが通る。
- 既知矩形障害物に対する車体corner、edge、interior、swept collisionを検出する。
- margin境界の直前・一致・直後でdeterministicな判定になる。
- map外、unknown、invalid vehicle parameterを安全側で拒否する。
- 直線、円弧、ヘアピン相当のsynthetic corridorでsafe / collision / infeasibleを区別する。
- Adjust candidateが入力を変更せず、適用後も最大shiftとclearance条件を満たす。
- Original / Working / Candidateの点数が異なってもsame-s差分が正しい。
- Save後もloaded originalが変化せず、別trajectory Open時だけ置換される。
- scrollbar移動後のworld-screen往復、point selection、drag、insertが同じworld座標を指す。
- zoom anchor、Fit、Center Selection後のviewportとscrollbar位置が期待値に一致する。
- 既存trajectory関連test、Pure Pursuit、V2X import、C++ validator contract testが回帰しない。

### 手動確認

- Original / Working / Candidateと凡例が明確に識別できる。
- 拡大後に水平・垂直scrollbarだけでコース全域へ移動できる。
- clearance issueを選択すると該当車体・壁・margin不足が同時に見える。
- Adjust前後の最大変位、最小clearance、危険区間数が比較できる。
- candidate破棄、Undo、Save As、別file Openでstateが混線しない。

### 統合確認

- `make autoware-build` とpackage testが成功する。
- 別名candidateをC++ validatorへ通す。
- 選定candidateだけを明示的にruntime configへ指定し、最初のヘアピンと1周を確認する。
- lane keeping、MPC failure、NaN / Inf、wall contact、penaltyを既存baselineと比較する。

## 未確定事項

- physical wallの正本をoccupancy gridとする判断、およびLanelet2 railとの差分。
- trajectory poseの車体基準点と`vehicle_info.param.yaml`の座標系対応。
- unknown cellをoccupiedとする安全側既定が実コースmapで過剰制約にならないか。
- footprint sweepの補間間隔と、map resolution 0.1mに対する許容差。
- 自動補正の最大shift、探索刻み、smoothness、曲率、変位上限。
- margin既定値。公式値がないため利用者入力を正とするか、ローカル候補を設けるか。
- occupancy gridだけで実際の車体接触を十分表現できるか。
- clearance reportを画面だけにするかsidecarとして保存するか。

## Definition of Done

- 3機能が独立操作として実装され、read-only操作とcandidate変更操作が分離されている。
- occupancy grid、車両footprint、margin、sweepを含むclearance検証がpure test付きで成立する。
- Adjust candidateが安全条件を満たさない場合にApplyできない。
- loaded Originalが不変で、Working / Candidateとの差分を常時確認できる。
- 水平・垂直scrollbarでzoom後も全域へ移動・編集できる。
- active CSV、runtime config、ROS / 評価契約を変更していない。
- build、package test、C++ validator contractの結果と、未実施のGUI・シミュレータ回帰を`tasklist.md`へ記録する。
