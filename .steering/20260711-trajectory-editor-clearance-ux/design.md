# Trajectory Editor Clearance / UX 改善 Design

作成日: 2026-07-11  
更新日: 2026-07-11  
状態: Core Implementation Complete / Deferred Requirements・GUI・Simulator Verification Pending

## 設計方針

1. Clearanceは検証を先に実装し、自動補正を後段に置く。
2. 物理壁の候補にはoccupancy gridを使い、Lanelet2 railは表示・比較情報として分離する。
3. 車体を点や単純半径ではなく、基準点に対する向き付き矩形として扱う。
4. waypoint単体だけでなく、連続pose間のswept footprintを検証する。
5. 自動補正はdetached candidate transactionを再利用し、直接workingを書き換えない。
6. 読込時Originalを不変にし、Working / Candidateと別state・別描画にする。
7. main canvasをscrollable world canvasへ変更し、水平・垂直scrollbarを正規navigationにする。
8. ROS、runtime MPC、active CSV、map、vehicle infoはEditorから変更しない。

## 主要決定

| ID | 決定 |
|---|---|
| CUX-D01 | `Validate Clearance` と `Adjust Clearance` を分離する |
| CUX-D02 | 初期wall sourceはoccupancy gridを第一候補、Lanelet2 railを参考表示とする |
| CUX-D03 | vehicle YAML値は読込候補であり、基準点とmarginは利用者確認を必須にする |
| CUX-D04 | point footprintとsegment sweepの両方がsafeなcandidateだけApply可能とする |
| CUX-D05 | 自動補正は経路法線方向の有限候補探索をbaselineとし、無制約smoothingで壁から押し出さない |
| CUX-D06 | `loaded_original` は次のOpenまで不変とし、保存baselineとは分離する |
| CUX-D07 | navigationは水平・垂直scrollbarを採用し、既存drag panは補助操作とする |
| CUX-D08 | GUI描画とclearance数値処理を別moduleにする |

## Interface Compatibility Check

### Verdict

- `Compatible`: offline Editor、参加者package内のpure core、test、documentationに限定する場合。
- `Needs migration`: Editorからruntime config、map、vehicle info、ROS契約を更新する場合。

### 維持する契約

- `trajectory_editor` と `pure_pursuit_trajectory_editor` の実行名・既存引数。
- `Point`、`_default_paths`、`load_osm_rails`、`load_trajectory` のimport互換。
- MPC canonical 7列、Pure Pursuit 8列。
- `/control/command/control_cmd` 等のROS topic/service/Domain。
- active trajectoryとoccupancy gridのファイル内容。

## 変更後アーキテクチャ

```text
trajectory_editor.py
  ├─ document/view state
  │    ├─ loaded_original (immutable per Open)
  │    ├─ working
  │    ├─ candidate
  │    ├─ viewport / layer visibility
  │    └─ same-arc Original/Working difference and layer rendering
  ├─ trajectory_preview.py
  │    └─ before/after/report dialog
  ├─ trajectory_clearance.py       # new, GUI/ROS independent
  │    ├─ occupancy grid adapter
  │    ├─ vehicle footprint
  │    ├─ point/swept validation
  │    ├─ lateral feasible corridor
  │    └─ adjustment candidate/report
  ├─ trajectory_clearance_dialog.py # new, Tk settings/report adapter
  │    ├─ vehicle/map/margin settings
  │    └─ structured issue report / Center Issue
  ├─ trajectory_plot.py
  │    └─ existing candidate comparison model
  └─ trajectory_contract.py
       └─ existing CSV/state validation and atomic save
```

GUI専用描画helperを分ける必要が生じた場合は `trajectory_canvas.py` を追加する。clearance coreはTk widgetを受け取らない。

## Data Model

```python
@dataclass(frozen=True)
class OccupancyGridSpec:
    yaml_path: Path
    image_path: Path
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    origin_yaw_rad: float
    negate: bool
    occupied_thresh: float
    free_thresh: float
    unknown_is_occupied: bool
    signature: str

@dataclass(frozen=True)
class VehicleFootprintSpec:
    reference_point: str
    wheel_base_m: float
    front_overhang_m: float
    rear_overhang_m: float
    wheel_tread_m: float
    left_overhang_m: float
    right_overhang_m: float
    margin_front_m: float
    margin_rear_m: float
    margin_left_m: float
    margin_right_m: float

@dataclass(frozen=True)
class ClearanceIssue:
    code: str
    severity: Severity
    point_index: int | None
    segment_index: int | None
    s_m: float | None
    clearance_m: float | None
    required_margin_m: float
    grid_cell: tuple[int, int] | None
    message: str

@dataclass(frozen=True)
class ClearanceReport:
    map_signature: str
    source_revision: int
    vehicle: VehicleFootprintSpec
    minimum_clearance_m: float | None
    conservative_minimum_clearance_m: float | None
    measurement_resolution_m: float
    colliding_point_count: int
    colliding_segment_count: int
    unknown_contact_count: int
    outside_map_count: int
    issues: tuple[ClearanceIssue, ...]
    is_safe: bool

@dataclass(frozen=True)
class ClearanceCandidate:
    source_revision: int
    map_signature: str
    vehicle: VehicleFootprintSpec
    parameters: AdjustmentParameters
    poses: tuple[Pose2D, ...]
    offsets: tuple[float, ...]
    before_report: ClearanceReport
    after_report: ClearanceReport
    max_shift_m: float
```

`VehicleFootprintSpec` の例として現在のYAMLから算出できる外形は、長さ `wheel_base + front_overhang + rear_overhang`、幅 `wheel_tread + left_overhang + right_overhang` である。ただしtrajectory poseの基準点確認前に既定footprintとして確定しない。

## Occupancy Grid Adapter

### 読込

1. YAMLからimage相対path、resolution、origin、threshold、negateを取得する。
2. PGMのP2/P5、コメント、max valueを安全にparseする。
3. pixelからoccupancy probability相当を求め、occupied / free / unknownへ分類する。
4. final_ver3のbinary PGM、origin yaw=0、negate=0という現行条件では、画像最大値正規化、Y反転、pixel threshold、5 cell未満の小occupied component除去をC++ runtimeへ合わせる。一般map loaderの完全互換とはせず、origin yaw、unknown、negate/free threshold、map外処理の差をfixtureと文書で固定する。
5. YAMLとPGMのpath、内容またはmetadataからmap signatureを作る。
6. malformed、非有限値、不正threshold、image size不一致は構造化errorにする。

Lanelet2 railはOGMとの重ね合わせ確認に限定する。`local_x/local_y`がないnodeのlon/latをClearance用local XYへfallbackしない。

### 座標変換

ROS map conventionと画像row方向を明示する。

```text
world point
  -> subtract origin translation
  -> rotate by -origin_yaw
  -> divide by resolution
  -> map x/y cell
  -> invert image row for PGM access
```

次の往復をfixtureで固定する。

- world cell center -> image row/column -> world cell center。
- origin yaw 0と非0。
- image上下端、map外、negative coordinate。

## Vehicle Footprint

rear-axle referenceを採用する場合の局所座標候補は次のとおり。

```text
x_min = -rear_overhang - margin_rear
x_max = wheel_base + front_overhang + margin_front
y_min = -(wheel_tread / 2 + right_overhang + margin_right)
y_max =  (wheel_tread / 2 + left_overhang + margin_left)
```

各trajectoryの `psi_rad` でworldへ回転・平行移動する。別reference pointを選ぶ場合は明示adapterで局所originを変換する。

map resolutionが0.1mの場合でもcornerだけのcell確認では細い壁を見逃すため、rectangleのedgeとinteriorを含むrasterizationまたはpolygon-cell intersectionを使う。

## Clearance Validation

### Point footprint

- 各poseでmargin込みoriented rectangleを構築する。
- rectangleとoccupied / unknown / map外cellの交差を検出する。
- raw minimumは各離散poseの車体矩形とunsafe cell矩形の距離として計算し、map resolutionを併記する。
- conservative minimumはpoint側の `raw - resolution / sqrt(2)` と、segment sweep側の距離場下限・回転膨張を含む値の最小とする。0.1m gridの約0.071mはpoint側量子化項に限られ、report全体の固定差引量ではない。
- 左右別clearanceは後続拡張とする。

### Swept footprint

waypoint間を次の両条件を満たす刻みで分割する。

- 並進量がmap resolutionの一定割合以下。
- heading変化によるfootprint corner移動量が同じ上限以下。

各subintervalは両端footprintのconvex hullを作り、端点間の回転arcを包含する量で
保守的に膨張してoccupied / unknown / map外との交差を判定する。距離表示にはunsafe
cellへのChebyshev距離場を使った下限を集計するため、安全側のfalse positiveや実距離
より小さい表示を許容する。

headingはpi seamをwrapして補間する。circular trajectoryはclosing segmentも同じ処理を行う。

### Issue codes候補

| Code | 意味 |
|---|---|
| `FOOTPRINT_COLLISION` | point poseの車体がoccupied cellと交差 |
| `SWEPT_FOOTPRINT_COLLISION` | pose間の掃引領域がoccupied cellと交差 |
| `CLEARANCE_MARGIN_VIOLATION` | 接触はないが要求margin未満 |
| `UNKNOWN_CELL_CONTACT` | unknown cellと交差 |
| `FOOTPRINT_OUTSIDE_MAP` | 車体またはsweepがmap外 |
| `MAP_TRAJECTORY_FRAME_MISMATCH` | trajectoryがmap範囲と対応しない |
| `INVALID_VEHICLE_FOOTPRINT` | 車体寸法・reference設定が不正 |

## Clearance Adjustment

### Baseline algorithm

初期方式はpath法線方向の離散offset探索とする。

1. working geometryから各点の接線・法線を求める。
2. `[-max_shift, +max_shift]` を明示stepでsampleする。
3. 各point / segmentでfootprintが安全なoffset候補を列挙する。
4. dynamic programming等の決定的探索で次を最小化する。

```text
cost =
    displacement_weight * offset^2
  + smoothness_weight * (offset[i] - offset[i-1])^2
  + curvature_weight * offset_second_difference^2
```

5. 選択offsetからcandidate XYを作り、`s/psi/kappa`を再生成する。
6. point / swept clearanceを全区間で再検証する。
7. clearance、最大shift、曲率、path length、infeasible区間をPreviewへ渡す。

単一点を最近傍free cellへ押し出す方式は採用しない。局所的に左右どちらも不可能な場合はinfeasibleとする。

ローカル暫定既定はsweep step 0.05m、最大lateral shift 0.50m、offset step
0.05m、最大絶対曲率0.70rad/m、margin 0mである。2026公式値や車両限界ではない。
探索はboundedな候補列挙とbeamを使うため、`INFEASIBLE`は指定条件内で候補を発見
できなかったことを示し、連続空間での非存在証明ではない。

### Candidate transaction

```text
Working revision R
  -> snapshot + map/vehicle/options signature
  -> adjustment worker
  -> candidate geometry validation
  -> candidate clearance validation
  -> Original / Working / Candidate preview
  -> Apply?
       no: discard without state change
       yes: signature/revision/content recheck
            -> one Undo snapshot
            -> replace Working
            -> geometry_dirty=false (candidate内で再生成・検証済み)
            -> speed_dirty=true for MPC
```

Adjustが生成したcandidateはgeometry validationを通すが、shape変更前の速度metadataは安全条件を保持しない。既存Recompute Speedを完了してからstrict saveする。resolution変更が必要な場合はNormalizeを独立操作として実行する。

## Original / Working / Candidate View

### State

```text
loaded_original  # immutable until another Open
saved_baseline   # optional, updated after successful save
working          # editable
candidate        # detached, optional
```

現行 `original_trajectory` は役割を分割する。Save成功で `loaded_original` を上書きしない。

### Layers

| Layer | Default | Style | Editable |
|---|---:|---|---:|
| OSM / occupancy grid | ON | gray map | no |
| Original | ON | gray dashed | no |
| Working | ON | blue solid + points | yes |
| Candidate | candidate時ON | orange dashed | no |
| Vehicle footprint | selected/issue時ON | cyan / red | no |
| Clearance violation | report時ON | red / magenta | no |

表示checkbuttonはview stateだけを変え、document revisionやdirty flagを変更しない。

Original / Workingの点数が違う場合はexisting `trajectory_plot` のsame-s interpolationを再利用する。Originalの`s_m`がstaleな場合は、表示専用canonical arcをdetached copyで構築し、原本を変更しない。

## Scrollable Canvas

### Widget構成

```text
canvas_frame
  ├─ Canvas(row=0, column=0, sticky=nsew)
  ├─ Vertical ttk.Scrollbar(row=0, column=1, command=vertical controller)
  └─ Horizontal ttk.Scrollbar(row=1, column=0, command=horizontal controller)
```

Canvas widget固有のoffsetを正本にせず、既存の`center_x / center_y / scale`を
viewport stateとして維持する。scrollbar controllerは表示layerから算出したworld
domain上のfractionとcenter座標を相互変換し、thumbを`set(first, last)`で更新する。

### Virtual world transform

既存のviewport中心基準変換を維持する。

```text
world_to_screen:
  screen_x = canvas_width / 2 + (world_x - center_x) * scale
  screen_y = canvas_height / 2 - (world_y - center_y) * scale

event_to_world:
  inverse world_to_screen(event.x, event.y)
```

virtual world domainは表示中のrail、Original、Working、Candidate、危険cell、footprint
cornerから計算し、finite検証とpaddingを持つ。scrollbar、右・中drag pan、zoom、
Center Selectionはすべて同じcenter/scale stateを更新する。

### Zoom

1. zoom前にcursorまたはviewport centerのworld座標を保存。
2. scaleを更新してworld domainと全objectを再構築。
3. 保存したworld座標が同じscreen位置へ戻るようcenter座標を更新。
4. scrollbar thumbとselectionを更新。

### Fit / Center Selection

- Fitはworld boundsと現在widget sizeからscaleを計算し、その中央を表示する。
- Center Selectionは選択pointをviewport中心へ合わせ、端ではworld domain内へclampする。
- issue選択時はCenter Selectionを呼び出せるが、勝手なzoom変更はしない。

### Existing pan

右・中drag panもcenter座標を更新し、その直後にscrollbar thumbを同期する。scrollbarと
異なるpan stateは持たない。

## UI案

### Toolbar / View bar

```text
Open Traj | Open Map | Validate | Validate Clearance | Adjust Clearance
Normalize Geometry | Recompute Speed | Save As | Overwrite | Undo | Fit

[x] Original  [x] Working  [x] Candidate  [x] Vehicle / Margin
Center Selection | Vehicle / Margin Settings | Clearance Report
```

### Vehicle / Margin dialog

- vehicle YAML pathとReload。
- reference point。
- wheel base、tread、front/rear/left/right overhang。
- front/rear/left/right margin。
- occupancy grid YAML path。
- unknown policy。
- 各値のsource (`YAML` / `manual override`)。
- 外形length / widthの計算結果。

ApplyはEditor設定へ反映するだけでtrajectoryを変更しない。設定変更後はclearance report / candidateをstaleにする。

## Error Handling

- map / vehicle / frame errorではAdjustを開始しない。
- worker exceptionでworking、Undo、revisionを変更しない。
- Adjust workerはprogress dialogのCancel eventをfeasible列挙、transition探索、candidate再検証の境界で確認し、取消時もworking、Undo、revisionを変更しない。
- stale map signature、vehicle spec、source revision、candidate contentをApply直前に拒否する。
- infeasible candidateを「最善努力でsafe」と表示しない。
- unknown / map外を無視するモードを用意する場合はwarningと明示確認を必須にする。
- clearance stateは`not_run / running / safe / unsafe / failed / stale`を区別する。失敗した再検証では過去のSAFEを無効化し、設定後のfailed/running/staleは保存不可とする。SAFE保存時もYAML/PGMを再読込してmap signatureを確認する。

## Test Design

### Pure core

- P2/P5 PGM、YAML relative path、origin yaw、threshold、negate。
- world/grid/image座標の往復とY軸反転。
- vehicle parameter validationとfootprint corner。
- rectangle-cell intersectionと距離。
- straight、rotation、pi seam、circular closing sweep。
- unknown、map外、狭路、左右壁、袋小路。
- feasible corridor、左右選択、max shift、smoothness、infeasible。
- source non-mutation、determinism、map signature。

### Editor state

- Validate Clearanceのread-only性。
- setting変更によるreport/candidate stale化。
- candidate cancel/apply/stale/Undo。
- Save後もloaded original不変。
- layer toggleでrevision不変。
- Original / Working same-s差分。

### Canvas transform

- zoom / scroll後のworld-screen-world round trip。
- horizontal / vertical端とclamp。
- scroll後のnearest point、drag、insert。
- cursor anchor zoom、Fit、Center Selection。
- layer bounds変更後のscrollregion。

### Regression / Integration

- 既存trajectory testsとC++ validator。
- Pure Pursuit 8列とV2X import。
- active final_ver3はread-only fixtureとしてhash確認。
- 別名candidateでC++ validation。
- user-selected candidateだけでsim / gate回帰。

## Implementation Phases

### Phase 0: Contract and baseline

- map / vehicle / pose referenceを調査・確定。
- existing Original lifecycle、pan、zoom、candidate stateのtest seamを追加。
- synthetic occupancy fixtureを用意。

### Phase 1: Scrollbars

- scrollable coordinate transformと2本のscrollbar。
- Fit、zoom anchor、Center Selection、existing pan同期。
- 編集操作回帰。

### Phase 2: Original view

- loaded originalとsaved baseline分離。
- 3-layer表示、凡例、same-s差分。
- Save / Open / Reset / Undo state test。

### Phase 3: Clearance validation

- map / vehicle adapter。
- footprint / swept validationとreport。
- main canvas overlayとissue navigation。

### Phase 4: Clearance adjustment

- lateral feasible samplesとdeterministic optimizer。
- candidate preview、Apply guard、infeasible report。
- Normalize / Speed dirty連携。

### Phase 5: Documentation and runtime verification

- README / `docs/spec/mpc-integration.md` 更新。
- build、package tests、C++ validator。
- selected candidateのsim / gate比較。

## Rollback

- Clearance module / buttonsを無効化しても、existing Validate / Normalize / Speed / Saveは動作可能にする。
- Scrollbar移行はfeature単位で戻せるよう、world/canvas transformをhelperへ分離する。
- loaded original state追加はCSV schemaへ影響させない。
- 生成candidateはactive CSVへ自動反映しないため、破棄で完全にrollbackできる。

## 未確定事項

- occupancy gridとLanelet2 railのどちらが物理壁に近いかのコース別確認。
- rear-axle reference仮説とMPC/AWSIM pose定義の照合。
- clearance distance計算方式とmap resolution由来の誤差表現。
- unknown policyとmap boundary policy。
- lateral searchのstep、max shift、cost weight、曲率上限。
- pointwise rectangleとcontinuous sweepの合格許容差。
- marginのローカル候補値と実測根拠。
- report sidecar保存の要否。

## Decision Log

- 2026-07-11: 先行Trajectory Editor改善とは別のステアリングとして作成した。
- 2026-07-11: zoom後の移動方法は水平・垂直scrollbarを採用することをユーザー決定とした。
- 2026-07-11: Original / Working / Candidateをmain canvasで常時参照可能にする方針とした。
- 2026-07-11: clearanceはread-only validationを先行し、自動補正はcandidate transactionとして実装する方針とした。
