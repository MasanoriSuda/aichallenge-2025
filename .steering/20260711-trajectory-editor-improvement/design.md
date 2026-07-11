# Trajectory Editor 改善 Design

作成日: 2026-07-11  
更新日: 2026-07-11  
状態: Phase 0〜6 Implemented / Optional Spline and Runtime Verification Pending

## 設計方針

1. 検証を補正より先に実装し、Validateを完全に非破壊とする。
2. UI、CSV adapter、数値処理、保存処理を分離し、数値処理をGUIなしでテストする。
3. 元データを直接加工せず、`original -> working -> candidate -> apply -> save` の状態遷移を明示する。
4. MPCの7列形式とPure Pursuitの8列形式を別adapterとして扱う。
5. C++ `path_core` / validator をruntime側の受け入れ基準とし、Pythonに独自の別契約を作らない。
6. 初期正規化は周期線形補間とし、周期splineは比較結果に基づく後続機能とする。
7. Editorで作った速度はoffline metadataであり、現状のC++ MPC走行速度を変更しないことを明示する。
8. 保存時に暗黙の幾何再計算を行わず、derived fieldが古い場合は明示処理を要求する。

## 主要決定

| ID | 決定 |
|---|---|
| D-01 | 3操作を独立させ、Validateは常にread-onlyとする |
| D-02 | MPC高度機能はstrict 7列形式に限定し、Pure Pursuit既存編集を維持する |
| D-03 | `circular` はduplicate endpointから独立したdocument stateとする |
| D-04 | candidateをpreview・再validationしてからworkingへapplyする |
| D-05 | 通常保存はsuffix付きSave As、上書きは確認付きとする |
| D-06 | periodic linearを最初のcanonical resamplerとし、splineはP3へ送る |
| D-07 | 0.25 m、1.0 m/s²、16 mは`Candidate`表示し、公式値と呼ばない |
| D-08 | `horizon_distance`とruntime speed consumptionは既存MPCステアリングの責務とする |
| D-09 | 初期グラフはTk/ttk内で実装し、新しいruntime依存を増やさない |
| D-10 | C++/Pythonの契約一致は共通golden CSV fixtureで検証する |

## Interface Compatibility Check

### Verdict

- `Compatible`: 変更をoffline Editorと参加者package内のtest/documentへ限定する場合。
- `Needs migration`: EditorがROS契約、MPC runtime config、評価成果物を変更する場合。

### Blocking Issues

- 現時点で外部インターフェース上のblocking issueはない。
- ただしMPC 7列を全形式へ強制するとPure Pursuitを壊すため、format adapter分離は必須である。

### Migration Notes

- `trajectory_editor` と `pure_pursuit_trajectory_editor` の実行名を維持する。
- `v2x_position_editor.py` がimportする既存symbolを互換adapterで維持する。
- runtimeで0.25 m pathやCSV速度を使う変更は `../20260710-chatgpt-pro-feedback/` のPhase 3〜5へ接続する。

### Required Doc Updates

- 実装後: package `README.md` のEditor操作・保存・制限事項。
- 実装後: `docs/spec/mpc-integration.md` のoffline Editor / runtime境界。
- ROS・評価契約を変更しないため、現段階で `docs/interface/*` の値変更は不要。

## 現行構成

```text
scripts/trajectory_editor -----------------------+
                                                   |
scripts/pure_pursuit_trajectory_editor --preset ---+
                                                   v
                               trajectory_editor.py (約905行)
                               + CSV format検出
                               + x/yだけの読込
                               + OSM rail読込
                               + geometry再計算
                               + manual edit / smoothing / undo
                               + Tkinter UI
                               + 直接Save / Save As
```

現行構成の問題は、parse・数値計算・UI・ファイル更新が同じmoduleとmutable dataへ結合している点である。保存時の`recompute_on_save`により、利用者が明示していない派生値変更も発生する。

## 変更後の構成

```text
existing launcher scripts
          |
          v
trajectory_editor.py                    # Tk/ttk UI adapter・互換export
  |       |       |       |
  |       |       |       +--> trajectory_preview.py
  |       |       |             parameter / preview dialog・responsive worker
  |       |       |
  |       |       +----------> trajectory_plot.py
  |       |                     XY / scalar graph / synchronized selection
  |       |
  |       +------------------> trajectory_processing.py
  |       |                     normalize / geometry / report
  |       +------------------> trajectory_speed.py
  |                             offline speed profile / constraint check
  +--------------------------> trajectory_contract.py
                                data model / adapters / validation / atomic write

test/test_trajectory_contract.py
test/test_trajectory_processing.py
test/test_trajectory_speed.py
test/test_trajectory_plot.py
test/test_trajectory_editor_compat.py
test/test_trajectory_cpp_contract.py
```

実装時の最小分割名は変更してよいが、次の責務境界を保つ。

| Component | 責務 | 禁止事項 |
|---|---|---|
| UI adapter | command、dialog、selection、state表示 | 数値アルゴリズムを直接実装しない |
| Format adapter | parse、serialize、schema、row mapping | GUI objectへ依存しない |
| Validation core | metrics、issue生成、constraint判定 | 入力documentを変更しない |
| Processing core | candidate生成、normalize、speed、report | ファイルを直接書かない |
| Plot adapter | before/after表示、selection同期 | candidateを変更しない |
| Save adapter | UI側suffix/確認、contract側atomic write | 暗黙のnormalizeをしない |

## データフロー

```text
CSV bytes
  -> tolerant parse + format detection
  -> TrajectoryDocument(original == working, revision=0)
  -> Validate(original/working) ----------------------> ValidationReport
  -> Normalize Geometry settings
  -> deep-copy working
  -> candidate processing
  -> Validate(candidate)
  -> before/after preview + TransformationReport
  -> Apply? --no--> discard candidate
       |
      yes
       v
  working replacement + revision increment + Undo snapshot
  -> optional Recompute Speed candidate
  -> validation / preview / apply
  -> Save As(default) or confirmed overwrite
  -> serialize to temporary file
  -> final validation / atomic replace
```

### State model

```python
class TrajectoryFormat(Enum):
    MPC = "mpc"
    PURE_PURSUIT = "pure_pursuit"

@dataclass(frozen=True)
class ValidationIssue:
    code: str
    severity: Severity
    line_number: int | None
    point_index: int | None
    segment_index: int | None
    s_m: float | None
    column: str | None
    value: object
    message: str

@dataclass
class TrajectoryDocument:
    format: TrajectoryFormat
    source_path: Path
    circular: bool
    original: TrajectoryDataset
    working: TrajectoryDataset
    revision: int
    dirty_geometry: bool
    dirty_speed: bool

@dataclass(frozen=True)
class CandidateResult:
    source_revision: int
    operation: str
    parameters: Mapping[str, object]
    dataset: TrajectoryDataset
    validation: ValidationReport
    transformation: TransformationReport
```

`CandidateResult.source_revision != document.revision` の場合は、古い結果としてapplyを拒否する。

## Trajectory CSV 契約

### MPC format

保存時のcanonical headerは次の順で固定する。

```text
s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2
```

- headerは重複・欠落・余分な列を許さない。
- 入力列は名前で解決し、C++ loaderと同様に順序を固定しない。保存時はcanonical順へ整列する。
- 全fieldは完全にfloatへ変換でき、finiteであること。
- Editor入力は再現性のため10進数表記だけを受理する。C++ `std::stod` が受理し得るhex floatは、runtimeより安全側の非canonical入力として拒否する。
- `s_m` は厳密増加。
- 連続区間長は `1e-6 m` より大きいこと。
- circular unique representationは末尾に先頭重複を持たず、closing edgeをindex wrapで表す。
- legacy入力の重複終端は `distance <= 1e-3 m` で認識する。

### Pure Pursuit format

```text
x,y,z,x_quat,y_quat,z_quat,w_quat,speed
```

- 既存schemaと列順を維持する。
- MPCのstrict 7列出力へ変換しない。
- manual edit後のorientationがstaleの場合は明示的な再計算を要求し、保存時に黙って変更しない。

### Circular state

CSV schemaにcircular metadata列を追加しない。次の順でdocument stateを決め、UIへ常時表示する。

1. 明示CLI `--circular` / `--open`。
2. preset既定値。既知のMPC周回presetはcircular。
3. legacy duplicate endpointによるauto候補。
4. 利用者確認。

末尾重複を除去したCSVを再openしてもcircularと扱えるよう、MPC presetまたは明示引数を正とする。duplicate endpointとtopologyを同一視しない。

### Cross-language contract

- Python定数: canonical header、closure tolerance、minimum segment length。
- C++定数: `path_core` の同等値。
- 共通fixture: valid、missing/extra/duplicate header、NaN/Inf、非単調`s`、退化区間、重複終端、pi跨ぎ。
- Python testとC++ test/validatorの期待結果をfixture単位で照合する。

## 編集・補間・再計算方式

### Validate Trajectory

読込を2段階に分ける。

1. CSV syntax層はraw header/row/line numberを保持し、可能な限り複数issueを収集する。
2. typed dataset層はschemaと数値変換が成立した場合だけ生成する。

XYすら構築できないerrorではnormalize/speed/plotを無効にするが、schema issue一覧は表示する。validationは同一objectへ書き戻さず、metricsとissuesを新規objectで返す。

主なmetrics:

- raw / normalized point count、path length。
- min/max/mean spacing、closing-edge spacing。
- max `abs(kappa)`、max wrapped `abs(delta psi)`、max `abs(delta kappa)`。
- min/max velocity、min/max acceleration、max lateral acceleration。
- duplicate endpoint、degenerate segment、constraint violation件数。

curve spikeはデータモデルとissue codeを先に用意し、判定式・閾値確定まで設定なしではwarningを出さない。

### Normalize Geometry

処理順を固定する。

1. candidate copy作成。
2. 明示選択時だけclosure duplicateを末尾レコード単位で削除。
3. 明示選択時だけ途中の退化区間を削除。
4. unique pointsの全edgeからcanonical arc lengthを作成。
5. 選択時は指定resolutionへ再サンプリング。
6. `s_m` を再計算。
7. `x/y` から `psi/kappa` を再計算。
8. metadata modeに従い `vx/ax` を保持、補間、または再生成。
9. candidate validationとreport生成。

#### Uniform resampling

circular path:

```text
L  = sum(distance(p[i], p[(i+1) mod n]))
M  = ceil(L / requested_resolution)
ds = L / M
s[k] = k * ds, k = 0 ... M-1
```

open path:

```text
L  = sum(distance(p[i], p[i+1]))
segments = ceil(L / requested_resolution)
ds = L / segments
s[k] = k * ds, k = 0 ... segments
```

最初の方式はpolyline上の線形補間とする。periodic splineは別modeとして後から追加し、初期UIでは未実装またはdisabled理由を表示する。

#### Geometry derivatives

uniform circular pointsはwrapped central differenceを使う。

```text
x'  = (x[i+1] - x[i-1]) / (2 ds)
y'  = (y[i+1] - y[i-1]) / (2 ds)
x'' = (x[i+1] - 2 x[i] + x[i-1]) / ds^2
y'' = (y[i+1] - 2 y[i] + y[i-1]) / ds^2

psi   = atan2(y', x')
kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)
```

分母が閾値以下なら`kappa=0`へ黙って置換せず、candidate errorとする。open endpointは明示したone-sided差分を使い、testで固定する。

#### Velocity metadata during normalization

- point countとtopologyが不変: `vx/ax`をbyte相当の値として保持可能。
- point countが変化: 元canonical `s` 上で周期/開路線形補間するか、Speed Profile再計算を選ぶ。
- 選択なし: candidate作成を拒否する。
- 既存rowを単純copyして新点へ割り当てない。

### Recompute Speed Profile

parameter contract:

```text
v_max > 0
a_max > 0
a_min < 0
ay_max > 0
minimum_speed >= 0
epsilon > 0 (internal numeric policy)
```

初期上限:

```text
v_upper[i] = min(v_max, sqrt(ay_max / max(abs(kappa[i]), epsilon)))
```

`minimum_speed > v_upper[i]` が1点でもあればinfeasibleとし、安全上限よりminimumを優先しない。

forward/backward relaxation:

```text
v[j] = min(v[j], sqrt(v[i]^2 + 2*a_max*ds_ij))
v[i] = min(v[i], sqrt(v[j]^2 + 2*abs(a_min)*ds_ij))
```

circularはseamを含む全edgeを反復し、最大変化が設定tolerance未満で収束とする。最大iteration到達時は保存不可のerror。open pathの開始・終了境界速度は任意指定の後続項目とし、未指定時は上限制約から解く。

各rowの`ax_mps2`はoutgoing edge基準で計算する。

```text
ax[i] = (v[j]^2 - v[i]^2) / (2*ds_ij)
```

open path最終rowは`0.0`。計算後に全制約を独立に再検査する。

## 入力検証とエラー処理

### Severity

| Severity | 意味 | 動作 |
|---|---|---|
| Error | schema、非finite、退化、非収束、制約違反など安全に出力できない | apply/save禁止 |
| Warning | legacy重複終端、候補閾値超過、C++ validator未利用など確認が必要 | 明示確認後に進行可能 |
| Info | 統計、削除予定、候補値 | 表示のみ |

### Transaction rules

- 操作途中の例外はcandidateを破棄し、workingとUndoを不変に保つ。
- workerを使う場合、Tk objectへworker threadから触れない。
- 結果返却時にsource revisionを照合する。
- parse errorを単一messageboxだけで終わらせず、可能なissueをtableへ表示する。
- unexpected exceptionはtrace用情報と利用者向け要約を分ける。

### Dirty derived fields

manual point edit、insert、delete、smoothing後はgeometryをdirtyとする。geometryがdirtyなMPC datasetは、明示的なNormalize/Recompute Derived Fieldsを実行するまでstrict saveを許可しない。速度がshape変更に対して古い場合はspeedをdirtyとし、metadata補間またはspeed再計算を要求する。

## MPC・Validator との互換性

- Python validationはinteractive feedbackを担当し、C++ strict loader/validatorはruntime受け入れ基準を担当する。
- CIではEditor生成fixtureをC++ `reference_path_validator --circular --resolution <value>`へ渡す。
- C++ validatorが利用可能なinstalled環境では、MPC export dialogから追加確認を実行できるadapterを検討する。利用不可でもPython errorを無視して保存はできない。
- runtime C++は現状CSVの`psi/kappa/vx/ax`を走行へ直接反映しない。UI reportへ`Offline profile; runtime consumption pending`を表示する。
- 0.25 m CSVを生成しても、runtimeの`resolution: 0.6`、固定`N`、speed overlayを自動変更しない。
- runtime移行が必要になった場合、既存ステアリングのPhase 3〜5を先に完了し、同じfixture・gateでEditorとruntimeを比較する。

## UI設計

### Main window

```text
+------------------------------------------------------------------+
| Open | Validate | Normalize Geometry | Recompute Speed | Save As |
+---------------------------+--------------------------------------+
| XY map / before-after     | Issues (Treeview)                   |
| overlay / selected point  | code severity line s value message |
+---------------------------+--------------------------------------+
| Graph tabs: spacing psi kappa velocity acceleration lateral-acc  |
+------------------------------------------------------------------+
| format | circular | revision | dirty flags | source | candidate  |
+------------------------------------------------------------------+
```

- `Validate` はdialogなしでも現在workingを検証できる。
- Normalize/Speedはparameter dialogを開き、候補作成後にpreview windowを表示する。
- issue row、XY point、各graph cursorは同じselection modelを共有する。
- before/candidateの線色、単位、legendを固定する。
- candidate適用前に変更点数、最大変位、制約違反数を上部へ表示する。

### Candidate preset

`AI Challenge 2026 Candidate - Safe`:

```yaml
resolution_m: 0.25       # editable local candidate
a_max_mps2: 1.0          # editable local candidate
horizon_distance_m: 16.0 # read-only MPC integration hint
```

`v_max`、`a_min`、`ay_max`、`minimum_speed` は必須または利用者設定とし、根拠なしのRace値を作らない。

### Plot implementation

初期実装はTk Canvas/ttkを使用する。現行launcherはpackage venvをactivateせずsystem Pythonで動くため、matplotlibを採用する場合は依存・launcher・headless testを同じ変更で整える必要がある。依存整理なしにimportを追加しない。

## 保存設計

### Filename policy

| 最終操作 | 既定suffix |
|---|---|
| Geometry normalization | `_normalized.csv` |
| Speed profile | `_speed_profiled.csv` |
| Manual edit only | `_edited.csv` |

同名suffixが既にある場合は重複付与せず、file dialogで利用者に選ばせる。

### Save sequence

1. working/candidateのdirty flag確認。
2. format別final validation。
3. MPCはcanonical header順へserialize。
4. 上書きの場合はpath、変更概要、warningを表示して確認。
5. targetと同じdirectoryにtemporary fileを作成。
6. 全内容write、flush、必要に応じてfsync。
7. temporary fileを再parseしてfinal validation。
8. `os.replace()`相当でatomic replace。
9. 成功後だけdocument path、original snapshot、dirty stateを更新。

失敗・cancelではtargetとdocument stateを不変にする。

## 変更対象

| Path | 変更内容 |
|---|---|
| `multi_purpose_mpc_ros/tools/trajectory_editor.py` | UI再構成、互換export、state管理 |
| `multi_purpose_mpc_ros/tools/trajectory_contract.py` | model、format adapter、validation、atomic write |
| `multi_purpose_mpc_ros/tools/trajectory_processing.py` | normalize、geometry、report |
| `multi_purpose_mpc_ros/tools/trajectory_speed.py` | offline speed profile、constraint validation |
| `multi_purpose_mpc_ros/tools/trajectory_plot.py` | before/after graph modelとselection |
| `multi_purpose_mpc_ros/tools/trajectory_preview.py` | parameter dialog、7-view preview、worker |
| `multi_purpose_mpc_ros/test/test_trajectory_*.py` | pure core、契約、互換テスト |
| `multi_purpose_mpc_ros/CMakeLists.txt` | pytest登録、必要なfixture定義 |
| `multi_purpose_mpc_ros/README.md` | 操作、保存、安全制限、runtime境界 |
| `docs/spec/mpc-integration.md` | 実装完了後のoffline/runtime境界 |

原則変更しないもの:

- `docs/interface/*`、`aichallenge_system/*`、ROS topic/service、result schema。
- `config.yaml` のruntime resolution/horizon/speed設定。
- 既存trajectory CSV。実装・テスト時に原本を自動上書きしない。

## テスト設計

### Pure core unit tests

- schema、header、row、finite、`s_m`、点数、区間長。
- angle wrapとpi seam。
- duplicate endpointとinternal degenerate segmentの区別。
- circular/open arc lengthとuniform resampling。
- 直線、円、pi跨ぎ閉路の`psi/kappa`。
- metadata保持・補間・選択不足error。
- speed parameter、curve cap、forward/backward、seam収束、infeasible、非収束。
- reportの点数、距離、極値、違反数。
- 入力object非変更と結果determinism。

### Contract tests

- PythonとC++が同じgolden fixtureをaccept/rejectする。
- MPC export header順とC++ validator exit code。
- closure toleranceの境界 `0.999e-3`、`1.0e-3`、`1.001e-3`。
- minimum segmentの境界。

### UI adapter tests / manual checks

- Validateでworking/Undo/file hashが変わらない。
- candidate cancel/apply、stale revision、Undo。
- issue/XY/graph selection同期。
- default suffix、overwrite cancel、atomic save failure。
- dirty geometry/speedの保存block。
- 未保存変更のopen/exit確認。

Tk widgetの詳細より、commandがpure coreとstate transitionを正しく呼ぶことを優先する。

### Compatibility tests

- MPC `trajectory_editor` の引数とdefault path。
- Pure Pursuit 8列のopen/edit/save。
- `v2x_position_editor` のimport smoke。
- package install後の2つのlauncher起動。

### Integration tests

- 全MPC CSVをPython validatorとC++ validatorへ通す。
- 生成candidateを一時パスへ保存し、既存原本との差分を確認する。
- 選定candidateだけを手動でconfigへ指定し、`make dev` / gateでlane keeping、NaN/Inf、ログを確認する。
- runtime speed反映は既存MPC Phase 5完了後に別途確認する。

## 移行・ロールバック方針

### Phase 1: Validation and safe save

- pure data model、format adapter、validation、pytestを追加する。
- Validate UI、structured issues、別名保存、overwrite確認を追加する。
- 既存manual editingとPure Pursuitを維持する。

Rollback: 新command/panelを無効化し、既存editor entryを維持する。新coreはruntime ROS nodeから参照されない。

### Phase 2: Periodic linear normalization

- explicit circular state、candidate、duplicate/degenerate removal、arc length、linear resampling、geometry、previewを追加する。

Rollback: Normalizeを無効化し、Validation + manual editing + safe saveへ戻す。生成CSVを既存原本へ自動反映しない。

### Phase 3: Offline speed profile

- speed candidate、constraint validation、reportを追加する。
- runtime非反映を明記する。

Rollback: Speed buttonを無効化し、既存CSV速度を保持する。

### Phase 4: Optional spline/smoothing

- linear結果とvalidator/gateを比較し、採用条件を満たす場合だけ実装・既定OFFで公開する。

Rollback: periodic linearへ戻す。

## 依存関係

- Editorのperiodic normalizationは、既存MPCステアリングPhase 3と数式・fixtureを合わせる。
- 0.25 mをruntime既定化するには、既存Phase 4の距離ベースhorizonが必要。
- Editor速度をruntimeで使うには、既存Phase 5のbase profile / runtime overlay分離が必要。
- C++ validatorのJSON・issue位置出力は既存Phase 3の未完taskであり、Editor初期実装のblocking dependencyにはしない。

## 未確定事項

- curve spike、seam、path length差、最大形状変位の閾値。
- runtime採用時にmetadata補間を許容する最終条件（Editorではpreserve/interpolate/recomputeを明示選択済み）。
- periodic splineの方式とlane境界合格基準。
- 実走向けspeed収束tolerance / max iteration（Editorローカル既定は設定済み）。
- open pathのboundary speed運用。
- Tk Canvas graphの実window操作性確認（model・双方向selectionは実装済み）。
- sidecar report JSONの要否。
- installed C++ validatorをGUI保存時にも必須実行するか、CI契約testに限定するか。

## Decision Log

- 2026-07-11: 指摘を3操作へ分離し、Validateを最初の実装とした。
- 2026-07-11: 0.25 m、1.0 m/s²、16 mを2026公式値ではなくCandidateとして扱うことを決定した。
- 2026-07-11: 7列canonical出力はMPCだけへ適用し、Pure Pursuit 8列を維持することを決定した。
- 2026-07-11: `horizon_distance`とruntime速度反映は既存MPCステアリングへ残した。
- 2026-07-11: 周期splineは初期必須から外し、periodic linearとの比較後に判断することを決定した。
- 2026-07-11: 既存の暗黙geometry-on-saveを明示candidate/applyへ置き換える方針とした。
- 2026-07-11: `vx/ax` policyをpreserve、arc-length interpolate、speed再計算へdeferの3択とした。
- 2026-07-11: GUI依存を増やさずTk Canvas 7-viewとworkerを採用し、splineは比較証跡ができるまでdisabledとした。
