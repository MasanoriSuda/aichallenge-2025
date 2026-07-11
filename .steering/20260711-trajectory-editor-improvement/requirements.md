# Trajectory Editor 改善 Requirements

作成日: 2026-07-11  
更新日: 2026-07-11  
状態: Phase 0〜6 Implemented / Runtime Simulator Verification Pending

## 目的

既存の Tkinter 製 Trajectory Editor に、MPC 用 trajectory を安全に検証・正規化・比較保存する機能を追加する。

単一の自動補正へ集約せず、次の3操作を独立させる。

1. `Validate Trajectory`: 入力や編集中データを変更しない検証。
2. `Normalize Geometry`: 候補コピー上での幾何形状正規化。
3. `Recompute Speed Profile`: 確定した形状に対する速度・加速度プロファイル生成。

目的は、補正内容と影響を利用者が確認してから別名保存できるようにし、C++ MPC が受け付けない CSV や、曲率・速度制約に異常がある CSV を誤って運用へ入れるリスクを減らすことである。

## 入力と正本

- 指摘原文: `chatgpt-pro-feedback.md`
- 関連ステアリング: `../20260710-chatgpt-pro-feedback/`
- Editor 本体: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/tools/trajectory_editor.py`
- MPC CSV 契約: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/src/path_core.cpp`
- Runtime validator: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/src/reference_path_validator.cpp`
- Editor の既存実行入口: `scripts/trajectory_editor`、`scripts/pure_pursuit_trajectory_editor`
- MPC 統合仕様: `docs/spec/mpc-integration.md`
- 参加者契約: `docs/interface/participant-interface.md`
- 評価契約: `docs/interface/evaluation-interface.md`

この文書の7列 CSV は現行リポジトリ内部の MPC 契約であり、Automotive AI Challenge 2026 公式の提出 CSV 仕様ではない。`resolution=0.25 m`、`a_max=1.0 m/s²`、`horizon_distance=16 m` も公式確定値ではなく、比較検証用のローカル候補値として扱う。

## 現状と課題

### 現行 Editor

- Python 3 + Tkinter の単一 GUI で、約905行の `trajectory_editor.py` に CSV 読込、幾何計算、編集、保存、UI が混在する。
- 同じ実装が、MPC の7列 CSV と Pure Pursuit の8列 CSV を扱う。
- CSV 判定は `x_m/y_m` または `x/y` の存在だけで、MPC の必須7列、列順、有限値、`s_m`、`psi/kappa/vx/ax` を検証しない。
- `float()` は NaN / Inf を受理し、途中の重複点・ゼロ長区間も読込時に拒否しない。
- 閉路は先頭・終端の位置差 `< 1e-5 m` だけで推定する。C++側の重複終端判定 `<= 1e-3 m` と一致しない。
- `Recompute geometry on save` が既定で有効であり、保存時に暗黙に `s/psi/kappa` を変更する。
- `Save` は確認、バックアップ、事前検証、原子的置換なしで元 CSV を直接上書きする。
- 手動編集、点挿入・削除、Laplacian 型 smoothing、Undo はあるが、検証一覧、等間隔再サンプリング、速度再計算、比較グラフ、補正レポート、専用テストはない。

### MPC Runtime との境界

- C++ strict loader は `s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2` の7列、有限値、厳密増加 `s_m`、非退化区間を検証する。
- 現在の C++ MPC は CSV の全列を検証する一方、ReferencePath 構築には主に `x_m/y_m` を渡して幾何量を再計算する。
- 現在の制御周期では基準速度が `effective_v_max` で上書きされるため、Editor が生成した `vx_mps/ax_mps2` だけでは走行時の速度プロファイルは変わらない。
- 0.25 m 周期 ReferencePath、距離ベース horizon、runtime speed profile は、既存ステアリングの Phase 3〜5 が所有する。Editor 側で同じ runtime 変更を重複実装しない。

### 互換性上の注意

- Pure Pursuit は `x,y,z,x_quat,y_quat,z_quat,w_quat,speed` の8列であり、全出力へ MPC の7列形式を強制すると既存機能を壊す。
- `v2x_position_editor.py` が `Point`、`_default_paths`、`load_osm_rails`、`load_trajectory` を Editor module から import している。
- `trajectory_editor` と `pure_pursuit_trajectory_editor` の実行名は既存利用手順である。

## 指摘の採否と優先度

| 項目 | 採否 | 優先度 | 方針 |
|---|---|---|---|
| 3操作の責務分離 | 採用 | P0 | Validateを最初に実装し、補正を暗黙実行しない |
| 非破壊 validation | 採用 | P0 | ファイルと編集中データを変更しない |
| MPC 7列 strict validation | 採用 | P0 | C++契約と同じ列・有限性・`s_m`・区間基準 |
| 問題一覧と該当点選択 | 採用 | P1 | 行、`s_m`、値、severity、グラフ位置を保持 |
| デフォルト別名保存 | 採用 | P0 | 上書きは明示確認し、保存前検証を行う |
| 重複・ゼロ長除去、`s_m` 再計算 | 採用 | P1 | レコード単位、候補コピー上で実行 |
| 0.25 m 等間隔再サンプリング | 条件付き採用 | P1 | 0.25 m は編集可能な候補値。周期線形を先行 |
| `psi/kappa` 再計算 | 採用 | P1 | `x/y` から周回対応で再計算 |
| 周期 spline・曲率 smoothing | 条件付き採用 | P3 | 周期線形との比較、最大変位、lane逸脱評価後 |
| 速度プロファイル生成 | 採用 | P2 | offline CSV生成。runtime反映は別ステアリング |
| Before / After 7グラフ | 採用 | P1 | XY、間隔、psi、kappa、速度、加速度、横加速度 |
| 補正レポート | 採用 | P1 | 入出力統計、処理条件、違反数を表示 |
| 2026プリセット | 条件付き採用 | P2 | `Candidate` と明記し、未確定値を編集可能にする |
| `horizon_distance` の変更 | 対象外 | - | MPC configの責務。Editorでは参考表示のみ |
| MPCゲイン・offset・OSQP変更 | 対象外 | - | 既存MPCステアリングの責務 |
| 公式Safety Gate合格保証 | 対象外 | - | Editor単体では保証しない |

## 対象範囲

### 対象

- MPC形式の trajectory 読込、検証、候補生成、比較、保存。
- Pure Pursuit形式の既存読込・手動編集・保存互換。
- GUIから分離した pure Python の model / validation / normalization / speed profile / report 処理。
- Editor 専用の GUI 非依存 pytest。
- C++ strict loader / validator と Python Editor の契約一致テスト。
- Editor の安全な保存フロー、Undo、未保存変更警告。
- package README と、実装完了時の `docs/spec/mpc-integration.md` 更新。

### 対象外

- ROS 2 topic、service、message 型、Domain、launch entry、result JSON、`output/latest/` の変更。
- `aichallenge_system/` の変更。
- MPC runtime の ReferencePath 再サンプリング、horizon、速度 overlay、OSQP、`wp_id_offset` の変更。
- Pure Pursuit CSV を MPC 7列へ変換すること。
- Lanelet2 中心線や左右境界そのものの自動編集。
- 実車向け速度・加速度既定値の確定。
- 周期 spline を、周期線形方式の比較証跡なしに初期実装の必須条件とすること。

## 機能要求

### R-OPS: 操作分離と状態

- `R-OPS-01`: `Validate Trajectory`、`Normalize Geometry`、`Recompute Speed Profile` を別操作として提供する。
- `R-OPS-02`: document は `original`、`working`、`candidate` を区別し、候補を承認するまで working data を変更しない。
- `R-OPS-03`: `circular` は重複終端の有無とは別の明示状態として保持し、GUI上で常時確認できるようにする。
- `R-OPS-04`: operation、入力パラメータ、入力 revision、出力統計を記録し、古い working data から生成した candidate を適用しない。

### R-VAL: Validate Trajectory

- `R-VAL-01`: validation はファイル、original、working、Undo履歴を変更しない。
- `R-VAL-02`: MPC形式では、入力順に依存せず必須7列 `s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2` が揃うこと、header重複・余分な列・列数不一致がないことを検証する。保存時だけcanonical順へ整列する。
- `R-VAL-03`: MPC全数値の完全変換、有限性、`s_m` の厳密増加、最低点数を検証する。
- `R-VAL-04`: 重複終端、経路途中の重複点、`<= 1e-6 m` の退化区間、最小・最大・平均 waypoint 間隔を区別して報告する。
- `R-VAL-05`: 角度差は `atan2(sin(delta), cos(delta))` と同等に正規化し、pi境界の人工的なジャンプを生成しない。
- `R-VAL-06`: 最大 `abs(kappa)`、最大 wrapped `abs(delta psi)`、最大 `abs(delta kappa)`、曲率 spike、速度・加速度範囲、最大 `v^2*abs(kappa)` を計測する。
- `R-VAL-07`: 設定された `v_max`、`a_max`、`a_min`、`ay_max` と数値許容差に対する違反を報告する。未設定の制約を暗黙の公式値で判定しない。
- `R-VAL-08`: circular path は closing edge を通常区間と同じ基準で検証する。重複終端入力の位置差と、正規化後の closing-edge spacing を別指標にする。
- `R-VAL-09`: issue は安定した code、severity、CSV行番号、point/segment index、`s_m`、列名、値、message を持つ。
- `R-VAL-10`: issueを選択するとXY表示と全グラフの該当点・区間を同期選択する。
- `R-VAL-11`: Pure Pursuit形式は既存8列を維持し、MPC strict 7列検証を適用しない。形式別 validator を明示する。

### R-NRM: Normalize Geometry

- `R-NRM-01`: 選択項目、値、予想変更点数を表示し、candidate copy 上だけで処理する。
- `R-NRM-02`: 重複終端判定は C++ と同じ `distance <= 1e-3 m` を既定とし、削除時は末尾レコード全体を1件削除する。
- `R-NRM-03`: 経路途中の退化区間削除は、保持点と削除点をレポートし、処理後に circular 3点以上、open 2点以上を保証する。
- `R-NRM-04`: `s_m` は座標から再計算し、circular では closing edge を含む canonical loop length を別に保持する。出力点は `[0,L)` とし、末尾重複を再追加しない。
- `R-NRM-05`: 等間隔再サンプリングは指定 resolution に対して seam を含む全区間を `0 < ds <= resolution * (1+tolerance)` とする。
- `R-NRM-06`: 初期方式は依存追加のない periodic/open linear interpolation とし、`psi_rad` と `kappa_radpm` は生成後の `x/y` から再計算する。
- `R-NRM-07`: topologyまたは点数が変わらない操作では `vx_mps/ax_mps2` を変更しないモードを提供する。点数が変わる場合は「周期補間」または「速度再計算」を明示選択させ、未選択のまま保存しない。
- `R-NRM-08`: smoothing は既定OFFとし、適用前後の最大位置変位、path長、曲率指標を表示する。
- `R-NRM-09`: periodic spline は任意モードとし、seam微分連続性、曲率spike、最大形状変位、wall/lane余裕、gate結果が周期線形より悪化しない場合だけ有効化する。
- `R-NRM-10`: candidate が validation error を持つ場合は working data への適用と保存を禁止する。

### R-SPD: Recompute Speed Profile

- `R-SPD-01`: `v_max > 0`、`a_max > 0`、`a_min < 0`、`ay_max > 0`、`minimum_speed >= 0`、`circular` を入力可能にし、全値の有限性と関係を事前検証する。
- `R-SPD-02`: 各点の上限を `min(v_max, sqrt(ay_max/max(abs(kappa), epsilon)))` で作る。
- `R-SPD-03`: `v[j]^2 <= v[i]^2 + 2*a_max*ds` と `v[i]^2 <= v[j]^2 + 2*abs(a_min)*ds` を適用する。
- `R-SPD-04`: circular path は closing edge を含め、許容差未満に収束するまで有限回反復する。最大反復到達時はcandidateを失敗扱いにする。
- `R-SPD-05`: `minimum_speed` が安全な速度上限を超える点がある場合、上限を破って下限を強制せず infeasible として報告する。
- `R-SPD-06`: `ax_mps2[i]` は点 `i` から次点への `(v_next^2-v_i^2)/(2*ds)` とし、circular はseamを含める。open path の最終点は `0.0` とする。
- `R-SPD-07`: open path の開始・終了速度境界は任意指定候補として扱い、仕様確定まで必須入力にしない。
- `R-SPD-08`: 全点で速度、縦加減速、横加速度を再検証し、違反が残るcandidateを保存しない。
- `R-SPD-09`: Editor生成速度はoffline CSV metadataであり、現状のC++ MPC走行速度へ直接反映されないことをUIとREADMEに表示する。

### R-UI: プレビュー、レポート、プリセット

- `R-UI-01`: before / candidate のXY経路、waypoint間隔、psi、kappa、velocity、acceleration、lateral accelerationを比較表示する。
- `R-UI-02`: グラフの横軸は原則 `s_m` とし、before/candidateを識別できる凡例と単位を表示する。
- `R-UI-03`: 補正レポートに元点数、補正後点数、総距離、削除点数、最小/最大/平均間隔、最大曲率、速度範囲、加速度範囲、最大横加速度、違反数、処理条件を表示する。
- `R-UI-04`: `AI Challenge 2026 Candidate - Safe` はローカル候補であることを画面に明記し、`resolution=0.25 m` と `a_max=1.0 m/s²` を編集可能にする。
- `R-UI-05`: `horizon_distance=16 m` はMPC側候補のread-only参考情報とし、Editorから `config.yaml` を変更しない。
- `R-UI-06`: 根拠のない Development / Race 値を新たな既定値として追加しない。

### R-SAVE: 保存安全性

- `R-SAVE-01`: 通常保存は元名へ `_normalized` または `_speed_profiled` を付けた `Save As` を既定とする。
- `R-SAVE-02`: 元ファイルまたは既存ファイルの上書きには、対象パスと変更概要を含む明示確認を要求する。
- `R-SAVE-03`: 保存前にcandidateを再検証し、MPC形式ではcanonical 7列順で出力する。
- `R-SAVE-04`: 保存は同一directoryのtemporary fileへ完全書込・flush後に原子的置換し、失敗時に既存ファイルを壊さない。
- `R-SAVE-05`: validation、normalize previewのcancel、overwrite confirmのcancelではファイル内容を変更しない。
- `R-SAVE-06`: 別trajectoryを開く、終了する、candidateを破棄する前に未保存変更を確認する。

### R-COMPAT: 互換性

- `R-COMPAT-01`: `trajectory_editor` と `pure_pursuit_trajectory_editor` の実行名と引数互換を維持する。
- `R-COMPAT-02`: `v2x_position_editor.py` が利用する既存import APIを維持するか、同一変更内で互換adapterを提供する。
- `R-COMPAT-03`: MPC保存物はC++ strict loaderと `reference_path_validator` の契約テストを通す。
- `R-COMPAT-04`: EditorはROS topic/service/Domain、評価成果物、MPC runtime configを変更しない。

## 非機能要求

- 数値処理は Tkinter / ROS 2 から分離し、GUIなしでpytest可能にする。
- 同じ入力と設定から同じ点列、統計、issue順序を生成する。
- validation error は例外文字列だけでなく構造化issueとしてUIへ渡す。
- 初期実装では新規数値ライブラリを必須にせず、既存実行入口のPython環境差を増やさない。
- 0.25 m候補で約350 mの周回を処理しても、UI操作を長時間ブロックしない。必要ならworkerと結果revision確認を導入する。
- source CSV、`output/`、rosbag、result JSONをテストfixtureとして書き換えない。

## 互換性・制約

- Interface verdict: offline Editor内に閉じる限り `Compatible`。
- runtime configやROSインターフェースをEditorから変更する場合は `Needs migration` とし、別要求・先行文書更新が必要。
- `docs/interface/*` の契約値変更は不要。実装後に package README と `docs/spec/mpc-integration.md` を更新する。
- C++とPythonでコードを直接共有できないため、canonical header、`1e-3 m` closure tolerance、`1e-6 m` minimum segment、angle wrapをgolden fixtureで照合する。
- 0.25 m出力を作成できても、runtime MPCを0.25 m / 16 m horizonへ移行済みとはみなさない。

## 未確定事項

- 曲率spikeの正式な判定式、warning/error閾値、近傍幅。
- seam `psi/kappa`、path length差、最大形状変位のwarning/error閾値。
- runtime採用時に、補間metadataを許容するかspeed再計算を必須にする最終運用条件。
- periodic splineの方式、依存ライブラリ、lane境界余裕の合格基準。
- circular speed profileの実走向け収束許容差と最大反復回数（Editorローカル既定は `1e-9` / `1000`、編集可能）。
- open pathの開始・終了速度、および最終 `ax_mps2` の運用要件。
- Tk Canvas previewの実window操作性確認（依存追加を避ける初期方式としては実装済み）。
- C++ validatorのJSON出力・問題位置出力を既存MPCステアリング側で先に拡張するか。
- Editorの候補レポートを画面表示だけにするか、sidecar JSONとして保存可能にするか。

## 受け入れ条件

### 自動テスト

- Validate前後で元CSVのSHA-256、original、working、Undo履歴が一致する。
- MPCの欠落・余分・重複header、短/長row、空値、部分数値、NaN / Inf、非単調 `s_m` を正しい行・列付きで検出する。
- `179 deg -> -179 deg` を約2度として扱い、約358度の差にしない。
- 重複終端、途中の重複、退化区間を別issueとして検出する。
- 重複終端をレコード単位で削除し、残存点数下限を守る。
- 円、直線、pi跨ぎ閉路を再サンプリングし、全区間とseamがresolution以下、`s_m`厳密増加、全値finiteになる。
- 直線は `kappa` が許容誤差内で0、既知半径円は `abs(kappa)` が `1/R` の許容誤差内になる。
- 速度profileが `v_max`、曲率速度上限、前進/後退制約、seam制約を満たす。
- 無効parameter、minimum speed不整合、非収束ではcandidateを適用・保存しない。
- cancelとoverwrite拒否で元ファイルがbyte単位で不変である。
- MPC出力headerがcanonical順で、C++ validatorが正常終了する。
- Pure Pursuit 8列CSVを開く、編集する、同じschemaで保存する回帰テストが通る。
- `v2x_position_editor` の既存importと起動が壊れない。

### 手動UI確認

- 3操作が分離され、Validateだけでは編集内容が変化しない。
- issue選択がXYと全グラフで同じ点・区間を示す。
- before/candidateの7グラフと補正レポートが一致する。
- 通常保存が別名を提示し、上書きに明示確認が出る。
- 未保存変更、candidate破棄、別ファイルopen、終了時の確認が機能する。

### 統合確認

- packageのpytest、C++ path_core test、validator contract testが成功する。
- 全MPC trajectory CSVをvalidatorへ通し、変更対象外CSVに意図しない差分がない。
- 生成CSVを指定した `config.yaml` で `make dev` または適切なgateを実行し、ReferencePath起動エラー、NaN / Inf、lane keeping退行がない。
- `/control/command/control_cmd`、`/localization/kinematic_state`、`/planning/scenario_planning/trajectory`、提出・評価契約を変更していない。

## Definition of Done

- 指摘内容が採用・条件付き・対象外へ分類されている。
- Validate、safe save、periodic linear normalization、offline speed profileがpure coreとテスト付きで実装されている。
- spline/smoothingは採用条件を満たして実装するか、根拠付きで保留されている。
- MPC 7列契約とPure Pursuit 8列互換の両方が維持されている。
- 0.25 m、1.0 m/s²、16 mを公式値として扱っていない。
- Editor出力速度とruntime MPC速度の責務境界がUI・README・仕様に明記されている。
- 自動テスト、C++ validator、シミュレータ回帰の実行結果が tasklist に記録されている。
