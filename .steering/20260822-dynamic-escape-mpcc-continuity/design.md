# Design

## 方針

### 1. MPCC activationをcore helperへ集約

`enabled`、`overtake_only`、OvertakeLine execution phase、Dynamic Escapeを入力し、
contouring-progressを要求するかと理由を一箇所で決める。

- 通常Cruise: legacy MPC（現行互換）
- ShiftOut / Pass / Return: progress MPCC（現行動作）
- Dynamic Escape: progress MPCC（今回追加）
- `overtake_only=false`: progress MPCC（現行動作）

Dynamic Escapeでextended 5-state solverが解けない場合は、既存どおり3-state
progress MPCCへ縮退する。progress preparation自体が失敗した場合はlegacy MPCへ
縮退し、そのformulationをtracking traceへ残す。

### 2. 横profileを前stageから接続

従来は全stageについて同じ実測`e_y`と横速度から到達区間を計算していた。
これでは各stage単独には到達可能でも、一つの制御列では順番に通れないprofileを
通してしまう。

新helperは一つのsegmentについて、

- 前segment終端の横位置・横速度
- segment距離と自車速度
- 横加速度上限とreserve
- collision corridor

から到達区間を求め、希望targetを区間内へclipする。選択targetに必要な定加速度から
次segmentの横速度を更新し、horizon全体をforward passする。

collision corridorのhard boundは変更しない。connected targetが作れない候補だけを
QP投入前にrejectする。

### 3. 診断

tracking traceへ次を追加する。

- `solver_formulation`: legacy / progress-3state / progress-extended
- `workspace_reset`: MPC/MPCC sparse solver mode resetの有無
- `connected_profile`: 使用有無
- `segment_shift`: 最大segment横移動
- `required_ay`: connected target生成時の最大必要横加速度

これにより、幾何preflight、profile接続、QP数値収束、formulation切替を区別する。

## 安全・互換性

- 壁、target footprint、EmergencyBrakeのhard guardは維持する。
- Q/R、OSQP max iteration、車体寸法、clearanceは変更しない。
- ROS I/O、launch、評価schemaに変更なし。
