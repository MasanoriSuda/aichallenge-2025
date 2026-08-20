# Design

## 観測した欠陥

現行はactive overtakeの予測経路が壁契約を満たさない場合、`WallPathAdmissionGate`を有効にするが、Missionを失効させない。出力は直前操舵を保持し、毎周期`a_min`で減速する。このため再計画ではなく、同じ経路が安全になるのを待って停止する。

## 修正方針

### 1. 壁NGを経路棄却イベントへ昇格

現在footprintがclearで、予測経路のみが次の理由でNGになった初回に再計画を要求する。

- prediction invalid
- predicted contact
- predicted out-of-map
- insufficient clearance

MPC内部では該当Mission generationをinvalidateし、ShiftOut/PassからFollowPrepareのDynamic Mission Waitへ遷移する。古いlast-feasible cache、MPCC execution trajectory、Frenet DP prefixも破棄する。通常のsoft replanではDP prefixを維持するが、今回のprefix自体が物理壁NGの証拠なので再利用しない。

現在footprintが不正・接触中なら再計画へ突進せず、従来どおりstopを維持する。

### 2. wall admissionは短い安全なhandoffとして残す

Mission失効と同じ周期には新経路がまだないため、wall admissionの保持は残す。ただし次周期以降はinvalidated Missionではなく、再計画側の出力を評価する。安全な物理経路を連続2回確認した時だけ解除する。

### 3. planner/physical契約差を同じ決定ログへ記録

`WallHandoffAdmissionRequest`へ以下を追加する。

- Mission generation
- planner wall contractの有無
- planner minimum wall clearance

resolutionへ以下を追加する。

- replan required
- planner/physical contract mismatch

ログにはplanner値、physical値、差分、mismatch、replanを出す。

## 影響範囲

- `overtake_execution_orchestrator.*`: 純粋判定とログ形式
- `mpc_controller_cpp.cpp`: Mission失効・Dynamic Mission Waitへのhandoff
- orchestrator unit tests

ROS interfaceとconfig schemaは変更しない。
