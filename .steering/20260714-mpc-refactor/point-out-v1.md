レビュー対象：e5e5171da0b611d19e1bebc45addf6cc678e0823

総評

方向性はかなり良いです。設計方針には賛成です。
特に次は適切です。

ROSノードをすぐ物理分割せず、同一process内でpure C++境界を作る
現行挙動をcharacterization testで固定してから移す
QpSolver → BasePath → Behavior → Reference → QP Builder → Cycle API → Arbiter と段階移行する
/control/command/control_cmd のpublisherを一つに保つ
性能改善と構造変更を同じ差分へ混ぜない

要求仕様でも、安全・外部契約・再現性を疎結合化より優先しており、競技コードのリファクタとして堅実です。
目標依存関係も概ね妥当で、現行の密結合ポイントを正しく捉えています。

ただし、現状の計画はPhase 0が重すぎることと、BehaviorとLocalReferenceの責務境界がまだ曖昧なことが主な懸念です。このまま実行すると、疎結合化へ着手する前に計測基盤づくりが長期化する可能性があります。

私の判定は、**「方針承認、計画は一部修正推奨」**です。

重要指摘
1. Phase 0の完了条件が広すぎます

現在のPhase 0には、

Docker image、compiler、OSQP、binary hash
Domain 0～4のROS graph、QoS、publisher owner
effective config exporter
MCAP recorder
recorder-ready barrier
QoS override
manifest/events/metrics/trace生成
path progressによる整列
toleranceの統計決定
dev、gate1～3、dev4、eval
trajectory topic mode
submit tarとeval image identity

まで含まれています。

品質保証としては立派ですが、Phase 1の単純なsolve_osqp()移動に必要な準備としては過剰です。

現在のPhase 0 DoDでは、B-01～B-09、H-01～H-08、D-01～D-11、全identityの再取得などが実質的なブロッカーになっています。

これでは、環境由来の問題やAWSIMの再現性問題が一つあるだけで、数百行のpure solver抽出にも進めません。

推奨修正

Phase 0を2段階に分けるべきです。

Phase 0-Minimum

Phase 1～2Aに進むために必要な最小baselineです。

canonical commit/config/resource hash
P/A/q/l/u
solver result
raw/final command
代表的なCruise 1ケース
feasible / infeasible / non-finite solver fixture
buildとpackage test
publisher ownerの確認
外部topic/typeの静的確認
Phase 0-Full

BehaviorやArbiterを移す前に必要な完全baselineです。

V2X状態列
Boost、Recovery、gear
stale odometry
dev4
gate群
eval
timing envelope
ROS graph/QoS
scenario recorder

つまり、

Phase 0-Minimum
  → Phase 1 QpSolver
  → Phase 2A BasePath
  → Phase 0-Full
  → Phase 2B以降

でも安全です。

QpSolverのmechanical extractionは、V2X、Boost、Recovery、eval imageを完全に凍結しなくても実施できます。

2. RaceBehaviorPlanner と LocalReferenceBuilder の責務がまだ重複しています

設計上、RaceBehaviorPlanner は、

pass side
target speed/offset
corridor narrowingの意図

を決めます。

一方、LocalReferenceBuilder は、

target velocity
lateral target
左右corridor
dynamic constraint

を作ります。

ここで問題になるのは、現在のギャッププランナが単なる戦術判断ではなく、

各horizon点への他車両投影
壁と車両からfree interval計算
左右ギャップ幅評価
到達可能性判定
target e_y
corridor narrowing

まで行っていることです。

この処理をどちらへ置くかが明確ではありません。

このまま起こり得ること
パターンA

RaceBehaviorPlanner がギャップ計算まで持つ。

するとBehavior層が、

horizon resolution
waypoint index
path bounds
prediction samples

へ依存し、実質的にLocalReferenceBuilderと再び密結合します。

パターンB

LocalReferenceBuilder がギャップ選択まで持つ。

するとLocalReferenceBuilderが、

Follow / Overtake状態
target vehicle
pass side判断
戦術的fallback

を知り始め、Behavior層との責務が重複します。

推奨する三分割
RaceBehaviorPlanner
    「追い越したいか」
    「どの車両が対象か」
    「左右の希望・禁止条件」
              ↓
LocalCorridorPlanner
    他車両予測
    free interval
    左右gap feasibility
    到達可能性
    corridor / target_ey候補
              ↓
LocalReferenceBuilder
    base path
    operational limit
    corridor candidate
    behavior request
    を最終ReferenceHorizonへ合成

値型も分けます。

struct BehaviorIntent
{
  BehaviorState state;
  VehicleTrackId target;
  PassSide preferred_side;
  double desired_speed_mps;
  bool allow_overtake;
  bool require_stop;
};

struct CorridorPlan
{
  bool feasible;
  PassSide selected_side;
  std::vector<double> lower_m;
  std::vector<double> upper_m;
  std::vector<double> target_lateral_m;
  double speed_limit_mps;
  CorridorRejectReason reject_reason;
};

BehaviorDecisionに完成済みcorridorまで入れると、Behaviorが幾何プランナ化してしまいます。

3. CycleInputへBasePathとConfigのsnapshotを毎周期入れる設計は注意が必要です

設計ではCycleInputに、

immutable base path/config snapshot
V2X snapshot
session state

などを持たせる方針です。

概念的には正しいですが、約400～500点の経路や各種vectorを40 Hzで値コピーすると、不要なallocationとcopyが増えます。

推奨

CycleInputには所有データではなく、不変スナップショットへの参照を持たせます。

struct CycleInput
{
  VehicleState vehicle;
  std::shared_ptr<const BasePath> base_path;
  std::shared_ptr<const ControllerConfig> config;
  V2xSnapshot v2x;
  SessionSnapshot session;
  TimestampSet timestamps;
};

またはライフタイムが明確なら、

const BasePathView & base_path;
const ControllerConfig & config;

です。

更新時だけ新しいsnapshotへatomic swapし、各周期は同じsnapshotを最後まで保持します。

auto path_snapshot = base_path_store.snapshot();
auto config_snapshot = config_store.snapshot();

CycleInput input{
    .base_path = path_snapshot,
    .config = config_snapshot,
};

これなら周期途中のparameter callbackやtrajectory callbackの影響も受けません。

設計文書には、

snapshot ownership
copyかviewか
lifetime
update atomicity
version ID

を追加した方がよいです。

struct SnapshotIdentity
{
  uint64_t path_version;
  uint64_t config_version;
  uint64_t v2x_version;
};

をCycleOutputにも残すと、どの入力から出力が作られたか追跡できます。

4. State所有者がまだ抽象的です

設計には、

previous CycleState + CycleInput
  -> decisions/results
  -> next CycleState + CycleOutput

とあります。これは良いです。

ただし、次のstateの所有先がまだ完全には決まっていません。

Behavior FSM state
overtake target lock
overtake phase
previous control sequence
previous steering
acceleration filter
solver failure counter
overtake solver failure counter
Boost phase
Recovery phase
gear request state
odometry/session latch

すべてを一つの巨大なCycleStateに入れると、今度は巨大State Objectによる新しい密結合が生まれます。

推奨

componentごとのstateに分けます。

struct BehaviorPlannerState;
struct MpcWarmStartState;
struct PostProcessorState;
struct BoostPolicyState;
struct RecoveryPolicyState;
struct SafetySupervisorState;

struct ControllerState
{
  BehaviorPlannerState behavior;
  MpcWarmStartState mpc;
  PostProcessorState postprocess;
  BoostPolicyState boost;
  RecoveryPolicyState recovery;
  SafetySupervisorState safety;
};

各componentは自身のstateだけを受け取り、自身の次stateだけを返します。

BehaviorStepResult RaceBehaviorPlanner::step(
    const BehaviorInput & input,
    const BehaviorPlannerState & previous);

MpcProblemBuilderにbehavior stateやfailure counterを渡さない方針は明記されていて良いです。
この原則を全componentへ拡張するとよいです。

5. Solver failureの同周期／次周期の意味をPhase 0まで未確定にしている点は正しいですが、API案が必要です

計画では、solver failureが追い越しRecoveryへ反映されるのが同周期か次周期かをPhase 0で確定するとしています。

これは非常に重要な確認です。

ただし、結果がどちらでも表現できるAPIにしておくべきです。

次周期反映の場合
Behavior(previous_solver_feedback)
  → Reference
  → MPC solve
  → solver feedbackを保存
  → 次周期Behaviorへ

これは計画中の、

MpcSolveResult
→ PreviousCycleFeedback
→ RaceBehaviorPlanner

で表現できます。

同周期反映の場合

同じ周期内で再度BehaviorまたはReferenceを作る必要があります。

Behavior nominal
→ MPC solve failure
→ fallback/recovery decision
→ possibly rebuild reference
→ second solve or direct safe output

この場合、単純な一方向pipelineでは再現できません。

推奨

Phase 0で次周期反映と確認できれば現行案のままで問題ありません。

同周期反映だった場合は、

solver failureはBehaviorへ戻さずCommandArbiterだけで処理する
または明示的な2-pass pipelineを定義する

のどちらかを設計書へ追加すべきです。

暗黙にcomponent間を呼び戻す構造は避けるべきです。

6. CommandArbiter / SafetySupervisor は分けた方がよいです

現在は一つのcomponent名としてまとめられています。

しかし、責務としては二つあります。

SafetySupervisor
stale odometry
non-finite
solver failure
collision/recovery latch
hard limit violation
SafeStop要求
CommandArbiter
NominalMpc
Fallback
Recovery
SafeStop

の候補から優先順位で一つを選ぶ

この二つを同じクラスにすると、検証と選択が混ざります。

推奨構造
Nominal candidate
Fallback candidate
Recovery candidate
Stop candidate
        ↓
SafetySupervisor
  各candidateのvalidity
  system-level inhibit
  mandatory stop
        ↓
CommandArbiter
  優先順位によりsource選択
        ↓
FinalCommandValidator
  finite
  angle
  slew
  accel
        ↓
RosOutputAdapter

ただし最初のmechanical extractionでは、ファイルは一緒でも構いません。型と関数境界だけ分けるのが安全です。

中程度の指摘
7. Baseline exact match対象を絞った方がよいです

現在はQP sparse structureをexact、numericをtolerance比較する方針です。これは妥当です。

ただし、疎行列のtriplet挿入順やEigenの内部格納順までexactにすると、数学的に同じQPでも失敗する可能性があります。

exactにすべきなのは、

dimensions
nonzero座標集合
upper/lower triangular policy
constraint row semantics
variable ordering

です。

比較時には、疎行列を、

(row, col, value)

でソート・正規化してから比較すべきです。

OSQP solutionも、最適解が一意でない問題では全要素一致を要求せず、

objective
constraint violation
first control
predicted trajectory
accepted status

を主比較にする方が堅牢です。

8. Phase 2AのBasePath read-only化で互換 façadeを長く残しすぎないこと

計画は「旧production pathをPhase gateまで残し、切替後は同Phase内で削除」としており良いです。

ただしBasePathについて、

public mutable vector
+ read-only API

を並存させる期間が長いと、新コードも旧vectorへアクセスし続けます。

Phase 2Aではコンパイル時に直接アクセスを禁止できるよう、

vectorをprivate化
friendを増やさない
view APIのみ公開
mutationはBuilder/Storeだけ

まで同Phaseで完了するべきです。

単なるgetter追加では疎結合化になりません。

9. Phase 2Bは大きすぎる可能性があります

現在のPhase 2Bには、

V2X adapter
tracker snapshot
behavior FSM
gap planner
overtake line
previous solver feedback
全状態sequence test
dev4

が入っています。

これはおそらく最も難しいPhaseです。

次のように分ける方がレビューしやすいです。

Phase 2B-1: V2xSnapshotBuilder
Phase 2B-2: FrontRisk / classification pure extraction
Phase 2B-3: Behavior FSM extraction
Phase 2B-4: OvertakeLine state extraction
Phase 2B-5: LocalCorridorPlanner extraction

各段階でlegacy結果と比較できます。

特にV2X trackerとBehavior FSMを一度に移すと、差分が出た際に、

tracker history
freshness
ID処理
projection
FSM transition

のどこが原因か切り分けにくくなります。

10. Config分離をPhase 5まで完全に延期しなくてもよいです

flat YAML compatibility loaderの本格整理をPhase 5へ置くのは正しいです。

一方で各pure componentの導入時には、最小限のtyped configが必要です。

例えば、

struct QpSolverConfig;
struct BehaviorConfig;
struct LocalReferenceConfig;
struct PostProcessorConfig;

は該当component抽出時に導入し、Phase 5では、

flat YAML
→ legacy resolver
→ complete ControllerConfig

のloader境界を整理する、という分け方がよいです。

巨大な現行MpcConfigを各componentが参照すると、ファイルを分けても設定依存は密結合のままです。

良い点
Baselineを「現行コードが正解」と扱っていない

既知の不整合をcandidate v0として記録し、安全修正をPhase 0bで別差分にしてからv1を固定する方針は非常に良いです。

これにより、

空ID統合
steering gain後の上限
速度プロファイル上書き
文書と有効機能の差

を、リファクタで無意識に保存することを避けられます。

実際にtasklistでもこれらを既知差分として列挙しています。

External contractと内部baselineを分離している

外部契約はexactに維持し、内部の数値挙動は測定したtoleranceで比較する構造は妥当です。

Productionにdual pathを残さない

test-only legacy oracleは許可するが、productionに長期の旧新切替を残さない方針も良いです。

競技コードでは、旧新両経路の存在がconfig事故や提出時の誤選択につながりやすいため、重要な原則です。

最終publisherを一つに固定している

MPC、Recovery、SafeStopが別publisherにならないことを要求・設計・tasklistの全てで一貫して固定しています。

推奨する修正版フェーズ構成
Phase 0A: Minimum characterization
  - external contract静的確認
  - QP fixture
  - solver fixture
  - command fixture
  - Cruise smoke
  - hard acceleration/steering safety確認

Phase 0B: Intentional safety fixes
  - 必要な場合のみ
  - baseline v1確定

Phase 1: QpSolver extraction

Phase 2A: BasePathStore immutable化

Phase 2B-1: V2xSnapshotBuilder extraction

Phase 2B-2: Front classification / risk extraction

Phase 2B-3: Behavior FSM extraction

Phase 2B-4: OvertakeLine extraction

Phase 2C-1: LocalCorridorPlanner extraction

Phase 2C-2: OperationalLimitResolver
           + LocalReferenceBuilder extraction

Phase 2D: MpcProblemBuilder extraction

Phase 3: One-cycle API

Phase 4A: ControlPostProcessor

Phase 4B: SafetySupervisor

Phase 4C: CommandArbiter
          + FinalCommandValidator

Phase 5: Config compatibility / tools / CMake整理

Final full characterization:
  dev / gate1-3 / dev4 / eval / submit

これなら1 Phaseあたりの変更量と原因切り分けが小さくなります。

計画承認前に追記したい項目

最低限、次の5点を文書へ入れることを推奨します。

Phase 0 Minimum GateとFull Gateを分ける
LocalCorridorPlannerをBehaviorとReferenceの間に追加
snapshotの所有権、version、copy/view方針を定義
component別state所有者を明記
Phase 2BをV2X adapter、FSM、OvertakeLine、Corridorへ分割

この5点を直せば、かなり実行可能性の高い計画になります。

現時点の計画は、思想としては良いのですが、少々「NASA品質の発射前チェックを済ませないと最初の関数を移せない」状態です。競技日程を考えると、安全性は落とさず、Phase 0の対象を変更リスクに比例させるのがポイントです。