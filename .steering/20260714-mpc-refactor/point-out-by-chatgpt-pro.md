今のmpcコードは密結合かな？

はい、かなり密結合です。

特に mpc_controller_cpp.cpp に責務が集まりすぎています。MPCの数理コアだけでなく、以下が同じ実装単位に入り込んでいます。

経路生成・平滑化
地図境界計算
V2X追跡
Follow / Overtake / SafetyBrake の行動判断
追い越しラインFSM
OSQP問題生成・solve
加速度・操舵後処理
ROS subscriber / publisher
Boost制御
stuck recovery
フェイルセーフ
Domain別設定
ラップ状態管理

実際、MPC クラス自体が純粋な最適化器ではなく、V2XGapPlanner を直接保持し、内部で行動FSMを評価し、追い越し状態も保持しています。

さらにQP生成の init_problem() の中で、

前方車リスク判定
ギャップ探索
追い越し側選択
Follow時の横プリポジション
追い越しライン生成
速度制限適用
横目標生成

まで実行されています。つまり、戦略判断と最適化問題の構築が同じ関数フローに結合しています。

密結合による具体的な問題
1. MPC単体を検証しにくい

本来MPCコアは、

現在状態
参照軌道
速度参照
走行可能範囲
障害物制約

を受け取り、

制御入力
予測軌道
solver状態

だけを返すのが理想です。

しかし現在はMPC内部がV2X trackerの状態、追い越しFSMの状態、時刻、車両IDに依存しています。そのため、純粋な経路追従性能を試したくても、対戦制御の副作用が入ります。

2. Config変更の影響範囲が読みにくい

例えば v2x_overtake_* の設定変更が、

行動状態
→ gap planner
→ 横目標
→ MPC制約
→ solver feasibility
→ recovery

まで連鎖します。

一つのパラメータが判断層と制御層の両方へ効くため、「勝つための調整」と「solverを安定させる調整」を分離できません。

3. MPC失敗と戦術失敗を区別しにくい

QP infeasibleになった場合、

本当に物理的な走行可能解がない
ギャッププランナの制約が厳しすぎる
追い越し目標が急変した
経路境界が壊れた
数値スケーリングが悪い

のどれかを切り分けにくい構造です。

現在は追い越し中にsolver失敗が続くと、MPC内部の状態でRecoveryへ遷移します。
これは安全上は合理的ですが、最適化器が戦術FSMを直接変更しているため、層の境界としては密結合です。

4. trajectoryとMPCも密結合

ReferencePath が、

interpolation
smoothing
curvature
path width
speed profile
path constraints

を全て保持しています。

さらにMPCが ReferencePath の内部waypoint配列や境界配列へ直接アクセスしています。データ構造を変更するとMPCの大部分へ波及します。

5. ROS Nodeが制御器の内部状態を直接操作している

Node側から、

mpc_->update_current_speed(...)
mpc_->update_v_max(...)
reference_path_->set_v_ref(...)
mpc_->get_control(...)

という順番依存のAPIで制御しています。

この形式では、呼び出し順を間違えると状態が不整合になります。入力を一つの構造体にまとめて1周期ごとに渡す方が安全です。

どの程度か

感覚的には次の評価です。

部分	結合度
ROS I/Oと制御周期	高い
MPCとReferencePath	高い
MPCとV2X planner	非常に高い
MPCとOvertake FSM	非常に高い
MPCとRecovery	中～高
Solver wrapperとQP builder	中
pure coreテスト可能性	低い

ただし、完全に悪い設計というわけではありません。競技開発では短期間に機能を積むため、単一ノードへまとめるのは自然です。また、v2x_overtake_core や stuck_recovery_core をpure C++へ切り出そうとしている点は良い方向です。

推奨する分離構造
MpcControllerNode
  ROS I/Oだけ
        ↓
RaceBehaviorPlanner
  Follow / Overtake / Defense / SafetyBrake
        ↓
LocalTrajectoryPlanner
  d_ref, corridor, speed_limit
        ↓
MpcCore
  QP生成・solveのみ
        ↓
ActuatorAdapter
  steer gain, map, accel clamp
        ↓
SafetySupervisor
  stale odom, solver failure, emergency stop

MPCコアのインターフェースは例えば次です。

struct MpcInput
{
  VehicleState state;
  std::vector<ReferencePoint> reference;
  std::vector<double> lower_bound;
  std::vector<double> upper_bound;
  std::vector<double> target_speed;
  std::vector<double> target_lateral_offset;
  PreviousControl previous_control;
};

struct MpcOutput
{
  ControlCommand command;
  PredictedTrajectory prediction;
  SolverStatus status;
  double max_constraint_violation;
};

class MpcCore
{
public:
  MpcOutput solve(const MpcInput & input);
};

MPCは、

NPCか対戦車か
今が追い越し中か
最終ラップか
Domain番号はいくつか

を知らない構造にします。

それらは上位層が、

target_speed
target_lateral_offset
lower_bound
upper_bound

へ変換して渡します。

最初に切り離すべき箇所

全面リファクタリングより、次の順が安全です。

OSQP solver wrapperを独立
行列を受けて解くだけにする
MPC QP builderを独立
V2XやFSMを参照しない
Behavior結果を入力構造体化
target_velocity_limit
target_ey
corridor
emergency_stop
ReferencePathを読み取り専用interface化
内部vectorへ直接触らない
Nodeの順番依存APIを廃止
update_*() を連続で呼ぶ方式から、1周期入力へ変更

特に、evaluate_v2x_behavior() と update_overtake_line() を MPC クラス外へ出すだけでも、かなり見通しが良くなります。

結論として、現状は「MPCを中心に全部がつながっている」構造で、密結合です。
性能改善を続けるなら、まずMPCを賢くするより、MPCが受け取る問題を上位層で完成させ、コアを「制約付き追従最適化器」に戻すのが良いです。

まずは現行品の疎結合化が先？

はい、基本的には疎結合化が先です。
ただし全面リファクタリングではなく、性能改善を止めない最小分離が正解です。

理由はシンプルで、今の状態でMPCコアへ速度状態、soft constraint、RTI-SQPなどを足すと、V2X・追い越しFSM・経路処理・ROS I/Oとの依存まで一緒に複雑になります。すると「性能が上がったのか、戦略側の副作用なのか、数値不安定なのか」が分からなくなります。

おすすめの順序はこれです。

まず分離する範囲
MpcCore
QP生成
OSQP実行
予測軌道生成
solver status返却
BehaviorPlanner
Follow
Overtake
SafetyBrake
将来のDefense
LocalReferenceBuilder
target_ey
v_ref
lower_bound / upper_bound
ActuatorAdapter
steering gain
steer rate
accel clamp
command変換

MPCコアは最終的に、これだけ受け取ればよい形にします。

struct MpcInput
{
  VehicleState state;
  std::vector<ReferencePoint> reference;
  std::vector<double> target_speed;
  std::vector<double> target_lateral;
  std::vector<double> lower_bound;
  std::vector<double> upper_bound;
  PreviousControl previous_control;
};

struct MpcOutput
{
  ControlCommand command;
  PredictedTrajectory prediction;
  SolverStatus status;
};

MPC内部から以下を追い出します。

V2X vehicle ID
Overtake phase
ROS_DOMAIN_ID
lap count
NPC / rival distinction
Boost
Recovery
ROS time / publisher
ただし、先に直すべき安全バグは例外

疎結合化より先、または同時に直すべきものがあります。

a_min = -1.6 を競技上限内へ修正
steering gain適用後の舵角上限
全点40 km/h上書きの停止
空V2X IDが同一キーへ統合される問題

これらは設計改善を待たずに直すべきです。

実務的な進め方

大規模にファイルを分ける前に、現在のクラス内で境界を作るのが安全です。

Step 1:
evaluate_v2x_behavior() の結果を
BehaviorCommand構造体へまとめる

Step 2:
init_problem() からV2X/FSM処理を除去し、
MpcInputだけを見るようにする

Step 3:
OSQP wrapperを独立クラス化

Step 4:
ROS Nodeから update_*() の順番依存を除去し、
solve(input) の一発呼び出しへ変更

Step 5:
テストが通った後にファイル・パッケージを分割

最初からディレクトリ構成を大改造する必要はありません。依存方向を変えてから物理的に分割する方が事故が少ないです。

判断基準

次の状態になれば、MPCコアの性能改善へ進んで大丈夫です。

rosbagなしでMPC単体テストができる
同じ入力なら常に同じQPが作られる
V2X有無でMPCコード自体は変わらない
追い越しは target_ey / corridor / v_ref の入力差だけで表現できる
solver failureとbehavior failureを別ログで判定できる

結論としては、疎結合化を先に行うべきですが、1〜2週間級の全面刷新ではなく、MPCの入力境界を作る小さなリファクタリングを先にするのがよいです。その直後に縦加速度制約を入れる流れが最も安全です。