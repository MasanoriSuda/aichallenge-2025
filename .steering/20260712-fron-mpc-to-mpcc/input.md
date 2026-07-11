以下ChatPROに相談したときのメモ

MPCをMPCCにするばあい
@[alexliniger/MPCC](https://github.com/alexliniger/MPCC)を使って移植する方針が良いのでしょうか？MPCで作っているものは
@https://github.com/MasanoriSuda/aichallenge-2025/tree/develop_julyにあります
10m 32s考えました
結論

alexliniger/MPCCをそのまま持ってきて現行MPCを置き換える方針は、あまりおすすめしません。

一番安全で効果が高いのは、

alexliniger/MPCCは数式・Spline・SQP・warm startの参照実装として利用し、実装本体は現在のmulti_purpose_mpc_rosに新しいMpccCoreとして段階的に追加する

という方針です。

言い換えると、**丸ごと移植ではなく「選択的移植」**です。

現行MPCは、すでにMPCCにかなり近い

現在のMPCは単純な時刻付き軌道追従ではなく、

x=[e
y
	​

, e
ψ
	​

, t]

という空間座標系の状態を使い、

u=[v, κ]

を最適化しています。各ステージはplanning_wp_id + nに固定され、経路横偏差、方位偏差、時間を評価しています。つまり、すでに「固定された空間区間を短時間で進む」というminimum-time MPCに近い構造です。

さらに現在の実装には、

occupancy gridから作った左右境界
V2Xによる動的なlb/ub
追い越し用の横目標
前走車による速度制限
solver失敗時の減速fallback
odometry timeoutやNaNチェック
Autoware/AWSIM向けROS 2インターフェース

がすでに統合されています。特にV2Xは、behavior FSMとgap plannerの結果をMPCの横制約・横目標・速度上限へ直接反映しています。

これらを捨てて、外部のstandalone MPCCへ載せ替えるのは、かなりもったいないです。

alexliniger/MPCCを丸ごと使いにくい理由
項目	現行実装	alexliniger/MPCC C++
実行環境	ROS 2 / Autoware / AWSIM	standalone simulation
モデル	空間座標系kinematic model	global座標のdynamic bicycle model
状態	3状態	10状態
入力	速度、曲率	駆動・操舵・進捗速度の変化率
solver	OSQP	HPIPM
障害物・他車	V2X corridorあり	C++版には障害物回避なし
経路境界	waypointごとの非対称・動的境界	基本的に固定左右幅
safety	timeout、fallback、command制限あり	simulation中心

Alex Liniger版は、

[X,Y,ϕ,v
x
	​

,v
y
	​

,r,s,D,δ,v
s
	​

]

の10状態と、

[
D
˙
,
δ
˙
,
v
˙
s
	​

]

の3入力を前提にしています。しかもNX=10、NU=3、N=60がコンパイル時定数です。

C++版README自身も、masterの車両モデルはfull-sized carのパラメータではうまく動かないこと、solverはHPIPMのみであること、C++版には障害物回避がないことを明記しています。

また、依存関係のinstall scriptはBLASFEOやHPIPMなどのリポジトリをバージョン固定せずcloneする構成で、現在もHPIPM API変更による問題がopen issueになっています。productionのAutoware環境へそのまま持ち込むには、再現性と保守性が弱いです。

おすすめする構成

ROSノードとMPCCを分離します。

MpcControllerNode
  ├─ ROS 2 subscriptions / publications
  ├─ AWSIM Boost
  ├─ V2X behavior FSM
  ├─ GapPlanner / OvertakeLine
  ├─ command filter / fail-safe
  │
  └─ ControllerCore interface
       ├─ LegacySpatialMpcCore   ← 現在のMPC
       └─ MpccCore               ← 新規

パラメータで切り替えます。

mpc:
  controller_variant: legacy_mpc  # legacy_mpc / mpcc / mpcc_shadow

mpcc_shadowでは、

車両へ出す指令は従来MPC
MPCCも裏で解く
solver時間、予測軌跡、最初の操作、制約違反だけ記録

という構成にします。

現在はmpc_controller_cppが単一のcontrollerソースへ多くの処理を集約しているので、まずcontroller coreの抽象化だけを行い、挙動を変えないPRを作るのが安全です。CMake上も現在の本番実行はOSQPをリンクしたmpc_controller_cppです。

例えば次のように分けます。

include/multi_purpose_mpc_ros/
  controller_core.hpp
  controller_problem.hpp
  mpcc/
    path_spline.hpp
    kinematic_model.hpp
    contouring_cost.hpp
    track_constraints.hpp
    mpcc_solver.hpp

src/
  legacy_spatial_mpc_core.cpp
  mpcc/
    path_spline.cpp
    kinematic_model.cpp
    contouring_cost.cpp
    track_constraints.cpp
    mpcc_solver_osqp.cpp

ROS topic、node name、Ackermann command、既存パラメータ、fallback経路は変えません。これは以前のPython→C++移植で採用した「外部契約を残し、比較しながら段階移行する」という方針とも一致します。

alexliniger/MPCCから持ってくるべき部分
持ってくる価値が高いもの
arc-length spline
X
ref
(θ)
Y
ref
(θ)
一階・二階微分
車両位置からSplineへのprojection
contouring error / lag error
誤差式
θを含むJacobian
一次近似によるQP化
progress state
θ
v
θ
	​

progress reward
前回解のshift
horizonを1段ずらす
周回境界でのunwrap
SQP
前回解周りで線形化
1〜数回のQP solve
solution mixing
θのtrust region

Alex版ではSpline上の位置からcontouring/lag errorとその微分を計算し、progress速度に負の一次コストを置いて前進を最大化しています。

また、前回解のshift、周回時の進捗unwrap、進捗状態のtrust region、複数SQP iterationという実装は、そのまま設計の参考になります。

最初は持ってこない方がよいもの
HPIPM interface
BLASFEOのvendor build
Magic Formulaタイヤモデル
10状態dynamic model
tire friction ellipse
hardcoded N=60
固定r_in/r_outによるtrack width
standalone plotting
simulation用main.cpp

特にdynamic tire modelを採用すると、少なくともv
y
	​

、yaw rate、タイヤ特性、質量、重心位置、慣性、駆動モデルの同定が必要になります。MPCC化と車両モデル同定を同時に行うと、問題が出た際に原因を切り分けにくくなります。

最初に実装するMPCCは「kinematic MPCC」がよい

最初の状態は、例えば次で十分です。

z=[X, Y, ψ, v, δ, θ]

入力は、

u=[a, 
δ
˙
, v
θ
	​

]

とします。

時間領域のkinematic bicycle modelとして、

X
˙
=vcosψ
Y
˙
=vsinψ
ψ
˙
	​

=
L
v
	​

tanδ
v
˙
=a,
δ
˙
=
δ
˙
,
θ
˙
=v
θ
	​


を使います。

コストは概ね、

J=
k
∑
	​

(q
c
	​

e
c,k
2
	​

+q
l
	​

e
l,k
2
	​

−q
p
	​

v
θ,k
	​

+q
v
	​

(v
k
	​

−v
ref
	​

(θ
k
	​

))
2
+r
a
	​

a
k
2
	​

+r
δ
˙
	​

δ
˙
k
2
	​

+r
Δu
	​

Δu
k
2
	​

+q
s
	​

ϵ
k
2
	​

)

です。

ここで、

e
c
	​

: contouring error
e
l
	​

: lag error
−q
p
	​

v
θ
	​

: progress最大化
ϵ: track制約用slack

です。

現在の空間モデルへ単純にθを追加するのは避けた方がよいです。
現在はすでに経路距離を独立変数として1ステージずつ進めています。その中でさらにθを進捗状態として最大化すると、「固定された空間進行」と「最適化される進捗」が二重になります。MPCC coreだけは時間領域モデルとして別実装にするのがきれいです。

また現在の空間モデルは1/v、1/v
2
を含み、低速時には線形化を止めるガードを入れています。時間領域kinematic modelなら、スタートや停止付近でもこの特異性を避けられます。

既存のV2X plannerはMPCCの外側に残す

最初から他車をMPCCの非線形制約として入れる必要はありません。

現在のplannerが生成している、

stageごとの
  lower lateral bound
  upper lateral bound
  preferred lateral target
  velocity upper limit

をReferenceCorridorとしてMPCCへ渡します。

MPCCの各SQP iterationでは、前回のθ
k
	​

に対してcorridorを評価し、

l(θ
k
	​

)≤e
c
	​

≤u(θ
k
	​

)

として固定または線形化します。

これなら、

Cruise / Follow / Overtake / SafetyBrake
pass side lock
curveでの追い越し禁止
前走車による速度制限
solver failure時のRecovery

をそのまま残せます。Alex版のC++には障害物回避がないため、ここは現在の実装の方が明らかに充実しています。

solverは最初からHPIPMへ変えない

最初はOSQPのままがよいです。

MPCC化だけでも、

状態
入力
コスト
経路表現
warm start
SQP
track制約

が変わります。ここでsolverまで同時に変更すると、切り分けが難しくなります。

ただし現在のsolve_osqp()は、solveのたびにosqp_setup()を呼び、workspaceを作り直しています。MPCCで1周期に複数回SQPを回すなら、solverをクラス化して、

疎行列パターンを固定
数値だけ更新
前周期の解を初期推定に利用
solve時間とiteration数を記録

する構造へ変えた方がよいです。

まずOSQPで動作を確立し、計測の結果として40 Hzの計算予算に入らない場合だけ、構造化QP solverへの変更を検討する順番が安全です。

段階的な移植順序
1. 現行MPCをcore化する

挙動を一切変えず、

class ControllerCore
{
public:
  virtual ControlPlan solve(
    const VehicleState & state,
    const ReferenceCorridor & corridor) = 0;
};

の後ろへ現行MPCを移します。

既存MPCのQP行列、最初の制御、予測軌跡をfixtureとして保存します。

2. continuous splineを追加する

現在の7列CSVにはすでに、

s_m
x_m
y_m
psi_rad
kappa_radpm
vx_mps
ax_mps2

があります。このs_mを使って周期Splineを構築します。

テスト対象は、

X(θ),Y(θ)
一階・二階微分
曲率
projection
周回seam
contour/lag Jacobianの有限差分一致

です。

3. static trackでMPCC shadow mode

最初は、

V2X無効
Boost無効
静的track boundsのみ
kinematic model
1 SQP iteration

で動かします。

従来MPCを車両へ出しつつ、MPCCについて、

solve時間
first control
progress
contour/lag error
track slack
infeasible回数

を記録します。

4. V2X corridorを接続する

現行gap plannerの出力を連続θへ対応付けます。

behavior FSMは変更せず、MPCCは「与えられたcorridorの中で最適化する」役割に限定します。

5. MPCCを実制御へ切り替える

static track、低速、スタート、周回seam、追い越しの順にgateを通します。

solver failure時に、毎周期legacy MPCとMPCCを行き来するのは避けます。最初は既存の安全減速fallbackへ倒し、legacy MPCはlaunchで明示的に選べる退避経路として残す方が挙動を予測しやすいです。

特に注意したい4点
1. N=30をそのまま流用しない

現在は、

N=30
path resolution 0.6 m

なので、物理的なhorizonは概ね18 mです。

時間領域MPCCでN=30、dt=1/40 sとすると0.75秒です。40 km/hでは約8.3 mしか先を見ません。

したがって、

mpcc:
  prediction_dt: ...
  horizon_steps: ...

をcontrol_rateから独立させ、現在の約18 m相当の先読みを維持するように決める必要があります。

2. 現在のQ/Rは流用できない

現在のQは[e_y,e_psi,t]、Rは[v,delta]用です。MPCCではcontouring、lag、progress、加速度、操舵速度へ意味が変わるので、数値をそのまま移してはいけません。

3. progressの「ズル」を防ぐ

lag costが弱いと、車両が進んでいないのにθだけ先へ進む解が出ます。

必要なのは、

十分なlag weight
v
θ
	​

の上下限
θのtrust region
前回解からのwarm start
projection結果との整合確認

です。

4. traj_mincurv.csvへ強く追従させると、ライン選択しない

min-curvature lineを参照Splineにしてcontouring weightを強くすると、MPCCにしてもほぼそのラインを追うだけです。

オンラインでライン選択させたい場合は、

trackの基準線と左右境界をhard geometryとして持つ
traj_mincurv.csvはpreferred lineとしてsoft costにする
Overtake時はpreferred-line weightを下げる

という二層構造が扱いやすいです。

ライセンス面

両リポジトリともApache License 2.0なので、選択的なコード移植は比較的行いやすいです。ただし、直接コピーするファイルでは、

元のcopyright headerを残す
Apache 2.0の通知を残す
変更したことを明記する
必要なattributionをREADMEやNOTICEへ追加する

必要があります。

私なら最初の実装単位はこうします
ControllerCoreを抽出し、現行挙動を固定する
PeriodicArcLengthSplineとprojectionを追加する
Alex版を参考にcontouring/lag costとJacobianを実装する
kinematic MPCCをOSQPでshadow実行する
現行V2X corridorをadapter経由で接続する

したがって方針名を付けるなら、

「alexliniger/MPCCの数式・Spline・SQP部分を選択的に移植し、現行ROS 2コントローラーの中へMPCC coreを追加する」

が最も良いです。Alex版のC++一式をproduction dependencyとして取り込むのではなく、現行実装を正本にするのが安全です。