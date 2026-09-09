# 縦入力モデルの比較と完遂計画への反映

2026-09-09。開始HEAD `ae2efe6a`、participantは`1c4f377e`から変更なし。
全体受入れは未完。一定biasを全horizonへ足す案は棄却し、接地・操舵・停止の
入力依存と、最新値選択の不確かさを扱うモデル比較を次の実装前条件にした。
[完遂計画](completion-plan.md)に対象コード、削除経路、合格条件を記載した。

## Nativeで棄却した仮説

`output/20260909-longitudinal-rest-model-r2`は現行nativeのtransitionと
`LongitudinalResponseObserver`をリンクした診断。package testの再実行ではない。
rolling-only比較はローカルCIL由来のR=0.37を分離したもので、完全plantではない。

| 初期状態・入力、0.2秒 | 現行wire=net | flat −0.37 | 動きに逆らうrolling component |
|---|---:|---:|---:|
| v=2、wire=+1 | 2.200 | 2.126 | 2.126 |
| v=2、wire=−1 | 1.800 | 1.726 | 1.726 |
| v=0、wire=0 | 0 | **−0.074** | 0 |
| v=0、wire=+0.37 | 0.074 | **0** | 0.0037 |

単位は終端速度m/s。flat補正は無入力静止で逆走を生成し、正入力を静止に置き換える。
さらに実observerへ静止速度0、公開制動−3を20ms刻みで2秒与えるとresidualは
`+2.9998417301328315m/s²`。これを未来の加速入力+1へ無条件に足すと
`3.9998417301328315m/s²`となる。停止時の入力飽和を含む残差を、入力に依存しない
物理外乱として全horizonへ引き継げない。既存prefix observerの全用途を誤りと
断定した結果ではない。r1/r2両方を保存し、修正済みtestのように扱わない。

## 実測状態を条件にした比較

診断単車`output/20260909-longitudinal-observed-single-r1`は6周、
253.053833秒、penalty0、monitor上のmoving overrideなし。
生成済み観測DLLを使っているため、通常性能や新モデルの受入れではない。
source ns、receipt、raw velocity/steering/IMU、Odometry、wire指令は
`output/20260909-longitudinal-observed-inputs-r1/d1.json`へ抽出した（97318行）。
受信11459、適用2630、単車wire一致10616。各適用の選択番号・source・float32入力を照合。

`output/20260909-longitudinal-response-comparison-r2/report.json`:
source15〜90秒の2143区間で推定し、90〜250秒の4571区間を保留評価。
別run dev2は6秒〜最初のD1Emergency9.834999780秒より前のみ、D1 38/D2 43区間。
別runへrefitせず、Emergency後の変更された応答を混ぜない。

| 速度増分MAE（m/s、通常35ms区間） | 単車保留 | dev2 D1 | dev2 D2 |
|---|---:|---:|---:|
| 現行wire=net | 0.046486 | 0.024123 | 0.018349 |
| rolling・dragと実測yaw/lateral項 | 0.023141 | 0.009799 | 0.003931 |
| 学習窓の一定offset | 0.012498 | 0.020616 | **0.026389** |
| 前輪の横力投影を含むfit | 0.010582 | 0.007167 | 0.001768 |

wheel fitのC1=3.347410/s、C2=2.070626m/s、flat=−1.278237m/s²は
同定用係数であり、production calibrationとして採用しない。
中点の**未来実測v、vy、yaw rate、steering**を使う比較なので、因果的なMPC予測
としての精度は未検証。現行七状態はvyとyaw rateを独立に伝播しない。
実適用記録も診断用で、productionから得られる入力ではない。
速度0.5m/s未満、source dt不正/50ms超、欠測などは除外理由をJSONへ残した。
低速・停止はこの表の合格範囲ではない。元のr1解析も上書きしない。

## raw yawの解析上の注意

ローカル`Vehicle::<FixedUpdate>g__ComputeVehicleState|70_0`は絶対Euler角を
直接差分し、角度の巻き戻りを処理していない。単車に5件の約1254rad/sがあり、
r1解析はこれを含んでいた。r2では各raw差分を、ローカルfloat32の5ms刻みから
得た1周分1256.6370849609375rad/sで主値へ戻してから中点を計算する。
観測の削除はしていない。Euler速度とbody angular velocityの違い、量子化は残る。
これは解析上の補正で、ROSメッセージやparticipantの修正ではない。
controllerはOdometryのyaw rateを使うため、このraw値を933の原因とは断定しない。

## 実行時の物理値

`output/20260909-longitudinal-physics-observed-dev2-r1`は、2受信器それぞれの
速度0.5m/s超でbody1件・wheel4件を取得して終了。6周の受入れrunではない。
主DLLは前sliceの4観測callと同じバイト列。変更したhelperはmain threadで
一度だけreflectionによる読み取りを加え、participant/Vehicleの値へ書き込まない。
missing memberは明示エラーにする。今回はprobe errorなし。新helperの実時間影響は
保証していない。receiver IDとROS Domainはこの短いreportでは対応付けていない。

`output/20260909-longitudinal-physics-analysis-r1/report.json`の実測:

- mass160kg、drag0.03、angularDrag1、skid0.236、rolling0.37、drive cap1.37、brake cap8。
- gripは**0.7**。Player.logに設定0.6を範囲[0.7,1]へclampした警告が2件ある。
  YAMLだけから0.6を実効値とした仮定を訂正する。現行wire操舵gain1.435を調整しない。
- steerDelay0.1秒、steerTimeConstant0.02秒、sleepVelocity0.02m/s、sleepTime0秒。
- COM local Unity xyzは約(−0.000005,0.169,−0.309008)m、inertiaは
  (64.16476,82.5931244,23.3309441)kg·m²。
- wheel sprungMass合計は約160kg、前輪合計は約25.881kg（16.176%）。
  両受信器とも片側前輪が非接地で、接地前輪は約14.375kg（8.985%）。
  sprungMassはWheelColliderの設定支持質量であり、瞬間の実測接地荷重ではない。
- その瞬間の前輪係数skid×接地sprungMass/(mass×5ms)は約4.241/s。
  全前輪接地を仮定すると約7.635/s。fitの3.347/sを物理定数とみなせない。

CILの`Wheel::UpdateWheelForce`は非接地なら横力・駆動力とも加えない。
全入力を4輪で等分し、横滑り打消し力を別途接地点へ加えるため、非接地輪は
入力から実加速度への変換にも影響する。2点の接地情報を全走行へ一般化しない。
このprobeはforceの各FixedUpdateを計測していない。
`source_ns`は選択指令の時刻で、物理状態の共通epochではない。
transformとworld COMの再構成差は最大約0.973mmで、補間されたtransform等の
時刻差を排除できない。観測geometryを厳密な同時刻proofへ転用しない。

## 検証・保全と残作業

実施した入口は`run_rest_comparison.py`（Compose内native）、
`run_single_observation.py single r1`、`extract_observation.py`（Compose内）、
`compare_measured_response.py`、`build_physics_probe.sh`（使い捨てCompose内）、
`run_physics_observation.py dev2 r1`、`analyze_physics.py`。
各command/build/output/hashは[evidence](evidence.json)とrun manifestへ封印する。
元DLLは変更なし、Docker生成の空probe mount placeholderのみ撤去、全container停止。
保護JSONのSHAは開始時のresult-summary `274ef27b…`、safety-gate `a297f288…`と一致。

26package build/2381testは1c4f377eの既存証拠をrunへ保存したもので、今回再実行
していない。本番変更がないため、観測・解析・文書の検証に絞る。
今回の結果は全体完遂でも新モデル採用でもない。次は[計画](completion-plan.md)の
M1で、実行時値と停止・接地・適用時刻を含む**因果的な**候補比較を閉じる。
