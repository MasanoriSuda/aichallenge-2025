# End to End AI 部門ベース仕様

> Automotive AI Challenge 2026 End to End AI 部門向けのローカル整理。
> 正本は公式ドキュメント:
> <https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/ai-class.html>
>
> 確認日: 2026-09-01
>
> 公式ページは WIP。提出期限はチームへ Slack で共有された 2026-09-07 を
> 作業計画に使用するが、公開仕様として確定した値ではない。

## Runtime Input Contract

走行時に End to End AI が使用できる入力は次の範囲とする。

- Camera
- LiDAR
- Steer Angle
- Wheel Odometry
- Gear Status

GNSS、IMU、V2X、地図上の自己位置、計画 trajectory、MPC の出力は、E2E 推論の
入力に使用しない。評価ハンドシェイクを維持する目的で既存 node が起動していることと、
モデルがその出力を利用することは分けて監査する。

大会説明では入力から横方向制御まで ML を使用することが前提である。現ローカルの
最初の baseline は 2D LiDAR から steering を直接出力する TinyLidarNet とする。
横方向のauthorityはMLに限定し、longitudinalは固定加速度を基本としながら、同じLiDARの
前方clearanceが3.0 m以下なら加速を抑止し、1.5 m以下なら制動する。

## Safety Gates

End to End AI 部門でも次を評価できる構成を維持する。

- 障害物停止
- NPC 追い越し
- 車線維持

単車周回を成立させた後、停止車両、低速車両、他車両を含む順に評価範囲を広げる。

## Local Development Stages

1. 固定スタート・単車で 3 周連続走行
2. 複数 start / seed で単車再現性確認
3. 停止障害物と sensor stale の安全確認
4. 教師 bag の同期・分割・失敗区間を監査
5. recovery data を追加して再学習
6. NPC / 他車両を含む動的障害物対応
7. 4 台・6 周の決勝参考条件

MPC / MPCC は教師データ生成と比較評価に使えるが、E2E controller の推論入力には
しない。

## Teacher Dataset Contract

教師データはbag/runを最小のidentityおよびsplit単位とする。sample単位で同一runを
trainとvalidationへ分割してはならない。

- 入力: `/sensing/lidar/scan`
- 教師label: `/control/command/control_cmd`
- 既定同期上限: 50 ms
- 既定scan契約: 750点、最大30 m
- 既定教師出所: `mpc`、`mpcc`、`human`

抽出時に教師出所を必ず記録し、同期timestamp、同期差、元bag、topic型、scan shape、
採用/reject件数をsequence metadataへ保存する。E2E自身のcommandを記録した失敗bagは
`student`として観測・解析できるが、corrective labelなしに教師へ混ぜない。

trainerはmetadata欠損、shape/type不一致、同期上限超過、非finite値、教師出所不一致、
train/validationのsequence重複を学習開始前に拒否する。古いdatasetを暗黙補完せず、
契約対応extractorで再生成する。

MPC教師収集は`make e2e-teacher`を使う。これは`e2e-single`と同じ1台・NPCなし・固定
start・3周・LiDAR onの条件でcontrollerだけをMPCへ切り替え、localizationに必要なIMUを
追加する。IMU/GNSSは教師controllerとAWSIM infrastructureだけが利用し、student model
featureへは追加しない。抽出時は`--label-source mpc`を指定する。

同梱AWSIMのruntime NPCはV2X fanoutへ参加しないため、既存MPCをそのままNPC教師には
できない。3台固定配置のMPC/Tiny比較は`make e2e-peer-audit-mpc`と
`make e2e-peer-audit-student`で行えるが、2026-09-01時点のMPC試走では3 domainとも
wall/terminal証明切れ、Emergency StopまたはRecoveryへ波及した。このworldのMPC出力を
教師へ自動採用しない。TinyLidarNetはV2X/IMUを購読せずLiDARだけからsteeringを推論する。
将来教師候補として再利用する場合も、Finish、接触なし、長時間停止なし、Recoveryなしを
run単位で証明してから抽出する。

本番形状のruntime NPCに対する学生専用gateは`make e2e-npc-single`とする。runtime NPCの
MPC教師を捏造せず、peerで学習した回避がLiDAR外観差へ汎化するかを別Acceptanceとして
測る。

`make e2e-npc-single`による2026-09-01の初回baselineでは、約150秒後に実速度が
0.02 m/sまで低下した一方、TinyLidarNetは+0.6 m/s2を指令し、前方LiDARは約8 m
空いていた。これは意図的停止ではなく接触・壁拘束後も固定加速を続けた失敗として扱う。

回避教師候補は`make e2e-npc-gap-teacher`で収集する。単車でadmission済みの
TinyLidarNet steeringをbaseとし、180度LiDARの物理距離からFollow-the-Gap residualを
加える。横補正は教師データ生成専用の`gap_teacher` modeであり、最終studentへ残さない。
一方、教師と同じ縦停止判定は`fixed_lidar_brake`としてstudentにも残し、ML steeringを
変更せず固定加速度だけを制限する。教師自身がFinish/contact/stall gateを通ったrunだけを
`--label-source lidar_gap_teacher`として抽出する。

2026-09-01のcandidate3は、合格した教師rolloutを第2回DAgger train sequenceとして
追加し、単車、既知NPC seed、未見NPC seedの3条件をすべてpost-start stall 0秒で完走した。
production checkpointのSHA-256は
`de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`である。
単一の成功runでは昇格せず、通常走行と未見配置を含む閉ループgateを必須とする。

同checkpointの6周練習参考`output/20260901-084641`は2018.59 m、584.09秒を走り、
post-start stall gateを通過してAWSIM `Finish`へ到達した。一方、4台決勝参考
`output/20260901-085903`ではd1のみstallなしで、d2/d4は前方が7〜8 m開き正加速を
指令したまま約1.5 mの車体間隔で横接触固着した。これは縦停止閾値や起動契約ではなく、
対称な他車相互作用に対する横方策の未汎化として扱う。

診断専用`make e2e-final-contact-teacher`は同じ4台worldのd4だけを既存
`gap_teacher`へ置換する。`output/20260901-090729`ではteacherが実際に
`side-clearance` authorityを取得して横へ逃がしたが、d4には84.42秒の正加速固着が
残った。このrunは教師データへ採用しない。既存teacherは現在の極端側方sector距離へ
反応するだけで、相対運動や接触前の将来占有を表現しないため、side閾値の変更だけで
productionへ移植しない。次の横回避teacherは接触前のcoherentなLiDAR returnを
物理riskとして評価し、診断gateに合格してから抽出・再学習へ進む。

追加のbag replayにより、旧teacherは接触前のcoherentな側方returnをwhole-sector
10th percentileで背景へ埋没させ、risk検出後もML操舵との線形blendで障害物方向の
commandを残すことが分かった。後継診断`precontact_teacher`は距離閾値を変えず、3点目の
nearest returnで単発noiseと物体surfaceを分け、risk成立時は障害物方向の操舵を許さない
projectionを行う。

`make e2e-final-precontact-teacher`による`output/20260901-092811`では、d4は
606.70秒・1949.30 mを走り、post-start low-speedおよびpositive-acceleration stallが
ともに0秒だった。旧teacher d4の84.42秒固着は再発しなかった。ただしAWSIMの終端状態が
得られず、production d1には別の横接触固着が発生したため、このrunからlabelを抽出せず、
新teacherもproductionへ昇格しない。全teacher条件などでFinish・接触・stallをrun単位で
証明することを次のadmissionとする。

`make e2e-final-precontact-teacher-all`による終端run
`output/20260901-100204`では、4 domainすべてが同じ`precontact_teacher`を使い、AWSIM
`Finish`まで走行した。Finish時に確定した約550秒の各bagは1715.10--1770.28 mを走り、
post-start low-speedとpositive-acceleration stallはいずれも全台0秒だった。このrunの
teacherはoffline corrective label sourceとして採用可能とするが、production authorityと
checkpointは変更しない。再ラベル時は旧`lidar_gap_teacher_dagger`を流用せず、
`lidar_precontact_teacher_dagger`、`LidarPrecontactTeacher`、control modeおよび元run/domain
をmetadataへ明記する。practice harnessはresult JSONを出さないため、新studentの昇格前に
評価互換gateでcollision/penaltyを別途確認する。

このrunのactive label全5417件を再学習すると、旧teacherと同じgap追従まで重複して
独立validation MAEが28.2%悪化した。新旧teacherのsteering差が0.02 rad以上ある418件だけに
限定しても、全層fine-tuneは新補正MAEを53.1%改善する一方で通常MAEを21.6%悪化させた。
最終層だけの更新は通常RMSEを0.27%悪化に抑えたが、新補正MAEの改善が2.6%しかなく、
表現力が不足した。したがってteacher labelの大量追加やproduction checkpointの即時置換は
行わない。全層候補はproduction overrideなしの診断候補として単車Finishを確認した後、
4台worldでのみ追加判定し、失敗時はbase policyを固定したML residualなどへ分離する。

全層候補の4台診断`output/20260901-103656`では、d1/d3/d4にpost-start stallは
なかったが、d2が走行開始後280.70秒からbag終端まで123.51秒停止した。停止中は前方
LiDARが約1.45--1.66 m、右側が約1.00 mで、`fixed_lidar_brake`は正しく加速度を0へ
抑止していた。したがって縦安全層を緩和せず、候補checkpointもproductionへ昇格しない。
全層fine-tuneは通常走行を退行させながら接触前横回避を十分再現できず、最終層だけでは
補正表現力が不足したため、以後の局所解はproduction baseをfreezeし、新旧teacher差分を
zero-output正常anchorと共に学ぶML steering residualとして分離する。residualはoffline
評価と閉ループgateを通るまでruntime既定で無効とする。

### Frozen-base steering residual A/B

productionは引き続きSHA-256
`de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`の
TinyLidarNetと`fixed_lidar_brake`であり、residual pathの既定値は空とする。診断時だけ
`TINY_LIDAR_RESIDUAL_CKPT_PATH`を明示し、runtimeは
`base_steering + learned_residual`を実行する。checkpoint不在、shape不一致、非finite値は
起動時に拒否する。

residual datasetは同一scan上の`LidarPrecontactTeacher`出力からfrozen production base
出力を引いた値をtargetとする。旧`LidarGapTeacher`との差は診断provenanceであり、runtime
targetへ使わない。失敗bagは接触・固着suffixを教師化せず、接触cutoffまたは明示した
`--max-duration-sec`の早い方までを因果prefixとして保存する。短い失敗prefixが長い成功runへ
埋没しないよう、trainerは既定で各sequenceへ等しいsampling massを与える。train/valは
引き続きrun/domain単位で分離する。

2026-09-01の診断候補は単車`output/20260901-120424`で1015.22 m、平均3.40 m/s、
NPC `output/20260901-121209`で1020.53 m、平均3.20 m/sを走り、双方ともpost-start low-speed
およびpositive-acceleration stallが0秒だった。一方、4台`output/20260901-121938`では
d1/d2がsim時刻約118秒から211--219秒固着した。両者は正加速+0.6 m/s2を出し続け、停止位置の
間隔は約1.9--2.2 mだったため、縦停止ではなく横方策の失敗である。d3/d4は同runで
1049.37 m / 1102.46 mを走りstall 0秒だった。

失敗直前10秒の教師補正はd1が平均正方向、d2が平均負方向で、最も近いLiDAR状態間の教師符号
一致率は0%だった。現単フレームresidual CNNはこの境界で出力をほぼ0へ平均化し、強い再学習は
過去の正常anchorへ漏れた。したがってresidual runtime基盤はA/B用に保持するが、どの候補も
productionへ昇格しない。次のモデル変更は同じSliceへの閾値追加ではなく、時系列LiDAR、
wheel odometry等の許可入力、または状態付きarchitectureを別Sliceで評価する。

bag単位の固着監査は次で行う。起動待ちは除外し、一度1.0 m/s以上で走行した後の
0.15 m/s以下の連続時間と、そのうち正加速指令中の連続時間を別々に判定する。縦安全層が
正しく加速を抑止しても、その場で停止し続けるcandidateを成功扱いしない。GUIの見た目
ではなく、このJSONとAWSIM Finish/接触結果を合わせてcandidate admissionを判断する。

```bash
docker compose run --rm --no-deps autoware-command \
  python3 /aichallenge/ml_workspace/tiny_lidar_net/analyze_e2e_run.py \
  /output/<run>/d1/rosbag2_autoware --fail-on-stall
```

### Competition run acceptance

`analyze_e2e_run.py`の合格は「bag上で長時間固着していない」ことだけを表し、完走、周回数、
接触・壁ペナルティを証明しない。production候補の最終判定には、同一runに属するAWSIM結果、
motion analysis、起動時provenanceを`analyze_e2e_competition.py`で統合する。

AWSIMは結果JSONをprocess cwdへ書くため、`run_simulator.bash`はAWSIMのcwdを必ず
`/output/<run_id>/`へ変更する。リポジトリ直下の`aichallenge/result-summary.json`等は過去runに
上書きされた可能性があり、competition acceptanceの根拠に使用しない。古いrunにAWSIM結果が
残っていない場合はmotion gateがpassでも`incomplete`とする。

```bash
docker compose run --rm --no-deps autoware-command \
  python3 /aichallenge/ml_workspace/tiny_lidar_net/analyze_e2e_competition.py \
  /output/<run> \
  --expected-control-mode fixed_lidar_brake \
  --expected-checkpoint-path \
    /aichallenge/workspace/install/tiny_lidar_net_controller/share/tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy \
  --checkpoint-file \
    /aichallenge/ml_workspace/tiny_lidar_net/checkpoints/20260901_055824/candidate.npy \
  --expected-checkpoint-sha256 \
    de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa \
  --output /output/<run>/e2e-competition-analysis.json \
  --fail-on-rejection
```

既定はpenalty 0件を要求する。診断目的で許容数を変える場合は
`--max-penalty-count`を明示し、production昇格の結果と混同しない。

2026-09-01のfrozen production candidateによるcompetition matrixでは、単車
`output/20260901-151131`だけが3/3周、penalty 0、stall 0で合格した。runtime NPC
`output/20260901-152109`は2/3周後に右側へ埋まり、正加速中のstallが117.05秒、wall
penaltyが1件発生した。4台`output/20260901-153143`は全台未完走で、d1は0/6周かつ
wall penalty 361.79秒、d3にもwall penalty 77.02秒が発生した。d2/d4はstall・penalty
なしだったが、clean lapが約90秒で4/6周のままtimeoutした。

これらの失敗中もcheckpoint、`fixed_lidar_brake`、推論出力は維持され、d1では前方8 m
以上が開いた状態で正加速しながら右壁へ拘束されていた。したがって次のmodel Sliceは
threshold変更や既存candidateの即時再学習ではなく、失敗直前状態が学習分布外なのか、同じ
単一scanに逆向きactionが必要なobservation aliasingなのかをtrain/validationおよび
seed-disjoint teacher runと比較して判定する。判定が終わるまでproduction checkpointを固定する。

`analyze_e2e_state_coverage.py`による上記3失敗の監査では、原因は一種類ではなかった。NPC d1は
物理geometryではproduction/teacherのcross-run p95内だが、frozen policy embeddingでは
全sampleがp95外で、successor teacherが全sampleにmaterial correctionを要求した。peer d1は
物理geometryが既知である一方、近い別runのteacher actionが51.1%で正負に分かれ、単一scanの
observation aliasingが支持された。peer d3はproduction物理geometryの100%、teacher
geometryの37.9%がp95外で、明確なcoverage不足だった。

この結果から、同じdatasetへのepoch追加、clearance/brake閾値変更、過去のrecurrent候補昇格は
行わない。次のcandidateは、per-beam geometryを保持した表現、許可されたtemporal/state入力、
不足状態のseed-disjoint teacher coverageを組み合わせ、閉ループ試験前に失敗状態とnormal anchorの
識別改善をofflineで証明する。

競技失敗bagは最初のpenaltyより前、かつ0.5 m未満のconfirmed contactより1秒前で切り、
`competition_failure_teacher_v1`として再ラベルした。NPC d1とpeer d1はtrain、peer d3は
validationに固定する。既存10 sequenceとmulti-rootで結合した`recurrent_direct_v3`は、train
9 sequence/29304 sample、validation 4 sequence/12313 sample、同期差最大47.305 msである。
scan最小値は0.4993 mでcontact suffixを含まず、split identity重複もない。次のoffline比較で
peer d3をtrainへ移すことは禁止する。

同じsplitに対するleft/neutral/right補正方向の診断probeを3 seedで比較した結果、frozen
`fc3`＋speedの平均balanced accuracyは0.5245、frozen `conv5`空間map＋speedは0.8779、
同じ空間mapへ1/8 step差分を加えた方式は0.8420だった。material-sign accuracyも順に
0.4656、0.8654、0.8227であり、短時間差分は空間単体を全seedで下回った。したがってこの
evidenceが支持する次の候補は、compact recurrent adapterではなくfrozen baseのfull spatial
featureを使うstatic adapterである。分類probeはcheckpointではなく、continuous correction、
normal anchor、seed-disjoint評価を通るまでruntimeへ接続しない。peer d3のmaterial supportは
右補正16 sampleだけなので、双方向の回避能力を証明したとは扱わない。

速度を除いた`conv5`空間mapも3 seed平均balanced accuracy 0.8709、material-sign
accuracy 0.8617であり、速度ありとの差はそれぞれ+0.0070、+0.0038に留まった。このため
最初のbounded adapterはROS入力を増やさずLiDAR-onlyとした。候補はvalidation material
MAEを36.68%、peer d3を56.63%改善したが、独立normal validationへの補正MAEが0.01939 rad
となり0.01 rad Gateに不合格だった。runtimeへは接続しない。次のデータ契約ではtrain split
のnormal candidate3状態をzero-residual anchorとして明示し、独立normal validation、teacher
validation、peer d3の割当と全Gateを固定したまま再評価する。

normal train 3 sequenceをzero-residual anchorとして追加した再評価では、独立normal MAEは
0.00992 radへ改善してGateを通った一方、material改善は26.44%、方向精度は75.81%へ低下し、
固定した30%/80% Gateに不合格だった。したがってnormal漏れだけを直す重み調整には進まない。
次は同じtrain-only normal anchorと未変更validationを使う分類probeで、通常状態と補正方向が
空間表現上で同時に分離可能かを確認する。分離不能ならinput/data、分離可能ならcontinuous
head/lossの問題として扱う。

normal anchorを含めた3 seed分類では、速度なしstatic spatialはmaterial-sign 85.06%を
維持したが、未見normalの14.56%をmaterialと誤分類した。1/8-step LiDAR履歴でも13.76%に
留まり、material-signは80.42%へ下がった。LiDAR-only static/short-historyでは通常と補正の
観測重なりが残る。速度ありcontrolはnormal datasetに同期速度がなくzeroを代入しているため
採用根拠にしない。次のAI platform作業は元bagからnormal train/validationの速度を同一契約で
同期したimmutable datasetを生成し、実速度で分離性を再監査することとする。

歴史的なv1監査では元bagの`/localization/kinematic_state`をLiDAR時刻へ同期した専用normal
schemaを使ったが、このtopicはfused localizationでありE2Eの許可入力ではない。その結果は
モデル構造の比較に限って残し、production候補の学習・実行根拠には使わない。v1はtrain
3 sequence、validation 1 sequence、計19,714 sample、同期差最大36.12 msだった。実速度を
使った3 seed分類では、static spatialの未見normal誤発火が
14.17%から12.18%へ下がった一方、material-signは85.99%から85.29%へ、balanced
accuracyは86.30%から86.02%へ僅かに低下した。速度は補助信号ではあるが単独の解決策ではない。

frozen base＋full spatial map＋実速度のcontinuous adapterを同じ固定Gateで評価すると、独立
normal MAEは0.00873 radで合格したが、material改善27.82%と方向精度77.28%がそれぞれ
30%/80% Gateを下回った。candidateはruntimeへ接続しない。分類probeが使うtrain-feature
単位の固定標準化と、continuous adapterが使うsample単位LayerNormの契約差を次に監査し、
Gateやruntime brake閾値は変更しない。

分類probeとの正規化差を切り分けるため、teacher/normalのtrain-only conv5統計をcandidate
bufferへ固定したcontinuous adapterも1本評価した。独立normal MAEは0.00927 radで合格したが、
material改善18.88%、方向精度71.28%へ悪化したため不採用とする。per-sample LayerNormが
主因という仮説は反証された。probeにはさらに128次元の固定random projectionがあり、full
conv5 headとの容量差が残るため、次はその差だけを再現する。ここでもGateとproductionは固定する。

probeと同じseeded 128次元projectionまで再現した候補も、material改善21.47%、方向精度
72.41%で不採用だった。projection自体はregistered bufferとして厳密checkpointへ保存するが、
性能原因ではなかった。続いて分類probeだけをproduction同等のsequence-balanced samplerへ
切り替えると、実速度static spatialのmaterial-signが3 seed平均85.29%から70.32%へ低下し、
continuous candidateの72.41%と同じ範囲になった。normal誤発火は12.18%から7.17%へ改善した。
したがって現時点の律速はmulti-task lossより上流のsampling contractである。短い失敗prefixを
消さない目的で各sequenceへ等しい総質量を与えた結果、722～937 sampleの局所runが4,000～
6,000 sampleの成功runと同じ影響を持ち、未見validationの補正方向を損ねている。次はGateを
変えず、sample分布とclass weightingを一致させた候補を1本だけ評価する。

sample samplerとaggregate class massを一致させたprojection候補では、material改善30.51%、
方向精度84.51%となり、sequence samplerで失った両Gateを回復した。sampler root causeは確認
できた。一方、teacher anchor MAEは0.01086 rad、独立normal MAEは0.01486 radで不合格だった。
3-class softmaxがneutral activationとleft/right homotopyを同時に所有しているため、normalを
優先したsequence samplerとmaterialを一般化したsample samplerの間にtrade-offが生じている。
次はsampling質量や閾値を再調整せず、補正activationとconditional signをfactorizeしたheadを
同じrepresentation、data、sample sampler、continuous loss、Gateで1本だけ評価する。

activationとconditional signをfactorizeした候補では、material方向84.07%とteacher anchor
MAE 0.00988 radは合格したが、material改善26.16%、独立normal MAE 0.01355 radで不合格
だった。factorizationはteacher split内のneutral競合を軽減したが、未見normalとの両立を
解決しなかった。sampler、正規化、projection、head分離を独立に反証したため、次にloss重み
やGateを調整することは禁止する。normalへ一律zero residualを与える契約とfailure teacher
残差が、同じLiDAR＋speed観測へ矛盾したlabelを与えていないか、teacher生成経路を監査する。

全corpusへ現行precontact teacherを再適用した結果、保存済みteacher train/validationはbase・
successor steeringとも最大誤差0.0 radで再現し、version driftはなかった。一方、zero-normal
trainの14.03%、validationの10.81%へmaterial補正を要求し、validation平均絶対補正は
0.01970 radだった。さらにzero-normalの元`dagger_aggregate_v2`は4 sequenceすべてが
`lidar_gap_teacher`または`lidar_gap_teacher_dagger`で、production candidate3走行ではなかった。
従来normal anchorと独立normal Gateは誤った状態分布を使用していたため、その結果をcandidate
昇格判断へ使わない。今後のnormal sourceは、checkpoint SHAがcandidate3と一致し、runtime
modeが`fixed_lidar_brake`、所定lap完走、penalty/stall 0を満たすrunだけとする。既存の
`output/20260901-151131`はこの条件を満たすが、run-disjoint train/validationのため追加の
production合格runを収集してからnormal datasetを再構築する。

追加のfrozen production試走`output/20260901-170521`は3/3周、penalty 0、stall 0で完走し、
最速87.626秒、平均91.911秒だった。これをtrain、既存`output/20260901-151131`をvalidation
として、両bagから直接LiDARと実速度を同期した`production_normal_anchor_v1`を生成した。
train 5920 sample、validation 5927 sampleで、同期差最大はそれぞれ17.54 ms、18.38 msである。
builderはcompetition report、runtime mode、checkpoint SHA、Finish/lap、penalty、motion admission、
result hashを再検証し、run/bagのsplit再利用を拒否する。旧gap-teacher由来normal corpusは今後の
candidate admissionに使用しない。

現行precontact teacherを新production normalへ再適用すると、material補正要求はtrain 6.13%、
validation 6.33%残った。これは成功したcandidate3方策とsuccessor teacherが同じ観測でも異なる
ことを示し、teacher補正を通常状態の正解へ一律昇格できない根拠である。sample sampler、full
spatial map、実速度の候補を新contractで学習すると、material改善34.23%、方向84.42%、normal
MAE 0.00994 radだったが、teacher-neutral MAE 0.01104 radで不合格だった。原因に対応して
neutral leakage lossだけを0.5から1.0へ変更した限定A/Bは、material改善30.10%、方向84.94%、
teacher-neutral MAE 0.00847 rad、production normal MAE 0.00586 rad、peer方向100%で全offline
Gateを通過した。checkpoint SHAは
`6ae9d618ea8093b1ff7d212cae760e90c71f84749f986af479681f5f729155d1`であるが、productionへは
未接続である。出力を適用しないshadow runtimeを追加し、candidate内のfrozen baseとproduction
baseの全tensor完全一致、freshな実速度、PyTorch/NumPy数値一致を必須とした。runtime例外や
速度欠損はproduction指令へ波及せず、既定のshadow pathは空である。

最終shadow run `output/20260901-174303`は3/3周、penalty/stall 0で完走し、coverage 99.927%、
shadow error 0、最小scan 19.94 Hz、平均callback 5.90 ms、最大48.04 msだった。補正の区間p95
最大は0.35843 rad、非zero区間は68中64で、候補は実走分布でも単なるzero outputではない。
production checkpoint/mode gateも同時にpassした。独立した先行run `output/20260901-173425`
も3/3周、penalty/stall 0である。これによりshadow実行基盤と候補のruntime可用性は合格とするが、
操舵性能の改善はまだ実証していない。次は既定OFFを維持した限定authority A/Bで判断する。

限定authorityの成功runだけを追加学習し、失敗runをheld-out validationへ固定したDAgger v3は、
失敗直前200 sampleのmaterial方向精度を89.8%から93.9%へ改善し、右側へ開口が変わった後の
5 sample継続応答遅延を0.450秒から0秒へ短縮した。失敗runはgradientにもearly stoppingにも
使っていない。`output/20260901-184620`のshadow再生では3/3周、penalty/stall 0、coverage
99.968%、shadow error 0、最小scan 19.94 Hz、平均推論6.24 msで合格した。checkpoint SHAは
`3b30f567d9a6bdf5384611ff8dfd759d79c8ed683c34e326e7d940afb2e67a5f`である。これはruntime
可用性とheld-out遷移改善の証拠であり、production昇格ではない。次は既定OFFと0.12 rad上限を
維持した、明示的な限定authority A/Bを1本だけ実施する。

同v3を0.12 rad限定authorityへ接続した`output/20260901-185403`は3/3周、penalty/stall 0で
完走し、coverage 99.925%、推論error 0、applied 6640 sample、clipped 97 sampleだった。
ただし3周合計278.611秒は直前shadowの276.427秒より2.184秒遅く、単車ラップ改善は未実証で
ある。default OFFを維持し、次はv3の目的である動的障害物対応を同一0.12 rad上限のNPC
シナリオ1本で判定する。NPC失敗時に上限緩和やruntime特殊条件を追加してはならず、失敗前
sequenceを固定して学習分布または表現の原因へ戻る。

v3限定authorityのruntime NPC試走`output/20260901-190146`では、egoが3/3周を
`102.363 / 89.086 / 95.532秒`で完走し1位、penalty/stall 0となった。coverage 99.954%、
推論error 0、clippedは8758 applied sample中136だった。v2の同シナリオ
`output/20260901-180313`は2/3周後にwall penalty 137.35秒で未完だったため、held-out
direction-transition改善は閉ループでも支持された。ただし単発runであり、production既定は
candidate3＋`fixed_lidar_brake`、spatial authority OFFのままとする。次は同一設定の独立repeatを
行い、再現した場合だけpromotion可否を判断する。

独立repeatとして開始配置seedを2027へ変更した`output/20260901-191114`も、egoが3/3周を
`102.423 / 89.655 / 102.593秒`で完走し1位、penalty/stall 0となった。最小LiDAR距離は
left-front 0.770 mまで低下したが固着せず、coverage 99.932%、推論error 0だった。seed 2026と
2027の2本平均は3周合計290.827秒である。これによりv3の動的障害物改善は再現したと判定する。
次は値調整ではなく、v3 artifactを提出package内へ固定し、single learned steering owner、
SHA検証、既定値、rollbackを含むpromotion contractを監査してからproduction昇格を判断する。

runtime NPCを含むAWSIM v2 summaryは複数vehicleを`vehicle_number=1`として出力する場合が
ある。この場合、domain identityの正本はv3の`dN-result-details.json`とし、summaryは同じ
Finish/lap状態のentryが存在することだけをcross-checkする。summaryの先頭entryを無条件に
domain結果として採用しない。

### Wheel-speed input correction

E2E runtime input contractの再監査により、spatial adapterの速度入力を
`/localization/kinematic_state`から
`/vehicle/status/velocity_status`（`VelocityReport.longitudinal_velocity`）へ変更した。
dataset builderの既定topic/typeも同じ値へ統一し、旧Odometryは明示的なlegacy再現時にだけ
選択できる。held-out authority failure sequenceはimmutable sequence IDでtrainから除外した。

同じv3 architecture、frozen base、0.12 rad authority上限で再学習したwheel-speed v4のSHAは
`838565279cedf8f29539005d90ff9ad06d031a608939ebc06c48331fc2fa780f`である。単車run
`output/20260901-193157`は3/3周、penalty/stall 0、3周合計276.902秒で、旧v3 authorityの
278.611秒より1.709秒速く、authority OFF基準との差は0.475秒だった。runtime coverageは
100%、error/stale 0、最小scan 19.88 Hzである。NPC 2 seedを通過し、提出package内のartifact
SHA、launch既定値、rollback、tar内容を検証するまではproduction昇格と扱わない。

### Full-steering spatial authority promotion

wheel-speed v4とbase-conditioned v10は、NPC失敗時に必要な教師補正が
`+0.83--0.88 rad`であるのに対し、modelとruntimeの補正範囲が`+/-0.12 rad`だったため
productionへ昇格しない。v10は失敗区間で正しい回避方向を選んだが、平均補正が
`+0.11986 rad`に張り付いた。これは閾値やタイムアウトでは解決できない表現能力の不足である。

この根本原因に対し、frozen base steeringを入力に含めたまま補正範囲を`+/-1.2 rad`へ拡張した
v11を学習した。外部ML入力は2D LiDARと
`/vehicle/status/velocity_status`の車輪速度だけである。qualified artifactは
`spatial_steering_adapter.npy`、SHA256は
`f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`である。

厳格offline監査では、aggregate material MAE改善35.99%、runtime-bounded方向一致94.95%、
peer改善68.53%・方向一致100%、held-out focus改善37.59%・方向一致95.68%、独立normal
MAE 0.009699 radで全Gateを通過した。shadow単車、authority単車、NPC seed 2026/2027は
いずれも3周完走、penalty/stall 0、coverage 100%、inference error/stale 0だった。NPC 2本は
ともに1位で、最大実補正は0.86176 radだった。したがって旧0.12 rad候補はfallbackとしても
残さず、v11だけをlearned steering correction ownerとする。

participant launchは提出package内のartifact path、期待SHA、base-steering conditioning、
model/runtime上限1.2 rad、authority ONをproduction既定として所有する。DockerとMakefileの
未指定環境変数はこの既定値を空値で上書きしない。明示的なcustom artifactはauthorityを同時に
明示しない限りshadow-onlyであり、`gap_teacher`と`precontact_teacher`は通常起動経路で
authorityを継承しない。legacy residual pathは空のままとする。

提出package既定だけを使った
`output/20260901-e2e-full-steering-packaged-default-single`は、3周
`101.069 / 89.156 / 90.400秒`、penalty/stall 0、6439/6439 authority適用、
clipping/stale/error 0で合格した。source、install、提出tar内artifactのSHAは一致し、tarの
最上位は`aichallenge_submit/`だけである。

明示的なrollbackは次で行う。この場合もmodelをshadow観測には利用できるが、最終操舵は
frozen baseへbit-for-bitで戻る。

```bash
make e2e-single TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED=false
```

authority単車はshadow基準より約6秒/3周遅いため、周回速度は後続Sliceの課題である。ただし
性能調整のために、物理的に教師を表現できない0.12 rad制約へ戻したり、障害物固有のtriggerを
追加したりしてはならない。

周回速度低下の後続監査では、authority走行の加速度指令にshadowとの差や負加速度はなく、
操舵total variationが約11.6%増えていた。current-distributionのbase-authority runを
zero-residual normal anchorとして追加したv12は、独立normal MAEを0.009699から
0.007926 radへ改善した一方、凍結した壁回避failure末尾のmaterial改善を96.01%から
12.55%へ悪化させた。中間のsequence balanceを使ったv13もmaterial改善28.75%で既存Gateに
不合格だった。

このため、normal anchorの比率調整はここで打ち切り、v11をproductionに維持する。現状の
課題はruntime閾値ではなく、static single-frame soft mixtureがnormalの無介入とfailure時の
大補正を同時に分離する能力にある。次候補はneutral/no-interventionを明示的に学習する出力
構造、または時系列表現を独立Sliceで評価し、凍結failure replayを必須Gateに含める。

明示neutralを持つ`categorical_expert`を同じdata/representationで1本評価したが、
winner-take-all decodeはaggregate material改善34.33%、方向89.73%を維持する一方、anchor
MAE 0.010205 rad、独立normal MAE 0.015161 radで不合格だった。neutral選択時は厳密に0となるが、
約5.78%のfalse left/right選択で完全なside magnitudeを出すため、soft mixtureより誤分類の
影響が大きい。runtimeへwinner-take-allを追加せず、次の独立変数は安定したmode選択に必要な
時系列情報とする。

current teacher/normal corpusでcausal spatial lagを3 seed再評価した結果、static
conv5+speedの平均balanced accuracy/material方向0.8888/0.8889に対し、temporalは
0.8601/0.8497へ全seedで悪化した。base steeringを加えた比較でもstatic 0.8991/0.9046、
temporal 0.8737/0.8721だった。短い履歴やcompact GRUを再実装する根拠はなく、次はteacher
correctionとproduction-normal zero labelの観測衝突を監査する。

同じv11入力でteacher materialとproduction-normal zero labelの最近傍を監査したところ、
material teacher状態の8.23%が成功normal run間の自然距離p50内、29.32%がp95内に入った。
逆向きも7.62% / 44.04%である。random projectionを使わない50-binのLiDAR最小・平均距離、
車輪速度、base steeringでも3.20% / 19.80%と2.98% / 28.08%の衝突が残った。したがって
圧縮表現は衝突を増幅するが、根本はstatic observationに対するteacher補正と成功normalの
zero-intervention labelが一部矛盾していることである。

物理geometry表現を同一classifier protocolで3 seed評価してもbalanced accuracyは全seedで
既存spatial+baseを下回り、normal false-materialは2 seedで増加し、凍結failure tail方向精度も
不安定だった。このため新modelの学習やruntime threshold追加は行わずv11を維持する。次は
immutable sample単位でteacher/normal conflictを特定し、重要なfailure tailを保持したまま
曖昧labelをadmission対象外にできるかをデータ契約として監査する。

sample単位監査では、現行`LidarPrecontactTeacher`が保存teacher補正を最大`4.77e-7 rad`誤差で
再現し、version driftを否定した。一方、成功normal 4,500 sampleの6.20%へmaterial補正を要求し、
理由は`side-clearance` 169、`gap-selected` 110だった。さらに4台固着run
`output/20260901-121938/d2`の末尾200 material sampleは、128次元projection後には60%がnormal
p50内だが、物理geometryでは0%だった。したがってprojected距離によるconflict除外は必要な
failure教師を削除するため禁止する。physical距離だけで除外しても、production表現が既に情報を
失っているため根治にならない。

次の独立変数はteacher thresholdやnormal比率ではなく、frozen conv5の1,088次元空間を128次元へ
固定random projectionする契約である。同じsplit、speed、base steering、classifier、3 seed、
focus/peer Gateでfull conv5表現を先に診断し、厳格改善がなければ学習・runtime統合へ進めない。

full conv5診断ではaggregate p50 conflictが8.26%から5.97%へ減ったが、4台固着run d2末尾は
60%から61%となり改善しなかった。3 seed分類でもprojected+baseの平均balanced/material方向
0.89906/0.90455に対し、full conv5+baseは0.89370/0.89382で全seed悪化した。normal false-
materialは0.08959から0.08802へ僅かに改善しただけで、full-map adapterを学習する根拠はない。

固定projectionはaggregate衝突の一因だが、通常走行用に凍結したbase conv5自体が既知failureの
回避幾何を保持していない。次はruntime modelを変えず、物理LiDARへ局所畳み込みを学習する
correction専用action probeを3 seed評価する。raw/geometry MLPとfrozen conv5の両方を厳格に
上回れない場合、static LiDAR＋speed＋base契約の限界と判定し、値調整へ戻らない。

物理scanのangular localityを保持する4層1D CNN action probeも、3 seed平均で
balanced/material方向が0.83441/0.84131、normal false-materialが0.12795となり、projected
frozen+baseの0.89906/0.90455/0.08959を全て悪化させた。focus tail方向も2 seedで失敗したため、
static correction専用CNNからauthority candidateを作らない。

これによりstatic表現についてはprojected/full frozen conv5、raw/geometry MLP、trainable local
CNNを反証した。次の独立変数は幅やprojectionではなく、sequence境界とego motionを守る
spatiotemporal geometry encoderによるcausal observabilityである。これも同じrun-disjoint
3 seed分類で先に診断し、不合格ならLiDAR/speed観測に対するteacher/zero-normal label契約自体を
再設計する。

物理LiDARを局所1D CNNで符号化し、車輪速度とfrozen base steeringを加えたtokenを
unidirectional GRUへ入力するcausal probeを3 seed評価した。aggregate balanced/material方向は
0.92462/0.94894まで改善したが、成功normalへのfalse-materialはstatic baselineの0.08959から
0.11574へ全seedで悪化し、凍結focus末尾の方向精度も1 seedで0.50へ落ちた。よってruntime
checkpointやcontinuous residual候補へ昇格しない。

projected/full frozen conv5、raw/geometry MLP、trainable local CNN、短期lag、causal local
CNN+GRUまで反証したため、同じlabelのままモデル容量を増やす探索はここで終了する。次は成功runの
zero-interventionとheuristic teacherの補正要求を単純に混合せず、run outcomeと「介入が必要だった
か」を明示する教師・正常ラベル契約を独立Sliceで設計する。failure tailを最近傍除外すること、
runtime thresholdを追加すること、normal比率だけを再調整することは禁止する。

## Submission Artifacts

公開案内では、取り組みスライドと走行動画を提出する。スライドには少なくとも、
走行データ、他車両への回避・停止、モデル構成、学習データと評価、独自性を記載する。
提出フォーマットと期限は WIP のため、最新の公式ページと運営連絡を提出前に再確認する。
