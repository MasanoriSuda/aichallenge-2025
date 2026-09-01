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

bag単位の固着監査は次で行う。起動待ちは除外し、一度1.0 m/s以上で走行した後の
0.15 m/s以下の連続時間と、そのうち正加速指令中の連続時間を別々に判定する。縦安全層が
正しく加速を抑止しても、その場で停止し続けるcandidateを成功扱いしない。GUIの見た目
ではなく、このJSONとAWSIM Finish/接触結果を合わせてcandidate admissionを判断する。

```bash
docker compose run --rm --no-deps autoware-command \
  python3 /aichallenge/ml_workspace/tiny_lidar_net/analyze_e2e_run.py \
  /output/<run>/d1/rosbag2_autoware --fail-on-stall
```

## Submission Artifacts

公開案内では、取り組みスライドと走行動画を提出する。スライドには少なくとも、
走行データ、他車両への回避・停止、モデル構成、学習データと評価、独自性を記載する。
提出フォーマットと期限は WIP のため、最新の公式ページと運営連絡を提出前に再確認する。
