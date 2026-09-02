# TinyLiDARNet Workspace

このworkspaceでは、[TinyLidarNet](https://arxiv.org/abs/2410.07447)用のデータ変換・学習・deployコードを提供しています。

- 参考: [TinyLidarNet: 2D LiDAR-based End-to-End Deep Learning Model for F1TENTH Autonomous Racing](https://arxiv.org/abs/2410.07447)

- TinyLiDARNetについての解説は[こちら](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/ml_sample/algorithms.html#tinylidarnet)を参照してください。

- TinyLiDARNetの実行用コードは、[tiny_lidar_net_controller](../workspace/src/aichallenge_submit/tiny_lidar_net_controller)を参照してください。

## 学習用データの作成

以下2つのtopicを含むrosbagを記録した後、`extract_data_from_bag.py`を実行します。

- [`sensor_msgs/msg/LaserScan`](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg) : 2D LiDAR点群のtopic
- [`autoware_auto_control_msgs/msg/AckermannControlCommand`](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_control_msgs/msg/AckermannControlCommand.idl) : 学習のtarget(教師)となる、アクセルとステアリングの情報を含むtopic

教師commandの出所は必ず`--label-source`で指定します。MPC/MPCCの教師bagなら次のように
抽出します。

```bash
python3 extract_data_from_bag.py \
  --bags-dir /path/to/record/ \
  --outdir ./dataset \
  --label-source mpcc \
  --max-sync-delta-sec 0.05 \
  --val-fraction 0.2 \
  --split-seed 2026
```

同一runのsampleがtrainとvalidationへ混ざらないよう、bag単位で決定論的に
`dataset/train/<sequence_id>`または`dataset/val/<sequence_id>`へ出力します。各sequence
には同期誤差、元bag、topic型、scan shape、label sourceを記録した`metadata.json`を
保存します。同じ`rosbag2_autoware`というbasenameを持つ複数runも別IDになります。

E2E自身が失敗した走行を観測用に抽出する場合は`--label-source student`とします。
student commandはcorrective labelではないため、trainerの既定設定では教師として拒否
されます。壁スタック等の失敗bagは、MPC/MPCCまたは人間による正解commandを付け直して
から教師へ追加してください。

## 学習

trainerは学習開始前に次を検証します。

- 750点LiDAR、30 m range契約
- scan/controlの同期差が50 ms以下
- `label_source`が`mpc`、`mpcc`、`human`、`lidar_gap_teacher`、
  `lidar_gap_teacher_dagger`、`lidar_precontact_teacher_dagger`のいずれか
- 全配列の長さ、finite値、timestamp順序
- train/validation間のsequence ID非重複

古いmetadataなしdatasetは既定では読み込まれません。上記extractorで再抽出してください。

`train.loss.accel_weight`を`0.0`にすることで、steeringのみ学習できます。現baselineは
MLがsteeringを所有し、`fixed_lidar_brake`が固定加速度をLiDAR前方距離だけで安全側へ
制限するため、まずsteeringのみの学習を推奨します。
`train.pretrained_path`には`.pth/.pt`のraw state dictだけでなく、提出runtimeで使う
`.npy/.npz`weightも指定できます。全parameterのkey、shape、finite値を検証してから
warm-startします。各学習runはtimestamp directoryへ保存され、dataset sequence ID、設定、
pretrained checkpointのSHA-256を`training-manifest.json`へ記録します。
希少なcorrective labelで既存feature extractorを壊したくない場合は、
`train.trainable_layers=[fc4]`のように最終層だけを明示できる。空配列は従来どおり全層を
学習し、選択内容はmanifestへ保存される。

```bash
python3 train.py \
data.train_dir=/path/to/train_dir \
data.val_dir=/path/to/val_dir \
model.name='TinyLidarNet' \
train.loss.steer_weight=1.0 \
train.loss.accel_weight=0.0
```

## 重みの形式変換
採点環境において実行できるように、pytorchではなくnumpyを用います。そのため、`.pth`から`.npy/.npz`に重みを変換します。
```bash
python3 convert_weight.py --model tinylidarnet --ckpt ./ckpts/weight.pth
```

変換時もtrainingと同じcheckpoint契約（全key、shape、finite値）を検証します。閉ループ確認前に
production重みを上書きせず、コンテナから見えるcandidate pathを明示してA/Bできます。

独立したvalidation runに対し、production重みより候補のsteering MAEが改善したことを
閉ループ試験前に確認できます。sequence IDとcheckpoint SHA-256をJSONへ保存します。

```bash
python3 evaluate_checkpoint.py \
  --dataset-dir dataset/<name>/val \
  --candidate checkpoints/<run>/candidate.npy \
  --baseline /aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy \
  --output checkpoints/<run>/offline-evaluation.json \
  --require-rmse-improvement
```

全sampleのMAE/RMSE/P95に加え、production出力と教師labelが0.02 rad以上異なる
corrective subsetを別集計します。希少な回避labelを通常走行sampleの平均に埋没させません。

```bash
make e2e-single \
  TINY_LIDAR_CKPT_PATH=/aichallenge/ml_workspace/tiny_lidar_net/checkpoints/<run>/candidate.npy
```

override未指定時はpackage同梱のproduction重みを使います。存在しないpathや、
`tiny_lidar_net`以外のcontrollerとの組み合わせは起動前に拒否されます。

### Closed-loop run admission

単車・NPC試走後は、一度走行を開始した後に実車速がほぼ0へ固着していないかをbagから
確認します。全post-start低速時間と正加速指令中の低速時間を別々に判定するため、LiDAR
安全層が加速を止めただけで回避できていないrunも失敗になります。これは接触や壁拘束を
「センサ停止」と誤認しないための最低限の自動gateです。Finish、接触、周回時間の判定を
置き換えるものではありません。

```bash
docker compose run --rm --no-deps autoware-command \
  python3 /aichallenge/ml_workspace/tiny_lidar_net/analyze_e2e_run.py \
  /output/<run>/d1/rosbag2_autoware \
  --output /output/<run>/d1/e2e-run-analysis.json \
  --fail-on-stall
```

AWSIM結果を含む最終判定は、motion JSONを生成した後にrun単位で実行します。結果JSONを
持たない旧runは成功扱いせず`incomplete`になります。

```bash
python3 analyze_e2e_competition.py /output/<run> \
  --expected-control-mode fixed_lidar_brake \
  --expected-acceleration-mps2 0.8 \
  --expected-maximum-forward-speed-mps 4.6 \
  --expected-checkpoint-path \
    /aichallenge/workspace/install/tiny_lidar_net_controller/share/tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy \
  --checkpoint-file \
    /aichallenge/workspace/install/tiny_lidar_net_controller/share/tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy \
  --expected-checkpoint-sha256 \
    de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa \
  --expected-residual-checkpoint-path '' \
  --expected-spatial-checkpoint-path \
    /aichallenge/workspace/install/tiny_lidar_net_controller/share/tiny_lidar_net_controller/ckpt/spatial_steering_adapter.npy \
  --spatial-checkpoint-file \
    /aichallenge/workspace/install/tiny_lidar_net_controller/share/tiny_lidar_net_controller/ckpt/spatial_steering_adapter.npy \
  --expected-spatial-checkpoint-sha256 \
    f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c \
  --expected-spatial-use-base-steering true \
  --expected-spatial-authority-enabled true \
  --expected-spatial-authority-max-abs-delta-rad 1.2 \
  --expected-recurrent-checkpoint-path '' \
  --expected-recurrent-authority-enabled false \
  --output /output/<run>/e2e-competition-analysis.json \
  --fail-on-rejection
```

提出動画とproduction freezeではraw artifactだけでなく、spatial path/SHA/authorityと
recurrent無効状態まで同じコマンドで固定する。これらの期待値を省略した解析はhistorical runの
診断には使えるが、提出動画のidentity証明としては扱わない。

固定加速度を比較する場合は、既定値を直接編集せずrunごとに明示します。値はstartup logと
competition analysisへ残るため、異なる加速度の結果を同じ設定として扱いません。

```bash
make e2e-single \
  LOG_DIR=/output/<run> \
  TINY_LIDAR_ACCELERATION=0.8

python3 analyze_e2e_competition.py /output/<run> \
  --expected-control-mode fixed_lidar_brake \
  --expected-acceleration-mps2 0.8 \
  --expected-checkpoint-path \
    /aichallenge/workspace/install/tiny_lidar_net_controller/share/tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy \
  --checkpoint-file checkpoints/20260901_055824/candidate.npy \
  --expected-checkpoint-sha256 \
    de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa \
  --output /output/<run>/e2e-competition-analysis.json \
  --fail-on-rejection
```

無制限の`0.8 m/s2`は旧試験で最高`6.515 m/s`まで加速してwall penaltyと長時間停止を
起こしたため不採用です。`1.0 m/s2`も同じ理由で不採用です。固定加速度だけを上げず、
packaged defaultはcross-seed NPC Gateに合格した`0.8 m/s2`と`4.6 m/s`上限の組み合わせです。
比較runでは`TINY_LIDAR_ACCELERATION`、`TINY_LIDAR_MAXIMUM_FORWARD_SPEED_MPS`と対応する
analyzer引数を対にしてruntime identityを固定します。

前進速度上限は固定加速度の立ち上がりと定常速度を分離します。正の加速度だけを減らし、
LiDAR安全層の制動や負加速度を弱めません。速度が欠損・staleなら正の加速度を禁止します。
診断時に`0.0`を明示すれば無効化できますが、production既定は`4.6 m/s`です。

```bash
make e2e-npc-single \
  LOG_DIR=/output/<run> \
  E2E_START_RANDOM_SEED=2035 \
  TINY_LIDAR_ACCELERATION=0.8 \
  TINY_LIDAR_MAXIMUM_FORWARD_SPEED_MPS=4.6

python3 analyze_e2e_competition.py /output/<run> \
  --expected-control-mode fixed_lidar_brake \
  --expected-acceleration-mps2 0.8 \
  --expected-maximum-forward-speed-mps 4.6 \
  --fail-on-rejection
```

この組み合わせは実seed 2035/2036で各3周完走、1位、penalty 0、stall 0となったため
productionへ昇格しました。合計周回時間はそれぞれ256.49秒、255.87秒です。昇格後に
環境変数を指定しないpackaged defaultでもseed 2037を255.65秒、penalty/stall 0で完走し、
launch既定値まで同じauthorityが到達することを確認しています。

競技runが失敗した後は、checkpointを再学習する前に失敗直前状態のcoverageを監査できます。
`--failure`は`run:dN`形式で複数指定し、最初のpenalty前10秒をproduction学習分布と
admitted precontact-teacher分布へ照合します。出力は診断専用で、checkpoint昇格を許可しません。

```bash
python3 analyze_e2e_state_coverage.py \
  --checkpoint checkpoints/20260901_055824/candidate.npy \
  --production-dataset dataset/dagger_aggregate_v2 \
  --teacher-dataset dataset/precontact_residual_base_v4 \
  --failure /output/<npc-run>:d1 \
  --failure /output/<peer-run>:d1 \
  --failure /output/<peer-run>:d3 \
  --output /output/e2e-state-coverage-audit.json
```

異なるadmitted sourceを時間datasetへまとめる場合、配列をコピーして出所を失わないよう
`--additional-source-root`を繰り返します。builderは全source identityを出力前に検査し、
重複runを拒否します。各immutable rootがtrainまたはvalの片方だけを持つことは許しますが、
全rootの集合に両splitが揃わなければ拒否します。速度入力の既定はE2E許可入力である
`/vehicle/status/velocity_status`（`autoware_auto_vehicle_msgs/msg/VelocityReport`）です。
`/localization/kinematic_state`はfused localizationなので、新しいproduction datasetや
runtime inputには使用しません。holdoutを再学習から隔離する場合は
`--exclude-source-sequence-id`へimmutable sequence IDを指定し、存在しないIDや重複指定は
builderに拒否させます。

```bash
python3 build_recurrent_dataset.py \
  --source-root dataset/precontact_residual_base_v4 \
  --additional-source-root dataset/competition_failure_teacher_v1 \
  --output-root dataset/recurrent_direct_v3
```

新しいsteering modelを学習する前に、frozen baseのcompact feature、spatial feature、
spatial＋短時間差分のどれがsuccessor teacherの補正方向を識別できるか診断できます。
これは分類probeであり、runtime checkpointは生成しません。

```bash
python3 probe_e2e_action_separability.py \
  --dataset dataset/recurrent_direct_v3 \
  --checkpoint checkpoints/20260901_055824/candidate.npy \
  --output /output/e2e-action-separability-probe.json
```

通常走行との同時分離に使うnormal sourceは、teacher rolloutではなく、frozen production
checkpoint自身が完走したrunに限定します。`e2e-competition-analysis.json`がpassで、runtime
mode/checkpointが一致し、所定lap完走、penalty 0、stall 0であることをbuilderが再検証します。
train/validationへ同じrunまたはbagを再利用できません。生成物には操舵labelを保存せず、
各LiDAR時刻へ50 ms以内で同期できる実速度と、run/result/checkpointのhash provenanceだけを
保存します。

```bash
python3 build_production_normal_anchor_dataset.py \
  --run train:/output/<admitted-production-run-a> \
  --run val:/output/<admitted-production-run-b> \
  --expected-checkpoint-sha256 <frozen-checkpoint-sha256> \
  --output-root dataset/production_normal_anchor_v1

python3 probe_e2e_action_separability.py \
  --dataset dataset/recurrent_direct_v3 \
  --checkpoint checkpoints/20260901_055824/candidate.npy \
  --normal-recurrent-root dataset/production_normal_anchor_v1 \
  --output /output/e2e-spatial-normal-speed-separability.json
```

`build_normal_anchor_recurrent_dataset.py`と`dagger_aggregate_v2`由来のnormal corpusは過去結果の
再現・監査専用です。gap-teacherが作った状態をfrozen productionのzero residualとして扱うため、
新candidateのadmissionには使用しません。

probeが空間表現を支持した場合も、次の候補はoffline限定で学習・評価します。candidate
artifactにfrozen baseを含め、評価時にbase tensorの同一性、teacher validation、peer subset、
独立normal leakageを同時に検査します。

```bash
python3 train_spatial_adapter.py \
  --dataset dataset/recurrent_direct_v3 \
  --base-checkpoint checkpoints/20260901_055824/candidate.npy \
  --output-root checkpoints/spatial-adapter-v1

python3 evaluate_spatial_adapter.py \
  --dataset dataset/recurrent_direct_v3 \
  --candidate checkpoints/spatial-adapter-v1/<run>/candidate.npy \
  --base-checkpoint checkpoints/20260901_055824/candidate.npy \
  --normal-dataset-dir dataset/dagger_aggregate_v2/val \
  --output /output/e2e-static-spatial-adapter-gate.json \
  --fail-on-gate
```

速度ありcandidateもまずoffline限定で評価します。`--use-speed`時は、速度を0で代用した
legacy normal datasetを禁止し、同期済みnormal recurrent rootを必須とします。

```bash
python3 train_spatial_adapter.py \
  --dataset dataset/recurrent_direct_v3 \
  --base-checkpoint checkpoints/20260901_055824/candidate.npy \
  --normal-recurrent-root dataset/production_normal_anchor_v1 \
  --use-speed \
  --output-root checkpoints/spatial-speed-adapter-v1

python3 evaluate_spatial_adapter.py \
  --dataset dataset/recurrent_direct_v3 \
  --candidate checkpoints/spatial-speed-adapter-v1/<run>/candidate.npy \
  --base-checkpoint checkpoints/20260901_055824/candidate.npy \
  --normal-recurrent-root dataset/production_normal_anchor_v1 \
  --use-speed \
  --output /output/e2e-spatial-speed-adapter-gate.json \
  --fail-on-gate
```

offline Gate合格後もcandidateを直接productionへ接続しません。まずcandidate3を公開制御のまま
維持し、空間adapterをshadowとして実行します。checkpoint内の`base_*`がproduction baseと
1 tensorでも異なる場合は起動を拒否します。速度が100 ms以内に届かないsampleは0 m/sで代用
せずshadowだけskipします。

```bash
make e2e-single \
  TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH=/aichallenge/ml_workspace/tiny_lidar_net/checkpoints/<spatial-run>/candidate.npy

python3 analyze_spatial_shadow_run.py /output/<run> \
  --domain 1 \
  --competition-report /output/<run>/e2e-competition-analysis.json \
  --checkpoint-file checkpoints/<spatial-run>/candidate.npy \
  --expected-checkpoint-sha256 <spatial-candidate-sha256> \
  --expected-runtime-checkpoint-path \
    /aichallenge/ml_workspace/tiny_lidar_net/checkpoints/<spatial-run>/candidate.npy \
  --output /output/<run>/e2e-spatial-shadow-analysis.json \
  --fail-on-rejection
```

shadow Gateは3周完走、penalty 0、frozen production Gate合格、coverage 99%以上、error 0、
LiDAR 19 Hz以上、watchdog stale 0、finiteかつ非zeroの診断出力を同時に要求します。shadowの
補正は`/control/command/control_cmd`へ加算されません。

### Offline projected-conv5 recurrent adapter

stateful teacherの時間方向の判断を候補化するときは、productionをraw TinyLidarNetへ戻さず、
packaged spatial adapterまで含む現在のproduction steeringを固定baseとして扱います。
recurrent側へ渡す空間表現は、診断probeで選定したfrozen `conv5`の決定論的random projectionです。
projectionとtrain-only normalization statisticsはcheckpoint bufferへ保存し、raw TinyLidarNetと
spatial adapterの全tensorを学習対象から外します。独立production-normal train/val runでは、
記録済みcommandを架空の教師にせず、recurrent correctionが厳密に0へ戻ることを学習・評価します。

```bash
python3 train_recurrent_policy.py \
  --train-dir dataset/<speed-recurrent>/train \
  --val-dir dataset/<speed-recurrent>/val \
  --normal-recurrent-root dataset/<production-normal> \
  --model-type frozen_tinylidar_adapter \
  --base-checkpoint \
    /aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy \
  --production-spatial-checkpoint checkpoints/<production-spatial>/candidate.npy \
  --adapter-spatial-features projected_conv5 \
  --adapter-spatial-normalization fixed_train_statistics \
  --no-adapter-use-speed \
  --distillation-epochs 0 \
  --output-root checkpoints/<recurrent-run>

python3 evaluate_recurrent_policy.py \
  --checkpoint checkpoints/<recurrent-run>/<timestamp>/best_model.pth \
  --base-checkpoint \
    /aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy \
  --production-spatial-checkpoint checkpoints/<production-spatial>/candidate.npy \
  --val-dir dataset/<unseen-speed-recurrent>/val \
  --normal-recurrent-root dataset/<production-normal> \
  --correction-deadband-rad 0.02 \
  --unseen-source-bag-token <immutable-run-token> \
  --output /output/<run>/recurrent-offline-gate.json
```

`--correction-deadband-rad`はraw recurrent correctionへ適用してからproduction baseと合成し、
最後にsteering範囲へclampするdeployment契約です。validationに合わせてdeadbandを探索し続けず、
material labelの既定境界である0.02 radを先に固定し、その後に取得した未見runで一度だけGateする。
Gate合格はoffline shadow候補の資格であり、runtime authority昇格ではありません。runtime接続、
watchdog、hidden-state reset、閉ループ試走は別sliceで審査します。

recurrent datasetの因果車速ageは既定で50 ms以下を要求し、より緩いdatasetを暗黙には読みません。
実行済みteacherが別のruntime freshness契約を持つ場合だけ、学習・評価の両方へ
`--max-speed-sync-delta-sec`を明示します。選択値はtraining manifestとevaluation reportへ保存されます。

同じ成功runから複数domainのsequenceを追加した場合、sequence数を独立world数として扱いません。
研究用の`--outcome-run-balanced-successor`は、証明済み
`outcome_certificate.source_run_id`ごとにsampling massを均等化し、identity欠損時は拒否します。
これはopt-inであり、自然samplingや既存のsequence-balanced modeの既定動作を変えません。

runtimeはPyTorchへ依存させません。合格した`.pth`を、frozen raw TinyLidarNetとpackaged
spatial production baselineを埋め込んだNumPy artifactへ変換します。converterは対応外の
architecture、学習可能なproduction baseline、pressure duplicationを拒否します。

```bash
python3 convert_recurrent_policy.py \
  --checkpoint checkpoints/<recurrent-run>/<timestamp>/best_model.pth \
  --output checkpoints/<recurrent-run>/<timestamp>/candidate.npy \
  --manifest checkpoints/<recurrent-run>/<timestamp>/candidate-manifest.json

make e2e-single \
  TINY_LIDAR_RECURRENT_SHADOW_CKPT_PATH=/aichallenge/ml_workspace/tiny_lidar_net/checkpoints/<recurrent-run>/<timestamp>/candidate.npy \
  TINY_LIDAR_RECURRENT_SHADOW_EXPECTED_SHA256=<recurrent-candidate-sha256>

python3 analyze_recurrent_shadow_run.py /output/<run> \
  --checkpoint-file checkpoints/<recurrent-run>/<timestamp>/candidate.npy \
  --expected-checkpoint-sha256 <recurrent-candidate-sha256> \
  --expected-runtime-checkpoint-path \
    /aichallenge/ml_workspace/tiny_lidar_net/checkpoints/<recurrent-run>/<timestamp>/candidate.npy \
  --expect-async-shadow true \
  --output /output/<run>/e2e-recurrent-shadow-analysis.json \
  --fail-on-rejection
```

recurrent shadowは公開操舵へ一切加算しません。起動時にembedded raw/spatial baselineの
identityを厳密検証します。authority無効時は、検証済みproduction Conv5特徴量を1件だけ保持する
latest-wins workerへ渡し、production commandをpublishした後に診断推論します。worker backlogは
古いpending sampleをdropしますがproduction commandやwatchdogへ影響させません。車輪速度欠損は
診断sampleをskipし、LiDAR watchdogと推論例外ではhidden stateを破棄して古いepisodeを
持ち越しません。runtime logはsubmitted/completed/dropped/stale/errorを分離します。
runtime Gateは3周完走、
penalty 0、frozen production Gate合格、coverage 99%以上、error 0、LiDAR 19 Hz以上、連続成功run
でhidden reset 0、spatial production authority維持を同時に要求します。

shadow Gate合格後の限定authority試験は、checkpointに加えてauthorityを明示し、補正上限を
固定して起動します。checkpointだけを指定した場合は引き続きshadow-onlyです。既定値は
`false`であり、recurrent artifactは提出packageへ同梱しません。

```bash
make e2e-single \
  LOG_DIR=/output/<run> \
  TINY_LIDAR_RECURRENT_SHADOW_CKPT_PATH=/aichallenge/ml_workspace/tiny_lidar_net/checkpoints/<recurrent-run>/<timestamp>/candidate.npy \
  TINY_LIDAR_RECURRENT_SHADOW_EXPECTED_SHA256=<recurrent-candidate-sha256> \
  TINY_LIDAR_RECURRENT_AUTHORITY_ENABLED=true \
  TINY_LIDAR_RECURRENT_AUTHORITY_MAX_ABS_CORRECTION_RAD=0.24

python3 analyze_recurrent_shadow_run.py /output/<run> \
  --checkpoint-file checkpoints/<recurrent-run>/<timestamp>/candidate.npy \
  --expected-checkpoint-sha256 <recurrent-candidate-sha256> \
  --expected-runtime-checkpoint-path \
    /aichallenge/ml_workspace/tiny_lidar_net/checkpoints/<recurrent-run>/<timestamp>/candidate.npy \
  --expect-authority true \
  --expected-authority-max-abs-correction-rad 0.24 \
  --output /output/<run>/e2e-recurrent-authority-analysis.json \
  --fail-on-rejection
```

限定authorityは、同一周期で検証済みのspatial production操舵へ有限なrecurrent correctionだけを
加算します。加速度、安全停止、watchdogのauthorityは持ちません。速度欠損・identity不一致・
推論例外時はhidden stateとrecurrent authorityを破棄し、その周期のspatial production操舵を
維持します。単車3周の後、NPC Gateでも非劣化を確認するまではproduction昇格しません。

### Qualified production spatial adapter

production既定は、車輪速度とfrozen base steeringでconditionしたfull-range adapterです。
提出package内のartifactは
`tiny_lidar_net_controller/ckpt/spatial_steering_adapter.npy`、SHA256は
`f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`です。modelとruntimeの
補正範囲は`+/-1.2 rad`です。旧`+/-0.12 rad`候補はNPCで必要な操舵を表現できなかったため
fallbackとしてもproductionへ残していません。

通常の`make e2e-single`はparticipant launchのpackaged defaultを使います。ML workspaceの
checkpoint pathやhost環境変数は不要です。明示的なrollbackは次です。

```bash
make e2e-single TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED=false
```

別candidateを診断するときは、checkpoint pathを指定しただけではauthorityを得られません。
まずshadowでGateを通し、限定authority試験を行う場合だけ
`TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED=true`も明示します。これにより開発用artifactが
production既定のauthorityを暗黙に継承することを防ぎます。

### Runtime NPC corrective teacher

runtime NPCはV2Xへ現れないため、MPC教師を捏造しません。次のtargetは同じNPC worldで、
既存ML steeringへLiDAR gap residualを加えた教師候補を走らせます。

```bash
make e2e-npc-gap-teacher
```

このmodeは教師収集専用です。run admission後のbagだけを、明示的な出所で抽出します。

4台runの停止前に、production spatial candidateとdiagnostic pre-contact
teacherの差を同一bag上で比較する場合は、次のoffline監査を使います。停止runだけでなく同じ
worldのclean domainも必ず指定します。`comparison_window_kind`は停止domainでは停止前10秒、
clean domainでは発進後60秒です。teacherとの差はground truthではなく、成功runでも同じ差が
出るなら学習targetへ使いません。

```bash
python3 audit_interaction_divergence.py \
  --candidate checkpoints/<spatial-run>/candidate.npy \
  --candidate-use-base-steering \
  --case d1=/output/<run>/d1/rosbag2_autoware/rosbag2_autoware_0.mcap \
  --case d2=/output/<run>/d2/rosbag2_autoware/rosbag2_autoware_0.mcap \
  --case d3=/output/<run>/d3/rosbag2_autoware/rosbag2_autoware_0.mcap \
  --case d4=/output/<run>/d4/rosbag2_autoware/rosbag2_autoware_0.mcap \
  --output /output/<run>/interaction-divergence.json
```

```bash
python3 extract_data_from_bag.py \
  --seq-dirs /output/<run>/d1/rosbag2_autoware \
  --outdir dataset/obstacle_v1 \
  --label-source lidar_gap_teacher
```

学習済みstudentが新しい状態分布で失敗した場合は、student commandを教師dataとして
再利用しません。接触suffixを除外し、同じgap teacherでpre-contact scanを再labelします。
既定ではteacherがactiveな補正sampleだけをtrain splitへ保存します。

教師自身が同じworldを完走できた場合は、その合格run全体も別sequenceとしてtrainへ追加
します。失敗直前の局所補正だけでなく、回避開始後に安全な走行状態へ戻る分布を学習する
ためです。既知seedの再現合格だけでは昇格せず、未見seedを必須gateとします。

```bash
python3 relabel_gap_teacher_bag.py /output/<failed-run>/d1/rosbag2_autoware \
  --checkpoint checkpoints/<student-run>/candidate.npy \
  --outdir dataset/<dagger-name>
```

run-level admissionを通った`precontact_teacher`を使う場合は、旧teacherのprovenanceを
流用せず明示的に選択する。

```bash
python3 relabel_gap_teacher_bag.py /output/<admitted-run>/d1/rosbag2_autoware \
  --checkpoint checkpoints/<student-run>/candidate.npy \
  --teacher-mode precontact_teacher \
  --competition-analysis /output/<admitted-run>/e2e-competition-analysis.json \
  --require-executed-success \
  --novel-policy-only \
  --outdir dataset/<dagger-name>
```

車速でside commitmentを変えるstateful teacherは、合格runの全scanを時刻順に再生する。
`--active-only`や`--novel-policy-only`で途中状態を間引くとteacher stateを再現できないため、
時間モデル用sourceには使わない。LiDARより未来の車速を使わない
`latest_preceding`同期を行い、50 msを超えるscanが1件でもあればrun全体を拒否する。

```bash
python3 relabel_gap_teacher_bag.py /output/<admitted-run>/d1/rosbag2_autoware \
  --checkpoint checkpoints/<student-run>/candidate.npy \
  --teacher-mode speed_committed_teacher \
  --competition-analysis /output/<admitted-run>/e2e-competition-analysis.json \
  --require-executed-success \
  --no-active-only \
  --outdir dataset/<speed-committed-name>
```

`--require-executed-success`は、同じrun/domainで実際にteacherがauthorityを持ち、Finish・
penalty 0・stall 0だったことと、checkpoint/result/motion artifact hashを再検証する。
証明はdataset metadataとsequence identityへ含まれる。速度同期したrecurrent派生をhard labelへ
使う場合も`build_recurrent_dataset.py --require-executed-success`を指定し、証明を失ったsourceを
拒否する。`speed_committed_teacher`由来ではraw datasetに保存された因果速度列をそのまま
継承し、bagを別方式で再同期しない。派生label sourceも
`lidar_speed_committed_teacher_recurrent_direct`として旧teacherと区別する。

独立runを評価専用の時系列corpusへ変換する場合は、train splitを捏造したり既存train
rootを併合せず、`--split val`を明示する。既定では従来どおりtrain/valの両方を要求する。

```bash
python3 build_recurrent_dataset.py \
  --source-root dataset/<validation-only-raw> \
  --output-root dataset/<validation-only-recurrent> \
  --split val \
  --max-speed-sync-delta-sec 0.1 \
  --require-executed-success
```

`--novel-policy-only`は同一scan・同一base steeringに対する一世代前のreference teacherとの差が
0.02 rad以上あるlabelだけを残す。新teacherに固有でない通常のgap追従を大量に再学習して
既存の車線維持を退行させないための診断・stateless dataset admissionであり、閾値はmetadataへ
記録する。stateful recurrent sourceには使用しない。

`speed_committed_teacher`が失敗した場合、`required_stop_distance_m`をそのまま制動閾値へ
昇格させない。合格runにも同じ条件が存在するため、成功bagと失敗bagを同時に逐次再生して、
瞬時gapと完全な退避軌道を混同していないかを次で監査する。この出力は診断専用で、label抽出や
runtime authorityを許可しない。

```bash
python3 audit_speed_committed_escape_contract.py \
  --case success=/output/<successful-run>/d3/rosbag2_autoware \
  --case failure=/output/<failed-run>/d3/rosbag2_autoware \
  --failed-label failure \
  --failure-start-sec <stall-start-sec> \
  --checkpoint /path/to/tinylidarnet_weights.npy \
  --output /output/<failed-run>/teacher-escape-contract-audit.json
```

瞬時gapの方向が車体寸法上も実行不能かを切り分ける場合は、同じbag群を
`audit_swept_maneuver_certificate.py`で監査する。これは現在scanをbase linkへ戻し、
shift/counter-shiftと完全停止suffixをkinematic bicycleでrolloutして、全時刻の矩形車体
clearanceを検査する。成功bagにも同一条件を適用し、失敗bagだけを識別できなければlabel生成へ
進まない。

```bash
python3 audit_swept_maneuver_certificate.py \
  --case success=/output/<successful-run>/d3/rosbag2_autoware \
  --case failure=/output/<failed-run>/d3/rosbag2_autoware \
  --failed-label failure \
  --failure-start-sec <stall-start-sec> \
  --checkpoint /path/to/tinylidarnet_weights.npy \
  --output /output/<failed-run>/swept-maneuver-certificate-audit.json
```

この証明はcurrent scanを静止点群として扱うため、動くpeer、遮蔽された壁、未来のscanを
証明しない。`selected_infeasible_opposite_feasible`が失敗run固有でない場合、閾値や操舵offsetを
調整せず、時系列の動的占有を持つteacherへ進む。

時間情報そのものに識別力があるかを調べる場合だけ、`audit_future_occupancy_maneuver.py`を
privileged offline oracleとして使う。各rollout時刻へ記録済みの未来LiDARとodometryを合わせ、
現在base frameへ変換して同じ候補bankを検査する。未来入力はruntimeへ使用できず、記録された
実走経路からのscanは反実仮想候補の視点でもない。また、成功runの半数以上を同じ候補bankが
表現できなければ、失敗runとの率差があっても`inconclusive-candidate-bank-misses-success`として
label生成を拒否する。

```bash
python3 audit_future_occupancy_maneuver.py \
  --case success=/output/<successful-run>/d3/rosbag2_autoware \
  --case failure=/output/<failed-run>/d3/rosbag2_autoware \
  --failed-label failure \
  --failure-start-sec <stall-start-sec> \
  --checkpoint /path/to/tinylidarnet_weights.npy \
  --output /output/<failed-run>/future-occupancy-maneuver-audit.json
```

提出候補をfreezeするときは、単車合格と混走合格を同じ意味に扱わない。
`audit_e2e_submission_readiness.py`へinstall/source artifact、production runtime契約、
単車competition/spatial report、混走motion/competition/spatial report、privileged oracleを
渡すと、`reject`、`single-vehicle-candidate-only`、`multi-vehicle-candidate`のいずれかを
出力する。reportの`status=pass`だけは信用せず、analyzerへ渡した期待runtime、各Domainで
観測したruntime、raw/spatial SHA、Spatial coverage/error/stale/authority適用を再照合する。
`--require-multivehicle`を付けた場合、single-onlyはexit 3でfail-closedとなる。

```bash
python3 audit_e2e_submission_readiness.py \
  --raw-checkpoint /install/path/tinylidarnet_weights.npy \
  --source-raw-checkpoint /source/path/tinylidarnet_weights.npy \
  --expected-raw-sha256 <sha256> \
  --spatial-adapter /install/path/spatial_steering_adapter.npy \
  --source-spatial-adapter /source/path/spatial_steering_adapter.npy \
  --expected-spatial-sha256 <sha256> \
  --expected-control-mode fixed_lidar_brake \
  --expected-runtime-raw-checkpoint-path /install/path/tinylidarnet_weights.npy \
  --expected-acceleration-mps2 0.8 \
  --expected-maximum-forward-speed-mps 4.6 \
  --expected-residual-checkpoint-path '' \
  --expected-runtime-spatial-checkpoint-path /install/path/spatial_steering_adapter.npy \
  --expected-spatial-use-base-steering true \
  --expected-spatial-authority-enabled true \
  --expected-spatial-max-abs-delta-rad 1.2 \
  --expected-spatial-authority-max-abs-delta-rad 1.2 \
  --expected-recurrent-checkpoint-path '' \
  --expected-recurrent-authority-enabled false \
  --single-competition /output/<single>/e2e-competition-analysis.json \
  --single-spatial /output/<single>/e2e-spatial-shadow-analysis.json \
  --peer-motion /output/<peer>/d3/e2e-run-analysis.json \
  --peer-competition /output/<peer>/e2e-competition-analysis.json \
  --peer-spatial /output/<peer>/e2e-spatial-shadow-analysis.json \
  --peer-domain 3 \
  --future-oracle /output/<peer>/future-occupancy-maneuver-audit.json \
  --output /output/<peer>/e2e-submission-readiness.json
```

### Frozen-base steering residual

production TinyLidarNetを変更せず、`precontact_teacher - frozen base`だけを学ぶ場合は
`train_residual.py`を使う。各sequenceには`base_steers.npy`、`reference_steers.npy`、
`steering_deltas.npy`とtarget identityを保存する。長い成功runのsample数だけで学習が支配
されないよう、既定の`--sequence-balanced-sampling`は各runへ等しいsampling massを与える。
material correctionの重みとzero/small anchorはその上で両方保持する。

```bash
python3 train_residual.py \
  --train-dir dataset/<residual-name>/train \
  --val-dir dataset/<residual-name>/val \
  --output-root checkpoints/<residual-run> \
  --init-checkpoint checkpoints/<previous-residual>/candidate.npy

python3 evaluate_residual.py \
  --dataset-dir dataset/<residual-name>/val \
  --checkpoint checkpoints/<residual-run>/candidate.npy \
  --normal-dataset-dir dataset/<normal-name>/val \
  --output checkpoints/<residual-run>/gate.json \
  --fail-on-gate
```

evaluatorは合成後residualだけでなく、ungated correctionとgate probabilityも出力する。
これにより「gateが閉じた」のか「補正headが状態を区別できない」のかを分離する。runtime A/Bは
次のように明示する。未指定時はresidual modelを生成せず、production base出力は変わらない。

```bash
make e2e-single \
  TINY_LIDAR_RESIDUAL_CKPT_PATH=/aichallenge/ml_workspace/tiny_lidar_net/checkpoints/<residual-run>/candidate.npy
```

単車、NPC、4台の順に全gateを通るまでこのpathを既定設定や提出物へ設定しない。
