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
  --expected-checkpoint-path \
    /aichallenge/workspace/install/tiny_lidar_net_controller/share/tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy \
  --checkpoint-file checkpoints/20260901_055824/candidate.npy \
  --expected-checkpoint-sha256 \
    de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa \
  --output /output/<run>/e2e-competition-analysis.json \
  --fail-on-rejection
```

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

### Runtime NPC corrective teacher

runtime NPCはV2Xへ現れないため、MPC教師を捏造しません。次のtargetは同じNPC worldで、
既存ML steeringへLiDAR gap residualを加えた教師候補を走らせます。

```bash
make e2e-npc-gap-teacher
```

このmodeは教師収集専用です。run admission後のbagだけを、明示的な出所で抽出します。

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
  --novel-policy-only \
  --outdir dataset/<dagger-name>
```

`--novel-policy-only`は同一scan・同一base steeringに対する旧`LidarGapTeacher`との差が
0.02 rad以上あるlabelだけを残す。新teacherに固有でない通常のgap追従を大量に再学習して
既存の車線維持を退行させないためのdataset admissionであり、閾値はmetadataへ記録する。

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
