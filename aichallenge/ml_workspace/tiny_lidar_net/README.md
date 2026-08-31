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
- `label_source`が`mpc`、`mpcc`、`human`、`lidar_gap_teacher`のいずれか
- 全配列の長さ、finite値、timestamp順序
- train/validation間のsequence ID非重複

古いmetadataなしdatasetは既定では読み込まれません。上記extractorで再抽出してください。

`train.loss.accel_weight`を`0.0`にすることで、steeringのみ学習できます。現baselineは
longitudinalを固定加速度としているため、まずsteeringのみの学習を推奨します。
`train.pretrained_path`には`.pth/.pt`のraw state dictだけでなく、提出runtimeで使う
`.npy/.npz`weightも指定できます。全parameterのkey、shape、finite値を検証してから
warm-startします。各学習runはtimestamp directoryへ保存され、dataset sequence ID、設定、
pretrained checkpointのSHA-256を`training-manifest.json`へ記録します。

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

単車・NPC試走後は、正の加速度指令を出しながら実車速がほぼ0へ固着していないかをbag
から確認します。これは接触や壁拘束を「センサ停止」と誤認しないための最低限の自動gate
です。Finish、接触、周回時間の判定を置き換えるものではありません。

```bash
docker compose run --rm --no-deps autoware-command \
  python3 /aichallenge/ml_workspace/tiny_lidar_net/analyze_e2e_run.py \
  /output/<run>/d1/rosbag2_autoware \
  --output /output/<run>/d1/e2e-run-analysis.json \
  --fail-on-stall
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
