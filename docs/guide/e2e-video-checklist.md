# E2E予選 走行動画チェックリスト

## 推奨シナリオ

packaged productionのNPC seed 2037を主動画にする。production既定値を使い、教師mode、
recurrent authority、custom checkpointは指定しない。

```bash
make e2e-npc-single \
  LOG_DIR=/output/e2e-preliminary-video-seed2037 \
  E2E_START_RANDOM_SEED=2037
```

録画後はAWSIMがresult JSONを書き終えてから停止し、次を実行する。

```bash
make autoware-bash
cd /aichallenge/ml_workspace/tiny_lidar_net
python3 analyze_e2e_run.py \
  /output/e2e-preliminary-video-seed2037/d1/rosbag2_autoware \
  --output /output/e2e-preliminary-video-seed2037/d1/e2e-run-analysis.json \
  --fail-on-stall

python3 analyze_e2e_competition.py \
  /output/e2e-preliminary-video-seed2037 \
  --expected-control-mode fixed_lidar_brake \
  --expected-acceleration-mps2 0.8 \
  --expected-maximum-forward-speed-mps 4.6 \
  --expected-checkpoint-path \
    /aichallenge/workspace/install/tiny_lidar_net_controller/share/tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy \
  --checkpoint-file \
    /aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy \
  --expected-checkpoint-sha256 \
    de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa \
  --expected-residual-checkpoint-path '' \
  --expected-spatial-checkpoint-path \
    /aichallenge/workspace/install/tiny_lidar_net_controller/share/tiny_lidar_net_controller/ckpt/spatial_steering_adapter.npy \
  --spatial-checkpoint-file \
    /aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller/ckpt/spatial_steering_adapter.npy \
  --expected-spatial-checkpoint-sha256 \
    f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c \
  --expected-spatial-use-base-steering true \
  --expected-spatial-authority-enabled true \
  --expected-spatial-authority-max-abs-delta-rad 1.2 \
  --expected-recurrent-checkpoint-path '' \
  --expected-recurrent-authority-enabled false \
  --output /output/e2e-preliminary-video-seed2037/e2e-competition-analysis.json \
  --fail-on-rejection
```

## 採用条件

- [ ] 3/3周をFinishした
- [ ] 1位である
- [ ] crash / wall / over penaltyが0
- [ ] post-start low-speedとpositive-acceleration stallが0秒
- [ ] runtime control modeが`fixed_lidar_brake`
- [ ] accelerationが`0.8 m/s²`、speed capが`4.6 m/s`
- [ ] raw/spatial artifact SHAがfreezeと一致
- [ ] recurrent authority、teacher mode、custom artifactが無効
- [ ] NPCへの接近、横回避、抜けた後の復帰が画面で確認できる
- [ ] LiDAR制動を説明する場合は、実際の作動と速度・距離をログで確認した
- [ ] HUD、車両、コースが読める解像度で録画できた

一項目でも不合格なら、見栄えのよい区間だけを切り出して合格動画として扱わない。

## 編集構成案

1. 5秒: タイトル、入力（2D LiDAR + wheel speed）、ML lateralを表示
2. 15〜30秒: 通常の車線維持
3. 20〜40秒: NPCへ接近して回避する場面
4. 10秒: 3周Finish、1位、penalty 0の結果画面
5. 5秒: モデル構成と残課題（controllable peer）

## 使用しない映像

- `speed_committed_teacher`、`precontact_teacher`など教師専用mode
- recurrent authorityのstrict runtime Gate不合格run
- MPC/MPCCの混走run
- penalty、stall、result未確定run
- seedやruntime identityをログで確認できないrun

これらは失敗分析・開発過程としてスライドへ載せられるが、production E2Eの成功動画とは
明確に分ける。

前方`1.5 m`の制動閾値は停止距離保証ではなく、現行productionの距離ベース安全
ヒューリスティックである。制動場面を資料へ載せる場合は「安全停止を保証」と表現せず、
そのrunで観測した進入速度、最短距離、停止または回避結果を併記する。
