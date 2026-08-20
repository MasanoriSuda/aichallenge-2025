# Requirements

## 背景

`output/20260820-105930/d1/autoware.log` の1周半試走では、速度窓と壁余裕契約の
矛盾は解消した一方、次の実行契約不具合が残った。

- 動的障害物回避中の単発OSQP失敗が毎回 `-3.0 m/s2` の強制停止になる。
- DynamicMissionWaitへ遷移した最初の1周期だけ横経路authorityが消える。
- same-side Mission置換では、候補のtarget/wall/freshness契約が共通に再確認されない。

## 目的

1. 安全な動的回避中の短いsolver失敗を、非加速のbounded continuationへ縮退する。
2. DynamicMissionWaitへの遷移と横authorityの発行を同じ制御周期で完了する。
3. 全てのruntime Mission置換でtarget clearance、wall clearance、予測鮮度を同じ契約で確認する。
4. 最終制御ログから、solver解、bounded continuation、crawl、forced stopを区別できるようにする。

## 制約

- 実車向けのfail-operational動作には広げない。`use_sim_time` の競技シミュレーションに限定する。
- emergency、壁接触、footprint不明、追従誤差超過、連続solver失敗では従来の停止を維持する。
- ROS 2 topic、message、launch、評価schemaは変更しない。
- `output/`、rosbag、ユーザーの既存変更は編集しない。
