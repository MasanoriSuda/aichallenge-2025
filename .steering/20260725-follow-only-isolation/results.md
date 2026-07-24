# 5 m追従単独分離実験 結果

## 結論

追い越しとRecoveryを完全に抑止しても、5 m境界で速度指令の急変と実速度低下が再現した。
したがって、急失速はOvertakeLine Recoveryだけの問題ではなく、Follow速度制限にも
独立した高優先度の原因がある。

現行は`v2x_follow_speed_limit_distance`と
`v2x_moving_follow_target_distance`がともに5.0 mである。5 mをわずかに超えると
Follow capが無効になって11.111 m/s指令へ戻り、5 m以内へ入ると先行車速度に基づく
約2.26〜4.45 m/sの上限が即時に掛かる。この境界に進入・離脱ヒステリシスがないため、
指令が大きく往復している。

## A/B条件

| 条件 | run | d2上限 | Overtake / Recovery |
|---|---|---:|---:|
| 現行 | `20260724-235653` | 16 km/h | 24 / 57回 |
| 追従単独 | `20260725-001842` | 16 km/h | 0 / 0回 |

追従単独runでは、Follow FSM、V2X追跡、Follow距離・速度、安全制約を維持し、
実験中だけstart-grid breakoutを無効化し、追い越し必要通路幅を100 mとして
Overtake成立を抑止した。

## 周回

| 車両 | 現行 Lap 1 / 2 | 追従単独 Lap 1 / 2 |
|---|---:|---:|
| d1 | 75.193 / 74.948 s | 75.093 / 74.809 s |
| d2 | 73.694 / 75.023 s | 73.694 / 74.868 s |

周回時間はほぼ同じだが、これはd1がd2の16 km/h走行に拘束されているためであり、
追従の滑らかさを示すものではない。

## 5 m境界

制御ログは約1 Hzのため、次の切替回数は実際の下限値である。

| 指標 | 現行 | 追従単独 |
|---|---:|---:|
| Follow前方車debug sample | 73 | 149 |
| `follow_cap=1` sample | 20 | 41 |
| capの0/1切替 | 13 | 45 |
| capの0→1切替 | 4 | 22 |
| cap有効時の`fd`平均 | 4.249 m | 4.660 m |
| cap有効時の`fd`範囲 | 3.040〜4.980 m | 3.060〜5.000 m |
| cap有効時の速度上限範囲 | 2.260〜4.380 m/s | 2.260〜4.450 m/s |

追従単独runの22回のcap有効化のうち、10回で前後2秒の指令速度が5 m/s以上低下し、
7回で実速度が1 m/s以上低下した。

代表例:

- `fd=4.87 m`
- command: 11.111 -> 3.802 m/s
- actual: 4.816 -> 3.804 m/s
- 最小実加速度: -1.456 m/s2
- d2実速度: 4.413 m/s

全Follow区間の最大値:

- command: 11.111 -> 2.367 m/s、0.5秒で-8.744 m/s
- actual: 5.160 -> 3.736 m/s、1秒で-1.424 m/s
- cap有効化前後の最小実加速度: -1.792 m/s2

追い越しとRecoveryが0回のため、これらはFollow固有の速度制限で発生している。

## 追従安定性

追従単独runの`final=Follow`かつ前方車ありのdebugでは、`fd`は次の範囲だった。

- 平均: 5.583 m
- 標準偏差: 1.012 m
- 最小 / 最大: 3.060 / 8.900 m
- d1実速度がd2より0.5 m/s以上遅い時間: 16.102 s

5 m一定へ収束しているのではなく、capの有効・無効をまたいで接近と離脱を繰り返している。

## ログ・MCAP健全性

- Overtake遷移: 0回
- OvertakeLine Recovery: 0回
- SafetyStop / runtime contact / active Reverse: 0回
- d1 MCAP: 219.019 s、78,123 messages、16,758,830 bytes
- d2 MCAP: 229.772 s、81,963 messages、17,581,249 bytes
- d1 V2X: 16.671 Hz、最大gap 0.072 s

通信欠損やRecovery混入では説明できない。

## 原因判定

最優先課題はFollow速度制限の入口不連続である。

```text
fd > 5 m
  Follow capなし
  commandが最大11.111 m/s方向へ戻る

fd <= 5 m
  moving-front Follow capを即時適用
  commandが約2.26〜4.45 m/sへ低下
```

次の最小実験は、`v2x_moving_follow_target_distance=5.0 m`を維持したまま、
`v2x_follow_speed_limit_distance`だけを7.0 mへ広げるA/Bである。目標距離の手前から
速度を連続的に合わせ、5 mちょうどでの大きな指令段差を減らせるか確認する。

本実験のOvertake抑止設定は診断専用であり、測定後に現行値へ戻し、
再ビルドしてinstall側にも現行設定を反映した。

## 実行した確認

- `git diff --check`: 成功
- `make autoware-build`: 診断設定・復元設定とも25 packages成功
- `make dev2`: d1/d2とも2周完了
- Autowareログと5トピックMCAPをレース開始から2周完了まで時刻同期
