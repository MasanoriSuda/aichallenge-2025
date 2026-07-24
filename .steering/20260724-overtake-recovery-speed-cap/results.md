# 追い越しRecovery急減速調整 結果

## 結論

`v2x_overtake_recovery_velocity`の3.0から5.0 m/sへの変更は、Recovery移行時の
急失速を明確に緩和した。物理接触、SafetyStop、Stuck Recoveryの新規発生もないため、
急失速対策として5.0 m/sを暫定採用する。

ただし追い越し全体は未解決である。d1はShiftOutからPassへ進みやすくなった一方、
壁余裕・横加速度の安全制約でRecoveryへ戻り、正常な`Pass -> Return -> Idle`は
今回も0回だった。速度設定だけで追い越し完遂まで直ったとは評価しない。

## 検証条件

- ベースライン: `output/20260724-231447`
  - Recovery上限: 3.0 m/s
- 変更後: `output/20260724-234011`
  - Recovery上限: 5.0 m/s
- dev2、d1/d2、2周以上
- d1/d2とも次の5トピックをMCAPへ記録
  - `/control/command/control_cmd`
  - `/clock`
  - `/localization/acceleration`
  - `/localization/kinematic_state`
  - `/v2x/vehicle_positions`
- Recovery遷移時刻をAutowareログから取得し、開始後2秒のMCAPを比較

`output/20260724-233714`はdev2の自動カウント開始に不要なadmin startを送ろうとして
開始待ちタイムアウトになったため、測定対象から除外した。設定変更後の有効な測定は
`20260724-234011`である。

## 比較結果

| 指標 | 3.0 m/s | 5.0 m/s | 評価 |
|---|---:|---:|---|
| d1 Lap 1 | 64.369 s | 64.164 s | 0.205 s短縮 |
| d1 Lap 2 | 61.389 s | 60.864 s | 0.525 s短縮 |
| 2周までのRecovery | 10回 | 19回 | 増加 |
| bag全体のRecovery | 11回 | 21回 | 増加 |
| `ShiftOut -> Pass` | 1回 | 10回 | Pass到達は増加 |
| `Pass -> Return -> Idle`完了 | 0回 | 0回 | 未解決 |
| Recovery後2秒の最低実速度 | 2.609 m/s | 4.587 m/s | 1.978 m/s改善 |
| Recovery後2秒の最大速度低下 | 2.424 m/s | 1.890 m/s | 22.0%縮小 |
| Recovery後2秒の平均速度低下 | 1.569 m/s | 0.717 m/s | 54.3%縮小 |
| Recovery後2秒の最小実加速度 | -2.484 m/s2 | -2.061 m/s2 | 急減速を緩和 |
| 2周中にd1がd2より0.1 m/s超遅い時間 | 60.630 s | 59.501 s | 1.129 s短縮 |
| Recovery後2秒の平均最大車間増加 | 2.306 m | 1.684 m | 27.0%縮小 |
| runtime contact / SafetyStop / active Reverse | 0 / 0 / 0 | 0 / 0 / 0 | 退行なし |

車間はd1 odometryのmap座標と、d1側V2Xメッセージに含まれるd2位置のユークリッド距離
として算出した。コース曲率の影響を含むため、個別最大値ではなく同じ計算による平均値を
比較に使用した。

## MCAP健全性

| 車両 | 記録時間 | message数 | MCAPサイズ |
|---|---:|---:|---:|
| d1 | 153.595 s | 54,706 | 11,750,725 bytes |
| d2 | 162.271 s | 57,797 | 12,413,197 bytes |

d1のV2Xは変更前17.417 Hz、最大gap 0.071 s、変更後16.114 Hz、最大gap 0.072 s
だった。最大gapは同等で、今回の差を説明する通信途絶はない。

## 残事象

変更後はd1が3.0 m/sまで離脱せず対象付近に残るため、再試行とPass到達が増えた。
その結果、bag全体のRecovery理由は次のように変化した。

| Recovery理由 | 3.0 m/s | 5.0 m/s |
|---|---:|---:|
| actual footprint wall margin violated | 3 | 6 |
| static wall clamp exceeds lateral acceleration limit | 3 | 8 |
| static wall clearance margin infeasible | 0 | 5 |
| locked target no longer executable | 5 | 2 |

target喪失は減ったが、壁制約による中断が増えた。これは速度上限の効果不足ではなく、
現在の横経路では安全制約を満たしてPassを完了できないことを示す。

次に扱うなら、wall marginや横加速度を緩和するのではなく、失敗後cooldownと
pass side・横経路候補の再選択を別ステアリングで検討する。今回はソースロジック、
壁安全条件、5 m Follow境界には変更を加えていない。

## 実行した確認

- `git diff --check`: 成功
- `make autoware-build`: 25 packages成功
- `make dev2`: 変更後run `20260724-234011`でd1/d2とも2周完了
- MCAP比較: 5トピックのmessage数、Recovery前後2秒のcommand/actual/acceleration/
  V2X車間を確認

