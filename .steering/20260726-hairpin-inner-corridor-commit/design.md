# Design

## 問題構造

現行は新規curve entryで通常の追い越し完了距離を満たせない場合、近距離かつ測定済み速度優位が
ある場合だけoverrideする。ヘアピン前では、内側corridorが有効な時点と速度優位が成立する時点が
ずれるため、内側候補をコミットできない。

## 方針

内側curve entry専用の `inner curve precommit` 判定を追加する。

precommitは以下をすべて満たす場合だけ許可する。

- 機能が有効
- 内側curve entry/hard entryが既存判定で許可済み
- まだ明示的なOvertakeLineをコミットしていない
- 前方車を正常に認識
- 緊急制動状態ではない
- 前方距離が通常entryの最小値以上かつcurve entry上限以内
- 内側の連続開放距離が既存の最小値以上
- 自車と先行車の相対速度が設定下限以上

通常の完了距離判定が真の場合は従来どおり進入する。偽の場合だけprecommitを追加許可として使う。
進入後は既存のlocked-side、transient gap hold、hard-curve continuationへ引き継ぐ。

## 設定

- `v2x_overtake_inner_curve_precommit_enabled`
- `v2x_overtake_inner_curve_precommit_min_relative_speed`

相対速度下限は負値を許容する。競技シミュレーション設定では、最新ログの `-0.22 m/s` を許容しつつ、
先行車が明確に離れていく場合を除外するため `-0.5 m/s` とする。

## 影響範囲

- ROS topic/service/message契約: 変更なし
- 評価JSON: 変更なし
- 外側追い越し: 変更なし
- 実車設定: 本機能は現行のsimulation competition用configでのみ有効化

## ログで期待する変化

変更前:

```text
wp59 ... pass=-1 inner_pass=1 ... reason=curve entry lacks measured gain/near-target range
wp71 Follow -> Overtake ... outer hard curve entry
```

変更後:

```text
wp59 Follow -> Overtake ... inner hard curve entry
OvertakeLine: Idle -> ShiftOut, side=-1
```

