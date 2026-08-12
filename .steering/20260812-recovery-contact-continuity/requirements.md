# Requirements

## 目的

`20260812-203808` の試走では、物理4/8 m Reverseとrolling-horizonは機能したが、
接触が多いD2で一過性の`contact_worsened`または0.4 m再計画直後の
`new_contact`によりReverseを中止し、Drive/Reverseを8回ずつ要求して復帰に
56.48秒を要した。またD1では同一Recovery incidentが約300秒保持された。

正常復帰後の履歴を一車身程度の通常前進で解除し、rolling Reverse中は接触が
継続して悪化した場合だけ方向変更する。

## 要件

- Rejoin後2.0 mの正常前進でadaptive retryとincident ledgerを初期化する。
- 2.0 m未満で再スタックした場合は同一incidentとして4 mから8 mへ拡張する。
- rolling再計画でprimitiveを更新しても、接触パッチの改善追跡を継続する。
- rolling Reverse中の`new_contact`/`contact_worsened`は0.20秒連続した場合だけ
  Supervisorへ悪化として通知する。
- pose jump、無効グリッド、out-of-map等のhard faultは遅延させない。
- rolling Reverse以外の既存接触判定は変更しない。
- 評価基盤、ROS topic/service、result JSON契約を変更しない。

## 対象外

- Reverse速度・加速度・最大距離の変更
- Stuck detector発火条件の変更
- Overtake plannerの変更
- 永続的な壁ピンを無視する変更
