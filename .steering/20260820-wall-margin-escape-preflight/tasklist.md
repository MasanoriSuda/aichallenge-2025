# Tasklist

- [x] 最新ログの wall preflight 棄却位置を確認する
- [x] 実寸 footprint と余裕込み footprint の契約を分離する
- [x] margin escape の純粋判定を実装する
- [x] controller の中心方向 admission と決定ログを実装する
- [x] 通常 clear / 実寸衝突 / 改善成功 / 改善失敗のテストを追加する
- [x] 対象 package をビルド・テストする
- [x] ユーザー所有の走行生成物を除外してコミットする

## Definition of Done

- 現在姿勢で余裕だけ不足している中心方向 candidate が限定的に採用される。
- 実寸衝突、短距離内に余裕復帰しない candidate、余裕復帰後の再接触は棄却される。
- decision trace だけで採否理由を特定できる。
- 既存および追加単体テストが通る。
