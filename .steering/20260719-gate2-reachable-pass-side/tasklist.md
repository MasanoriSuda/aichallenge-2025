# Tasklist

- [x] Gate2ログから初期yawと局所経路を分離して診断する
- [x] 到達距離優先のside選択をpure coreへ追加する
- [x] side選択の単体テストを追加する
- [x] shift targetからpass hard corridorへの状態依存遷移を追加する
- [x] shift中だけの速度上限を追加する
- [x] 低速回避専用の最低gap幅を有効にする
- [x] 壁marginを0.2 mへ緩和して評価し、左回廊は成立しないため0.8 mへ戻す
- [x] 左右候補幅を診断し、左側に物理回廊がないことを確認する
- [x] 低速横・heading feedbackによる直接操舵を追加する
- [x] 車列clearance追跡と2秒の解除ヒステリシスを追加する
- [x] package build/testを実行する
- [x] `make gate2`でPASSを確認する
