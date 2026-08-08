# Pass live-corridor phase hold requirements

## 背景

`output/20260808-213349`では、`ShiftOut`中にlive execution corridorの不成立が始まり、
同じhold期限を`Pass`へ持ち越した結果、`Pass`開始約1.45秒後に
`live overtake corridor unavailable`で`Recovery`へ遷移した。その後、壁際で停止して
Reverseへ入り、約11秒の復帰ロスと98.98秒ラップにつながった。

## 要件

- `ShiftOut`で消費したlive-corridor hold時間を、物理的に車体が非重複な`Pass`へ
  持ち越さない。
- `Pass`開始時刻と、その後に確認したlive corridorの最終有効時刻の新しい方を、
  `Pass`中のhold基準時刻にする。
- 車体が現在重複している場合は、`Pass`開始時刻による猶予を付与しない。
- target未観測、position jump、明示禁止WP、cooldown、EmergencyBrake、実壁接触、
  wall margin違反、solver異常は従来どおり緩和しない。
- 既存の`v2x_overtake_active_gap_loss_hold_sec`を再利用し、新しい調整パラメータは
  増やさない。
- ROS 2 topic、message、service、提出インターフェースは変更しない。

## 対象外

- recoverable contact continuationの判定変更
- 相手予測モデルや左右候補評価の変更
- Recovery/Reverseアルゴリズムの変更
- hold時間そのものの延長

