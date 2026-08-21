# 要求

## 背景

`output/20260822-072612` では、追い越し Mission 自体は開始できたが、
ShiftOut 開始直後に縦速度の所有権が追い越し系から通常レーシングラインへ戻った。

- decision 8239: `lateral_owner=overtake-line`、`longitudinal_owner=overtake-line`、
  `speed=5.93/inf/0`、実速度 9.31 m/s
- decision 8240: phase は ShiftOut、lateral owner も overtake-line のままだが、
  `longitudinal_owner=racing-line`、`speed=inf/inf/0`
- 約 0.52 秒後、実行軌道が `solution swept wall path collision at path index 2`
  で棄却された

候補生成ログの `timing_v` は約 6.1 m/s であり、その速度で成立確認した横経路を
実行中に 9 m/s 超へ戻すことは、候補採用時と実行時の契約不整合である。

## 目的

1. ShiftOut の横経路を実行している間は、その経路を検証した速度契約を維持する。
2. 前車速度 cap の解除と、横経路の実行速度契約を独立させる。
3. 将来同じ不整合が再発した場合、決定ログだけで判別できるようにする。

## 制約

- ROS 2 topic / service / message 型、Domain、提出インターフェースは変更しない。
- 壁余裕や車間の設定値を変更して症状を隠さない。
- Pass で横移動を終えた後の加速方針は変更しない。
- SafetyBrake、solver fallback、hard velocity limit は常に優先する。
- `aichallenge/result-summary.json` の既存変更には触れない。

## 完了条件

- 通常 Mission 候補が検証時の ego 速度を保持できる。
- frozen Mission の ShiftOut 中は finite な速度 reference が毎周期存在する。
- front cap が release 済みでも ShiftOut の速度契約が消えない。
- ShiftOut の横 authority があるのに速度契約がない状態を conflict として記録する。
- 単体テスト、対象 package のビルドとテストが通る。
