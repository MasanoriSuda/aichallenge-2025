# Requirements

## 目的

停止・低速車向け `LowSpeedDirect` が前方から横並びへ移る境界で対象車を見失い、
Pass 中に反対側へ切り返す、または静的壁だけを根拠に古い経路を継続する問題を防ぐ。

## 必須要件

- `LowSpeedDirectControlPhase::Pass` へ入った後は、同一 Mission 内で pass side を変更しない。
- `LowSpeedDirect` 開始時の対象車 ID を Mission 終了まで保持する。
- 前方専用 local planner が非 active になった後の retained Pass は、次を全て満たす場合だけ許可する。
  - 保持した対象車を継続観測できる。
  - position jump がない。
  - 現在の車体 footprint が非重複である。
  - 短時間予測 footprint sweep が非重複である。
  - 対象車が保持した pass side と反対側に留まる。
  - path bounds と静的壁 preflight が成立する。
- Shift 中は、現在側が実行不能なら従来どおり完全preflight済みの反対側候補を選べる。
- ROS topic/service、Domain、提出インターフェースは変更しない。

## 制約

- 現HEADのRecovery引継ぎと低速blocker早期Overtake開始を維持する。
- `aichallenge/result-summary.json` の既存変更は触らない。
- 実走による効果確認はユーザー側で行う。

## Definition of Done

- Pass 中に planner が反対側を返してもsideを保持する単体テストがある。
- retained Pass のtarget identity・現在footprint・予測sweep・side orderingを検証する単体テストがある。
- 対象packageがビルドでき、対象単体テストが成功する。

