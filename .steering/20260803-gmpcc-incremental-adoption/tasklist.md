# Tasklist

## Steering

- [x] 暫定1位log/MCAPを解析する
- [x] log2との差を整理する
- [x] 現行の時系列mission path実装との重複を確認する
- [x] requirements/design/tasklistを作成する

## Stage 0: current baseline

- [ ] `.steering/20260803-pass-horizon-safe-separation`の`make dev2`を6周実施する
- [ ] Pass/Return/Recovery/SafeSeparation統計を記録する
- [ ] target別の追い越し所要時間とPass最低速度を記録する
- [ ] baselineのHEAD、設定、AWSIM版、run pathを記録する

## Stage 1: horizon progress evaluation

- [ ] progress evaluationの入出力をpure struct/functionとして定義する
- [ ] predicted rear-clear time/distanceを候補ごとに計算する
- [ ] horizon最低速度と必要closingを計算する
- [ ] scoreとhard reject reasonを実装する
- [ ] left/right/base-line候補の単体テストを追加する
- [ ] target横揺れでcommit後sideが変わらないテストを追加する
- [ ] event logを追加する

## Stage 2: speed ownership

- [ ] mission generationへ速度ホライズンを保持する
- [ ] committed horizonとgeneric Follow capのownershipを整理する
- [ ] Emergency/別車両/wall/solverの優先順位をテストする
- [ ] Pass中の不要な負加速度を検出する診断を追加する

## Stage 3: dynamic opponent corridor

- [ ] target footprint sweepからside-specific corridorを生成する
- [ ] existing `lb/ub`へ時系列corridorを適用する
- [ ] 非凸・side不成立時はFollowへ戻しRecoveryを発生させない
- [ ] 複数車両とhairpinのテストを追加する

## Verification per stage

- [ ] `test_v2x_overtake_core`
- [ ] `git diff --check`
- [ ] `make autoware-build`
- [ ] `make dev2` 6周
- [ ] baselineとのmetric比較
- [ ] wall contact、停止、Reverse、55秒超過ラップの回帰確認

## Deferred experiments

- [ ] sim-only `a_max=1.0/1.37/2.8` A/B用steering
- [ ] heading-based multi-stroke recovery用steering
- [ ] full GMPCC prototypeのGo/No-Go判断

## Definition of Done for the next implementation turn

- Stage 0 baselineが揃っている。
- Stage 1だけを実装し、Stage 2以降を同じ差分へ混ぜていない。
- unit test/buildが成功している。
- `make dev2`でPass完遂率または追い越し所要時間が改善し、停止・壁接触が悪化していない。
