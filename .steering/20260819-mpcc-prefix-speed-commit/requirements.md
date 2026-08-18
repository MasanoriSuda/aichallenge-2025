# Requirements

## 目的

非同期 tactical worker が生成・検証した追い越し候補を live callback へ採用する際、
計画時と採用時で異なる速度サンプルを直接比較して候補を反復棄却する不整合を解消する。

## 必須要件

- 候補生成時に、その rollout が満たすべき最低 ego 速度を候補へ保存する。
- 非同期結果の採用では、候補の予測最低速度を計画時の要求値に対して検証する。
- 計画時要求値を持たない legacy 候補は、従来どおり live 要求値へ fail-closed する。
- 浮動小数点・rollout 離散化による微小差だけで棄却しない、設定可能な速度許容値を設ける。
- wall、target、no-return、残時間・残距離、result freshness の hard gateは緩和しない。
- progressive prefix と complete cross-side Mission で同じ最低速度採用契約を使う。
- ROS 2 topic、service、launch、評価インターフェースを変更しない。
- `aichallenge/result-summary.json` の既存変更へ触れない。

## 検証

- 計画時要求値と live 要求値が非同期遅延中にずれたケースの単体テスト。
- 計画時点ですでに最低速度不足の候補が引き続き拒否されるテスト。
- 計画時要求値がない候補が live 要求値へ fail-closed するテスト。
- `multi_purpose_mpc_ros` のテストとビルド。
