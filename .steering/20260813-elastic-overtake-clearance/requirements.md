# Requirements

## Background

`output/20260813-193949`ではMPCC-liteの同側Mission差し替えは動作したが、追い越し12開始に対して正常完遂は2回だった。Recovery 10回は次へ集中した。

- `optimized horizon escaped target separation bounds`: 8回
- `physical target separation conflicts with wall bounds`: 2回

現行のreceding-horizon処理には次の不整合がある。

1. target境界の初期生成では、target離隔を1.75→1.55→1.45 mへ緩和しても、壁側はロバスト計画余裕約0.40 mのままである。
2. optimizer後のstatic-map／横加速度補正では、初期採用したロバストtarget離隔を再び実行hard境界として検証する。
3. その結果、物理車体1.45 mとhard壁余裕0.20 mでは成立する経路までRecoveryへ落ちる。

## Goal

- 追加のtarget／wallロバスト余裕をpreferred制約として扱う。
- 実行を許可する最低境界は、物理target中心間隔と設定済みhard壁余裕から構成する。
- preferred境界が成立しないだけではRecoveryへ移行しない。
- 物理target境界とhard壁境界が本当に両立しない場合は従来どおりfail closedとする。

## Constraints

- 車体実寸、hard壁余裕、実壁接触、EmergencyBrakeを緩和しない。
- V2X、topic、launch、評価インターフェースを変更しない。
- 従来挙動へ戻せる設定スイッチを持たせる。
- `config.yaml`と`config_for_cloud.yaml`を一致させる。

## Acceptance criteria

- preferred壁境界で不成立でもhard壁境界で物理target離隔が成立すればtrajectory生成を継続する。
- post-validationは初期のロバストtarget離隔ではなく物理target離隔をhard境界として判定する。
- preferred余裕を使えなかったことを1 Hz debugで識別できる。
- 対象packageがbuildでき、既存および追加単体テストが通る。

