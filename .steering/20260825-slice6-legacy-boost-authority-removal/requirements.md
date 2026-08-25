# Requirements

## Objective

Slice 6 の単一 normal authority 化を妨げる、2025 由来の legacy boost 制御経路を物理削除する。

## Root-cause statement

`use_boost_acceleration` は単なる actuator option ではない。これを有効にすると canonical MPCC の
加速度指令を legacy 閾値ロジックで置き換え、`/boost_commander/command` へ別 message を publish
し、別 node が `/control/command/control_cmd` を所有する。checked-in の提出 launch が `false` に
固定していても executable と比較用 launch には到達可能な第二の normal command path が残る。

この二重 authority は canonical solution identity、Recovery、Emergency、2026 公式 `/awsim/cmd`
Boost の境界を曖昧にするため、flag を残して無効化するのではなく同じ Slice で削除する。

## Scope

- C++ controller の legacy boost parameter、加速度上書き、別 publisher を削除する。
- 比較用 Python controller／path constraint provider の同じ legacy switch を削除する。
- legacy `boost_commander` node、専用 message、launch argumentを削除する。
- 2026 公式の有限回 `/awsim/cmd` StartOnce Boost は維持する。
- canonical MPCC、Emergency、Stuck/gear/reverse Recovery は変更しない。

## Constraints

- parameter tuning、wall margin変更、solver設定変更を行わない。
- 新しい fallback、feature flag、timeout、lease を追加しない。
- `aichallenge/result-summary.json` はユーザー変更として触らない。

## Definition of Done

- source/config/launch/build定義から legacy boost authority のtokenとpublisherが消える。
- `/control/command/control_cmd` の normal producer は controller自身へ収束する。
- 2026公式 `/awsim/cmd` Boost と Recovery が維持される。
- build、package tests、source-contract tests が成功する。
