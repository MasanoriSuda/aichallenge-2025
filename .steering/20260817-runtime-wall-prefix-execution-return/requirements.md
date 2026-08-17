# Requirements

## Background

`bf016aa`後の実走`output/20260817-103103`では、path-aware wall warningから
`runtime wall escape prefix accepted`へ初めて到達した。横目標は1.55 mから1.20 mへ補正され、
front-cap releaseも保持された。

しかし、採用直後のPass entry physical gateが現在横位置を保持し、DP prefix authorityも
残り1.91 mで通常のterminal distanceにより解放された。その後、警告開始から0.52秒時点では
残り1.12 mしかなく、横加速度制限を満たす再計画ができずMissionを終了した。

また、同走行では`Pass -> Return`後にReturnが約38秒残留した。Return preflight referenceを
消費した後の汎用横goalが、基準線0 mではなく凍結済みpass goalを再び選んでいた。

## Goal

- 採用済みwall-escape prefixを、残りの安全な経路を使い切るまで実行する。
- wall-escape prefix実行中は、同じwarningを理由にPass entry gateで現在位置へ凍結しない。
- prefix用terminal distanceを通常DP Missionから分離する。
- Return preflight reference終了後も基準線0 mへのReturnを継続し、Idleへ完了できるようにする。

## Constraints

- wall接触、hard margin、map unavailable、target overlap、横加速度制限は緩和しない。
- wall-escape prefixの採用には従来どおりwall・target・kinematic preflightを要求する。
- 通常DP Missionのterminal distanceとMission再計画cooldownは変更しない。
- ROS 2 topic/service、Domain、launch、result JSON schemaを変更しない。
- ユーザーの`steering_tire_angle_gain_var: 1.79`変更は保持し、今回のコミットに含めない。

## Definition of Done

- 採用済みwall-escape prefixに専用の短いterminal distanceを適用する。
- 実行authorityが有効なwall-escape prefixはPass entry gateより優先される。
- authority喪失またはhard fault時は既存gate/再探索へ戻る。
- Return/Recoveryの汎用横goalは必ず基準線0 mになる。
- core unit test、package test、buildが成功する。
