# Design

## 問題の境界

現行の `evaluate_overtake_line_entry_preflight()` は、次の用途で共用されている。

1. 初回 ShiftOut / Pass admission
2. 横目標を再計算する same-side geometric extension
3. 横目標を固定した longitudinal horizon refresh

1 と 2 では、新しい横経路を採用するため中心間最小離隔と外まくり戦略の整合を
hard constraint とするのが妥当である。一方 3 は既に採用・進入した固定横経路の
縦方向有効距離だけを更新する処理であり、曲線の内外ラベル変化や固定 1.5 m の
中心間離隔だけで中断すべきではない。

## Policy 分離

core に `PassContinuationPreflightPolicyRequest/Resolution` を追加する。

footprint continuation policy を有効にする条件は全て AND とする。

- longitudinal refresh である
- short-horizon safety が成立
- target observation が連続している
- current body footprints が非重複
- predicted body footprint sweep が利用可能かつ非重複

有効時のみ、継続 preflight で次を advisory にする。

- target center からの固定 1.5 m 離隔
- course curvature による outside / inside role の反転

上記以外では従来どおり hard constraint とする。

## 維持する hard constraint

- fixed lateral goal と pass side
- target ID、side、mission generation、prediction expiry
- current / predicted footprint 非重複
- target position jump、pass-side intrusion、EmergencyBrake の不存在
- kinematic rollout による rear-clear 成立性
- wall bounds、static map wall、実壁接触
- lateral acceleration、steering curvature
- Pass の 32 m / 10 s 絶対上限

## Controller 変更

`evaluate_overtake_line_entry_preflight()` に明示的な policy を渡す。

- 初回 admission: default policy（全 hard constraint）
- geometric extension: default policy（全 hard constraint）
- longitudinal refresh: core の policy resolver が許可した場合だけ footprint policy

kinematic rollout の `feasible` は円近似の中心間横離隔を使うため、footprint policy
有効時には要求しない。rear-clear 到達可能性は従来どおり rollout で必須とし、横の
物理的安全性は current / predicted footprint と full-path static preflight で担保する。

## ログ

longitudinal refresh 成功ログに `footprint_policy=1` と
`outer_role_reversal` を出し、曲率上の役割反転を観測可能にする。

## 実走評価

- `rear_clear_refresh` から SafeSeparation へ入る回数
- `outer pass becomes inside before rear-clear` の継続 refresh 拒否回数
- `target separation does not fit wall-feasible bounds` の継続 refresh 拒否回数
- `Pass -> Return` 完遂数
- wall / lateral acceleration / footprint overlap による拒否が維持されること
- 6 周の 80 秒超ラップ数と合計時間
