# Committed Pass ポリシー・リファクタリング要件

## 目的

通常 Overtake の挙動を変えずに、Pass 中の速度 cap 解除・維持・再適用と
committed Pass 速度 floor の判断を純粋関数へ集約する。

実走で確認した「物理横離隔を得た後に execution horizon 制約で front cap が再適用される」
現象を、次段階で副作用やログ処理を追わずに変更・検証できる状態にする。

## 対象

- `multi_purpose_mpc_ros` の通常 Overtake
- ShiftOut / Pass 中の front cap ポリシー
- constrained horizon での release / hold 判定
- committed Pass 速度 floor の適用判定
- front cap 状態変化の診断理由
- 上記判断の単体テスト

## 対象外

- 設定値の変更
- front cap の解除・再適用条件そのものの変更
- 壁、横加速度、Recovery の発動条件変更
- pass side、corridor、Persistent Mission の方針変更
- ROS 2 topic、message、launch、評価インターフェース変更
- 実走による性能評価

## 挙動不変条件

1. validated start-grid breakout の速度所有権を変更しない。
2. front cap の release / hold / reapply 条件を変更しない。
3. committed Pass 速度 floor の適用条件を変更しない。
4. front cap 状態変化のログ理由文字列を変更しない。
5. 速度 reference、hard limit、floor の合成順序を変更しない。
6. `config.yaml` を変更しない。

## Definition of Done

- Committed Pass の速度ポリシーが ROS や controller state に依存しない純粋関数になる。
- controller は純粋関数の resolution を適用する構造になる。
- 現行の release / hold / reapply / speed floor / 診断理由を回帰テストで固定する。
- `make autoware-build` が成功する。
- `multi_purpose_mpc_ros` のテストが成功する。

