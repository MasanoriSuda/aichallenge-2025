# Requirements

## 目的

追い越し中に相手車両の予測位置が変化した際、DP候補を生成した後で棄却するだけでなく、相手車体との物理離隔を生成制約へ取り込み、同じ側で実行可能な代替経路を生成する。

## 対象事象

- rolling candidate が壁・運動制約を満たしても `target_bound=0` で昇格できない。
- 現行DP経路が相手予測に対して不成立になった後も古いprefixを保存し、新しい候補も不成立になる。
- 初回ShiftOut／直接Passとrolling refreshで、相手車体境界の検証時点が揃っていない。

## 制約

- ROS 2 topic、message、service、launch契約を変更しない。
- Recovery／MPC solver fallbackは今回の対象外とする。
- 物理離隔はhard constraint、追加のロバスト離隔はsoft preferenceとして扱う。
- 不成立な新候補は現行の最後の実行可能経路を破棄しない。

## 完了条件

- 時刻同期した相手予測をDP corridor生成へ適用できる純粋関数と単体テストがある。
- 初回・直接Pass・rolling refreshが同じ相手境界を含むDP候補を使用する。
- target-bound replan hold中は、不成立になった古いprefixを継がず現在姿勢から再接続する。
- package buildと対象単体テストが成功する。
