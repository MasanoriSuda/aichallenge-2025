# Requirements

## 目的

2026-08-20 の試走で頻発した `static wall execution preflight: invalid local path target`
を、通常の動的障害物回避経路に対する誤棄却として修正する。

## 背景

- 通常の GapPlanner 経路は `target_ey` と `target_active` に実行経路を持つ。
- 低速停止車用経路だけが持つ `pass_side_sign` / `pass_target_ey` を共通 wall
  preflight が必須としており、通常経路が有効でも棄却されていた。
- 現行ログは `invalid local path target` までしか示さず、入力契約のどの要素が不正か、
  また何サンプルを検証したかを判別できない。

## 要件

- preflight の有効性は `active`、`feasible`、`target_ey`、`target_active` から判定する。
- `pass_side_sign` / `pass_target_ey` は低速車専用メタデータとして、共通契約の必須条件にしない。
- activity mask が空の場合は全 target sample を有効として既存の低速車経路を維持する。
- activity mask のサイズ不整合、active sample 不在、active sample の非有限値を理由別に棄却する。
- 通常の疎な動的回避経路は、最後の active sample と短い末尾余裕までを検査する。
- preflight の実行 sample 数、active sample 数、active 範囲、異常 index を決定ログへ残す。
- ROS topic/service、Domain、result JSON、提出物契約、設定値は変更しない。
- ユーザー所有の走行結果・クラッシュ成果物は変更・コミットしない。

## Definition of Done

- 通常経路が `pass_side_sign == 0` だけを理由に棄却されない。
- 不正な target/mask は単体テストで理由別に fail closed する。
- 決定ログだけで preflight の入力範囲と棄却位置を判別できる。
- 対象 package の単体テストとビルドが成功する。
