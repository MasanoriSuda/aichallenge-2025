# Requirements

## 目的

`20260813-163640` で増加した `ShiftOut/Pass -> Recovery` を抑え、追い越しの前進性を戻す。

## 観測した回帰

- receding-horizon 導入後、Pass 遷移と Return 完遂が減少した。
- `target-side separation conflicts with wall/trust bounds` が一律に物理壁不成立へ変換され、Recovery が増えた。
- authority を持たない MPCC-lite shadow 評価中は制御周期の遅延が増えた。

## 制約

- 実車体が壁または相手車両と物理的に両立しない経路は許可しない。
- optimizer failure、V2X 不連続、EmergencyBrake を無条件に無視しない。
- ROS 2 topic、message、launch、提出インターフェースを変更しない。

## 完了条件

- robust 車間だけが不成立なら、nominal、実寸の順に制約を縮退して Pass を継続できる。
- 実寸車間でも壁内に収まらない場合だけ hard infeasible とする。
- hard infeasible の実理由が Recovery 遷移ログへ残る。
- shadow 評価を診断用途の 1 Hz に下げる。
- 対象 package の build/test が成功する。
