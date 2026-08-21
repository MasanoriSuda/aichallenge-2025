# Requirements

## 目的

DynamicEscape の候補経路が RViz 上で山状に生成・消失を繰り返し、候補の再評価中に制御が減速保持へ落ちる問題を修正する。

## 必須要件

- 物理壁判定を通過した同一周期の DynamicEscape 解を、不要な1周期待ちなしで採用する。
- 新候補の再評価中は、直前に物理壁判定を通過した実行列を時刻に合わせて保持する。
- 保持実行列の先頭指令を繰り返し再生せず、経過時間に対応する stage を使う。
- 保持実行列を使っている間、その horizon を平坦な速度・操舵列で上書きしない。
- 保持解の採用元、経過時間、stage、残り horizon、新候補の昇格結果をログで追跡できる。
- 壁接触、壁余裕不足、out-of-map を安全解として採用しない。

## 制約

- ROS 2 topic、message、launch、提出インターフェースを変更しない。
- `output/` と既存の `aichallenge/result-summary.json` を変更しない。
- DynamicEscape 以外の通常 MPC、正式 Overtake Mission、Recovery の所有権は変更しない。
