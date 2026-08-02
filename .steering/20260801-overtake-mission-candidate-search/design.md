# Design

## 現行課題

現行は1つの横目標と設定値のShiftOut距離だけを評価し、さらに
ShiftOut/Pass/Return全区間を開始時に同時検証している。実際のReturn開始は
距離固定ではなくlocked targetのrear-clearで決まるため、開始時の仮想Returnが
実走FSMより厳しい棄却条件になっている。

## 方針

### 候補生成

設定ShiftOut距離を基準に、短い側から複数の距離候補を生成する。各距離について、
動的corridorから得られる横目標区間内の以下を候補とする。

- 現在ラインに最も近い点
- plannerの推奨点
- corridor中央
- 区間内の中間点

### 候補評価

各候補は次の順でhard gateを通す。

1. ShiftOut＋Pass予測区間の動的corridor
2. 車体footprintを含むstatic wall
3. 横加速度上限

成立候補は、direct pass、短いShiftOut、少ない横移動、低い横加速度の順に選ぶ。

### mission固定

選択した横目標とShiftOut距離をOvertakeLine stateへ保存する。ShiftOut中の
目標生成、完了判定、closing-speed残距離計算は同じ保存値を使う。

### Return

開始時の動的判定からReturnを外す。Passはrear-clearまで横位置を保持する。
rear-clear後は既存のreturn-corridor blockerとlive static-wall horizonでReturnを
判定するため、第三車両や壁の保護は維持する。

## 非対象

- 加速度上限1.0 m/s^2の変更
- gap幅・壁余裕の攻撃化
- Recovery FSMの再設計
- ROSインターフェース変更
