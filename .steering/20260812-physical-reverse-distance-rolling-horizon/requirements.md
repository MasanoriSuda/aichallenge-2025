# Requirements

## 目的

`20260812-194821` の試走で、4.0 m Reverse設定にもかかわらずP1が実移動
2.002 mでescape完了扱いになった。またP2は4/8 m全体rolloutが不成立になると、
0.4 mのstepwise ReverseとDriveを39回ずつ切り替え、復帰に約410秒を要した。

停止予測距離と実移動距離の責務を分離し、短い安全区間しか見えない場合も
Reverse gearを維持したrolling-horizonで物理目標距離まで後退できるようにする。

## 要件

- 4.0/8.0 mのescape完了判定には実移動距離だけを使う。
- 停止予測距離はReverse内のブレーキ開始判定だけに使う。
- ブレーキ後に実移動が目標未満なら、gearを変えずReverseを再開する。
- full-distance rolloutが不成立でも、0.4 mのswept-footprintが成立する間は
  Reverse gearを維持してrolling再計画する。
- rolling区間ごとにReverse primitiveと操舵を再選択する。
- rolling中もstatic contact、V2X、course-worsening、最大距離・時間のguardを維持する。
- simulation-onlyかつ明示設定時だけrolling stepwiseを有効化する。
- 評価基盤、ROS topic/service、result JSON契約を変更しない。

## 対象外

- Stuck detectorの発火条件変更
- Overtake plannerの変更
- Recovery MPCの全面置換
- 最大Reverse速度・加速度の変更
