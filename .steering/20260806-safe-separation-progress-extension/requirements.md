# Requirements

## 目的

SafeSeparation の前方完遂中に、対象へ継続して接近できているにもかかわらず固定距離上限で Recovery へ落ち、追従相当まで失速する事象を減らす。

## 対象事象

- `20260806-094158/d1` で、対象前方距離が約 2.61 m から約 0.59 m へ縮小した。
- 車体非重複、forward escape 有効、速度参照約 5.2--5.4 m/s の状態だった。
- SafeSeparation 開始後の走行距離が約 12 m に達した時点で、理由を区別できない `safety/timeout bound reached` により Recovery へ遷移した。

## 必須要件

1. 対象へ一定量以上接近し、その進捗が直近まで継続している場合だけ SafeSeparation の局所時間・距離枠を再設定できる。
2. 再設定回数は有限とし、Pass 全体の絶対時間・絶対距離上限を越えない。
3. 車体重複、壁不成立、EmergencyBrake、対象ジャンプ、solver recovery など、既存の短期安全条件が不成立なら即座に Recovery とする。
4. 後方クリアと Return corridor が成立した場合は、上限判定より先に Return する。
5. Recovery 理由を少なくとも、短期安全不成立、局所時間上限、局所距離上限、絶対時間上限、絶対距離上限、入力異常に分離する。
6. 既存の ROS 2 topic、service、message、提出物契約を変更しない。

## 非対象

- 壁余裕、車体寸法、SafetyBrake 閾値の緩和
- Pass corridor の再設計
- 実車向け設定

