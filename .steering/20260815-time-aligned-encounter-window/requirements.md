# Requirements

## 目的

`output/20260815-170645`で多発したtarget-bound失敗を減らす。
現行のlive horizonは最大20 mを評価する一方、V2X相手予測は1.0秒後で
打ち止めになり、それより後ろの全sampleで同じ予測終端を再利用している。

上位参加者ログの`meet_window_m`と同様に、各経路sampleへ自車が到達する
時刻で相手位置を評価し、実際に出会うsampleだけへ相手制約を適用する。

## 制約

- 壁制約は緩和しない。
- 車体実寸の非重複制約は維持する。
- 前回追加したtarget-bound execution prefixは維持する。
- 予測可能時間を越えた遠方sampleはhard target boundにしない。receding
  horizonの次周期で、予測窓へ入った時点から制約する。
- `aichallenge_system`、ROS topic/service契約、result schemaは変更しない。

## Definition of Done

- 1秒より先のsampleで相手縦位置が等速外挿される。
- 設定した最大予測時間より先のsampleでtarget boundが非活性になる。
- 現行の1秒以内の予測結果とcandidate speed補正は維持される。
- 単体テストと`make autoware-build`が成功する。
