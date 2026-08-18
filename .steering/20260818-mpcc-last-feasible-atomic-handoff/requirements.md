# Requirements

## 目的

target-wall の将来制約が一時的に不成立になったとき、直前の物理検証済み MPCC 解を
即座に捨てて FollowPrepare へ落ちる挙動を減らす。

## 対象

- ShiftOut / Pass の target-bound failure
- 直前に解けた MPCC 実行軌道の再利用
- 新しい feasible horizon への atomic handoff

## 制約

- actual wall contact、wall margin violation、wall sample unavailable は保持しない。
- EmergencyBrake、solver Recovery、target jump / course rejection は保持しない。
- 通常の body overlap は保持しない。既存の recoverable side contact だけ例外とする。
- 直前解は現在の壁 footprint と相手予測を毎周期再検証する。
- 直前解が使えない場合は既存の current-lateral freeze へ戻す。

## Definition of Done

- target-bound failure 時に、再検証済み直前解を bounded prefix として選択できる。
- 新解が安定するまで Mission generation と実行 authority を保持する。
- hard fault では従来どおり即座に保持を解除する。
- 関連 unit test と `make autoware-build` が成功する。
