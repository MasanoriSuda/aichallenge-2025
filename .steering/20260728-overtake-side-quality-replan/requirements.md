# Requirements

## 目的

追い越し開始時の内側優先と、ShiftOut中の固定side継続によって、
相手が選択側へ移動し反対側が開いた場合にも同じ側へ進み続ける現象を解消する。

## 根拠

`output/20260728-000047/d1/autoware.log`では次を確認した。

- WP54で左右ともgap成立にもかかわらず`inner hard curve entry`で`side=-1`を選択
- commit後は反対側が`not evaluated`となり、locked sideだけを継続評価
- WP170のShiftOutではtarget相対横位置が`-0.93 m`から`+0.85 m`へ変化
- WP184で左右ともgap成立へ戻っても`side=-1`と固定goalを維持
- 最後は`wall=front`、MPC 8回連続失敗、`solver failure threshold reached`

起動ログでは`course_lateral_velocity=0`であり、相手横移動の生予測を
そのまま有効化するA/Bは過去に不採用としている。

## 変更範囲

- 新規追い越しの左右品質比較
- ShiftOut中の選択側競合検出
- ShiftOut前半の安定確認付きside再計画
- 後半または反対側不成立時のRecovery
- pure helper、単体テスト、設定、MPC暫定仕様

## 制約

- 左右の瞬間的な変化でsideを往復させない。
- Pass移行後はsideを直接反転しない。
- ShiftOut後半は相手を横切る反転を行わずRecoveryへ移す。
- actual/static wall、横加速度、solver、Emergencyのhard guardを維持する。
- `v2x_prediction_use_course_lateral_velocity`は今回有効化しない。
- ROS topic/service、Domain、評価成果物の契約を変更しない。
- 速度、壁margin、車間距離の既存設定値を変更しない。

## 完了条件

- 新規開始時に、内側という理由だけで低品質側を選ばない。
- ShiftOut前半で相手が選択側へ移動し、反対側が優位な状態が一定時間続けば
  現在位置から安全に反対側へ再計画する。
- ShiftOut後半または反対側不成立時は直接反転しない。
- 対象packageのビルドと単体テストが成功する。
- 実走効果はユーザーの`make dev2`で確認する。
