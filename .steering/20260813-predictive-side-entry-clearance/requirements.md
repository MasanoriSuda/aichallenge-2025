# Requirements

## 目的

2026-08-13 の試走で確認した、明らかに余裕の小さい側を選んで接触へ進む追い越しを減らす。

## 対象事象

- 左右とも Mission が成立すると、相手との将来車間より rear-clear 時間や入口の横移動量が優先される。
- 相手が選択側へ横移動中でも、固定の body-clear deadline margin で 3～4 m から仕掛ける。
- 接触直前まで frozen Mission を維持し、反対側の再評価が間に合わないケースがある。

## 要求

1. ShiftOut 完了から rear-clear までの予測最小対車 surface clearance を候補ごとに保持する。
2. 壁余裕と対車余裕の小さい方を interaction reserve として比較し、明確に広い側を優先する。
3. 相手が選択側へ横移動しているときだけ body-clear deadline margin を増やし、仕掛け距離を実質的に前倒しする。
4. 反対側への切替は current body が非重複かつ no-return 前という既存 hard guard を維持する。
5. 停止車両や相手が選択側から離れている場合は、固定距離の一律な入口禁止を追加しない。
6. ROS topic/service/message と評価インターフェースを変更しない。

## 完了条件

- core 単体テストで将来対車余裕が大きい候補が選ばれる。
- core 単体テストで選択側への横速度だけが deadline margin を増やす。
- 対象 package がビルドできる。
- 実走評価はユーザー側の `make dev2` で行える状態にする。
