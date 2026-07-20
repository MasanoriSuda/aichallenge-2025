# Requirements

## 目的

dev3のヘアピンで外回りだけでなく、前走車の内側に連続した通過回廊がある場合に
OvertakeLineを使ったイン差しを成立させる。

## 要件

- soft curve区間では、曲率符号から求めた内側にgapが成立した場合だけ新規ShiftOutを許可する。
- イン差し機能が有効で両側gapが成立する場合は、未ロック時だけ内側を優先する。
- hard curve内から新規イン差しを開始しない。
- soft curveで開始済みの内側ShiftOut/Passは、locked targetと内側gapが継続する場合だけhard curveでも維持する。
- 内側gapまたはlocked targetを失った場合は既存Recoveryへ戻す。
- 明示WP禁止、curve cooldown、EmergencyBrake、SafetyBrake、wall clearanceは緩和しない。
- 外回り機能と直線追い越しを維持する。
- ROS 2 topic、service、message、Domain、評価インターフェースは変更しない。
- 設定省略時は従来互換の無効状態とする。

## 完了条件

- pure coreで内側soft-curve進入、外側拒否、hard-curve新規進入拒否、locked継続、安全guardを単体テストする。
- dev3設定でイン差しを有効化する。
- `make autoware-build`が成功する。
- dev3実走はユーザー側で行う。
