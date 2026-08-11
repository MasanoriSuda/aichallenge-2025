# Design

## 現状

`FollowPrepare` の処理は、期限切れが関数前半、target異常とrear-clearが後半、
再開処理が `behavior_overtake` 分岐に分かれている。また一時停止前の phase は
保存されないため、`Pass` と `ShiftOut` のどちらを中断したかを後段で判断できない。

## 変更方針

### 1. pause originの保存

`OvertakeLineState` に `follow_prepare_origin_phase` を追加する。
`transition_overtake_line_phase()` が `FollowPrepare` へ入る瞬間に遷移元を保存し、
`FollowPrepare` を離れる時に初期化する。今回の再開先選択にはまだ利用せず、
次の性能修正に必要な事実だけを失わないようにする。

### 2. terminal actionのpure化

pure coreに以下を追加する。

- action: `Hold` / `Return` / `Recovery` / `Expire`
- reason: time/distance、position jump、course-progress discontinuity、stale、
  forbidden waypoint、rear-clear
- request: 現在のpause状態、各観測、期限と走行距離

既存挙動を維持するため優先順位は次の通りとする。

1. time expiry
2. distance expiry
3. position jump
4. course-progress discontinuity
5. forbidden waypoint
6. stale/lost
7. rear-clear
8. hold

controllerは前半で一度だけ解決結果を作り、既存どおりexpiryだけを即時終了する。
後半の`FollowPrepare`分岐は同じ解決結果を使ってRecovery/Return/Holdを行う。

### 3. 対象外

今回の変更ではpauseからの即時Resume、反対側再選択、速度cap所有権は変えない。
これらは実走ログで基準を確定してから別ステアリングで扱う。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`
- `mpc_controller_cpp.cpp`
- `test_v2x_overtake_core.cpp`

launch、yaml、評価インターフェースには影響しない。
