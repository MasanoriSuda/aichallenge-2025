# 連続外まくりPass corridor

## 目的

ヘアピンが連続するコースで、外まくりとして開始した固定Frenet sideが次の逆向き
カーブでは内側となり、低速車に塞がれてPassが失速・Recoveryへ移る事象を減らす。

## 要求

- `inner/outer`を現在地点だけで判定せず、前方の曲率をローリングホライズンで確認する。
- committed outer Pass中、次の有意なカーブで外側が反転する場合は、反転区間が一定距離
  継続するときだけ外側目標の更新を要求する。
- 外側目標の切替は、locked targetが十分前方にあり、現在車体が非重複で、壁・V2X・
  solverの短期hard guardが正常な場合だけ許可する。
- 横並び中または相手が近い場合は、反対側へ横断しない。
- 切替先はtarget separation、static wall、横加速度、rear-clear rolloutを再検証してから
  atomic commitする。
- 切替開始時はfront-capを再適用し、横断中に相手へ追突しないようclosingを制限する。
- 曲率ノイズによる左右チャタリングを、最小曲線継続距離、cooldown、最大回数で抑止する。
- actual wall contact、現在車体重複、EmergencyBrake、target discontinuity、絶対Pass上限は
  緩和しない。
- ROS 2 topic/service/message契約は変更しない。

## 完了条件

- 前方曲率列から外側切替要求を決める純粋関数を単体試験できる。
- Pass continuationで検証済みの反対側goalをcommitできる。
- 横並び・前方距離不足では切替を開始しない。
- package test、build、`git diff --check`が成功する。

