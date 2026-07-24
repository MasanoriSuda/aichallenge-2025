# 追い越し残事象修正 結果

## 結論

急失速は一つの縦制御問題ではなく、二つの独立事象だった。

1. ShiftOut中の最初の減速は、bounded corridor holdと2.0 m/s no-gap limitの
   重複適用であり、hold所有権をno-gap判定へ渡すことで解消した。
2. 約7.19 m/sから約1.0 m/sへの急失速は、停止車の誤判定で残留した
   LowSpeed direct controlが大操舵を出し、壁へ衝突したことが主因だった。

3 distinct valid V2X sample確認を導入した`20260724-085142`では、確認countは
最大1/3に留まり、LowSpeed direct誤起動、壁接触、contact、stuckは再発せず、
lap 1を70.566 sで完走した。したがって、既知の急失速連鎖に対する効果は確認できた。

一方、追い越しの`Pass -> Return -> Idle`完遂と、真の停止車を対象にしたdirect
controlの動的安全確認は未完了である。今回の結果を「追い越し全体が完成した」
とは扱わない。

## 修正前後比較

### corridor hold

| 指標 | 修正前 `070818` | 修正後 `073134` |
|---|---:|---:|
| hold中の`acceleration=-1.35 m/s²` | 70/70周期 | 最長holdで8/40周期 |
| command speed | 6.424→3.661 m/s | 6.477→6.466 m/s |
| actual speed | 6.470→3.634 m/s | 6.550→6.245 m/s |
| 2.0 m/s方向への崩落 | 発生 | 解消 |

全hold区間でも最低command speedは5.877 m/s、最低actual speedは5.809 m/sであり、
修正前のno-gap hard limit由来の急減速は再現しなかった。

### 壁衝突急失速

`20260724-083221`では次の順序を確認した。

- LowSpeedAvoidance direct controlが右target約-2.38 mで開始
- behaviorがFollowへ戻った後、OvertakeLineが左target約+3.28 mでShiftOut開始
- 古いdirect controlが最終出力を所有
- actual speed 7.191 m/s、raw steering -0.445 rad、最終出力-0.667 rad
- OvertakeLineはraw footprintでRecoveryへ移行したが、direct出力は継続
- 約0.079 s後に壁接触し、contactは303まで増加
- 1.003 sで7.191→0.999 m/s、sample平均加速度は約-6.17 m/s²

`d2`自身のログでは速度は約5.288→5.497 m/sだった。`d1`側trackerが速度差分を
作れない初回・同時刻観測を0 m/sとして扱ったことが、停止車誤判定の入口だった。

### 誤起動対策後

`20260724-085142`では次を確認した。

- 初期の停止車候補は1 distinct sampleだけで、確認は最大1/3
- 移動観測によりcountは0へreset
- LowSpeedAvoidance stateとdirect control entryなし
- `wall=1`、`current_contacts>0`、stuck recovery実行なし
- 以前の衝突箇所を約8.1〜8.3 m/sで通過
- lap 1を70.566 sで完走

この走行にはrosbagがなく、上記はAutoware/AWSIMログによる確認である。

## PassとRecoveryの観測

`085142`ではShiftOutからPassへ2回移行した。

1. `ShiftOut -> Pass`後、約0.5 sでlocked target stale/lostによりRecovery
2. `ShiftOut -> Pass`後、static wall clamp後のmax lateral acceleration超過によりRecovery

どちらも危険な横targetを継続しないfail-closed動作だが、
`Pass -> Return -> Idle`の正常完遂は確認できていない。

また、壁接触のない安全Recovery中にactual speedが4.051→1.389 m/sへ
1.019 sで低下し、sample平均は約-2.612 m/s²だった。これは`083221`の
壁衝突による約-6.17 m/s²とは別事象である。Recovery時のタイヤ運動・操舵中立化・
縦速度指令をrosbagで分離できていないため、現時点では原因確定や解消を主張しない。

## 静的検証

dedicated confirmation gap、behavior速度cap、最終publish操舵cap、
direct raw footprint wall stopまで含む最終差分をDocker環境で確認した。

- `docker compose run -T --rm --no-deps autoware-build`: 25 package成功
- 最終レビュー修正後の対象package再build: 成功
- 対象package test: 23/23 test target成功
- `colcon test-result`: 611 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功
- topic/service/message、Domain、launch、評価schema: 差分なし
- buildのstderrはsetuptools deprecation warningのみ

## 未検証・残課題

1. 真の停止車が存在するシナリオで、3 distinct sample後にdirect controlが開始し、
   操舵・速度capとraw footprint wall stopが期待どおり働くこと。現guardは現在poseの
   footprintと0.72 m marginに対する反応型で、凍結targetの将来rolloutは検証しないため、
   relevant vehicleが残る場合に安全停止latchとなり得る。
2. stale/lostにならない対象追跡で、`Pass -> Return -> Idle`まで完遂すること。
3. contactを伴わないRecovery中の約-2.612 m/s²減速をrosbagで、
   command、actual velocity、steering、yaw rate、タイヤ/接触状態に分解すること。
次に動的確認を追加する場合は、広いparameter tuningではなく、真の停止車direct経路と
Recovery減速の二つにシナリオを限定する。
