# First interval observation run and Recovery ordering failure

2026-09-13, controller7f263cc2/build141/package128.120hostsec diagnostic.

7f263cc2の標準描画single-r28は通常675送信、Rejoin送信3件・復帰完了1回、実期限外送信ゼロ。
4271callback中495の1件25.218ms（初期化14.746ms/Recovery6.402ms）、連続超過なし。
2771は送信前拒否。ゲームStart成立、車両Start・周回は未達。壁区間拒否は発生せず観測なし。
後半のRecovery2906をR818で完全再生。前進3候補は全て壁またはコース条件で拒否され、
「前進優先」の分岐が後退評価まで省く。これはordering-onlyという宣言と不整合。
R819は同じ地図・姿勢で既存後退4m/+0.15rad、8m/+0.10radの物理・コース検証に合格。
未評価の元の後退距離は未保存なので、これらは明示した距離での診断であり実行許可ではない。
次は優先指定が候補を消さない選択処理の回帰・修正・検証。先行する通常停止の原因、
標準描画の期限問題・全コース/M4–M6は未完。

Original snapshot2906/time66.274998518 has a clear current footprint and four
actual trials: forward-right0.8m lookahead collides, three forward0.6m candidates
all fail physical or course guards. No reverse trial occurred. All actual trial
results, full grid fingerprint and course checks reproduce exactly in R818.
R819 existing11reverse primitives select +0.15rad at3.9995/4/4.0005m and +0.10rad
at8m. Keep every original physical/course rejection and V2X/gear/revalidation.
No new fallback authority, parameter or primitive is justified. Repair ordering
semantics with a failing scene regression, then native/full-package/live checks.
The earlier normal-source loss remains separate. No physical infeasibility claim.

The first-interval observer did not fire because the query did not reject. Review
its retirement explicitly: retain through only the next structurally changed
Recovery-order scene, then review capture/regression or removal. The old r27
query remains unavailable; do not interpret cached history as fresh failures.
Source/binaries and protected artifacts verified; runtime stopped.
[Evidence](first-interval-live-evidence.json).
