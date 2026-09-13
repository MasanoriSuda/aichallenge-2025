# Recovery preference live validation and full-course coverage

2026-09-13, controller7ecd0b36/build142/package129.120hostsec diagnostic.

7ecd0b36のsingle-r29で車両Start、Rejoin送信5件・復帰完了1回、通常2741送信を確認。
実期限外送信ゼロ、送信前拒否22件からの通常送信なし。4500callback中2件超過、連続なし。
2267は28.585ms（normal join16.687/Recovery10.407ms）で送信前拒否、2286は26.109ms
（Recovery21.738ms、全体CPU14.764ms）。後者の処理内/off-CPU詳細は未確定。
Recoveryの実選択は前進5件だけで、修正分岐の後退実行は未観測。Startを修正効果と断定しない。
壁区間拒否・方向不明の新記録はなし。周回・全体M4–M6は未完。
次は同じ制御コードの全コースsingle-r30（6周/600sim秒、660host秒上限）で未到達範囲を確認。
過去r26/r80の実送信期限問題は未修復で、同じ元のガードと失敗時の中止を維持する。

Source, installed binaries and original DLL unchanged; protected artifacts restored,
runtime stopped. No epoch, physical margin, solver, authority or clock relaxation.
First post-motion failure capture can be followed by later valid normal supply.
Only Forward selection occurred; do not call this actual acceptance of the new
reverse branch. Existing source/native positives remain separate from live evidence.

Revised first-interval observer retirement: no rejection occurred in r29. Retain
only through the newly justified full-course r30 coverage after Start, then review
capture/regression or removal. Six-lap acceptance requires actual same-run results
and full timing/authority evidence. Monitor10hostsec polling is not instantaneous
stopping; preserve every command through teardown. No approval pause.
[Evidence](recovery-preference-live-evidence.json).
