# Renderer diagnostic and first interval capture boundary

2026-09-13. Baseline b56ecdab / controller38a562ee, build140/package127.

single-r27の描画なし診断は通常738送信、4281callback最大24.392ms、実期限外送信ゼロ。
時計の記録側受信間隔中央値5.001ms・最大14.327ms（標準描画r26は中央値0.574ms・
最大296.983ms）。環境差への支持はあるが、経路・場面が異なり描画の原因確定ではない。
Recoveryは後退25/前進3選択、小操舵−0.05radの指令と実負速度を確認。Rejoin送信2回、
復帰完了・車両Start・周回はこのrunでは未達。壁区間拒否は初の通常移動記録より前に
現れ、既存の観測範囲から外れていた。last_physicalは過去値の保持で、連続拒否を意味しない。
次は先行通常移動を必須としない初回実クエリ観測へ置換し、原地図・元の判定を一致再生する。
標準描画の送信期限問題と全コース・M4–M6は未完。

R807–R811 preserve original DLL/source/binary hashes, all run inputs and public data.
Unity NullGfxDevice confirms only -batchmode -nographics; no physics, clock, rate,
margin or authority changes. This120hostsec diagnostic is not six-lap acceptance.
The first prior moving-normal witness is2534; first post-motion loss2545. Earlier
last_physical rejection history is ineligible for the old recorder, not proof of
a broken owner. Revise the diagnostic scope before another run, retaining the
one live ordinary owner, no worker/override, fixed32results and async immutable map.
No producer repair is justified until exact original query replay. Retire this
observer after supported regression/fix or review its removal if single-r28 has
no eligible query. [Evidence](renderer-clock-live-evidence.json).
