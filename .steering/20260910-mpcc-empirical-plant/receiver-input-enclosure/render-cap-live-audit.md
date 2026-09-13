# Graphical200FPS diagnostic and reservation admission gap

2026-09-13, HEAD e8ec1e2c/control7ecd0b36, build142/package129.
Supported --target-fps 200 was the only startup intervention. Original DLL,
controller, sensors, RViz, physics/source clock and every guard were retained.

描画を保つ200FPS診断single-r31で、時計受信間隔は中央値5.004ms・最大13.948ms。
標準r30の中央値0.600ms・最大294.599msとの差はフレーム上限の関与を支持するが、将来の上限ではない。
4217callback最大23.941ms、通常送信0件。車両Start/Rejoin/周回は未達で、期限問題の修復とは判定しない。
初期501とactive856は証明worker33.429/29.854ms、採用時刻が予約期限を約5ms超過。
同時に履歴も更新済みなので、期限だけを唯一原因と断定せず、元の入力・世代・履歴を再生する。
壁区間2163とRecovery4164は元の判定に完全一致。余裕込み壁接触とコース悪化を正しく拒否。
次は役目を終えた壁区間observerを削除して1263の境界回帰を残し、予約生成・採用の時間と履歴を監査。
無変更の走行を反復せず、25ms期限・物理条件・正常系の認証を維持。全コース/M4–M6は未完。

Clock source19820samples all exactly match original float32 5ms cadence.
Recorder receipt gaps: r30 n44829, median0.599786ms, p9916.264542ms,
max294.599374ms, below1ms29656, above20ms5; r31 n19819,
median5.004037ms, p996.287006ms, max13.948227ms, below1ms77,
above20ms0. This supports the original60FPS cap as a contributor to burst
delivery. It does not certify a maximum future ROS-clock advance during a send.
Recovery selected24maneuvers; public maximum absolute velocity is reverse
1.385065675m/s. Normal publication, vehicleStart and Rejoin were not reached.

Original501: planning0.394999991, nominal0.414999991, deadline0.439999991,
current0.444999990; worker33.428505ms includes starting-domain10.630556ms.
Original856: planning10.389999767, nominal10.409999767,
deadline10.434999767, current10.439999766; worker29.853700ms includes
starting-domain9.668469ms. Both saved requests have an additional actual
Emergency ledger transaction. The snapshot records reservation-invalid and
preserves source/generation/history; no current physical query was attempted.
Do not call these CurrentPhysicalProof failures or attribute exclusively to clock.

R841 reproduces all147 interval samples at2163 exactly. The expanded original
preferred footprint already hits occupied556811; no physical body contact is
required to violate the original normal clearance. R842 reproduces both4164
Recovery trials: ForwardRight lookahead0.8m and ForwardStraight0.480073592m
are physically clear but worsen lateral alignment by0.054055595m and
0.037065038m. These are correct course rejections of the actually tried paths;
untested paths are Unknown, not proven physically impossible.

Next: remove the bounded current-wall interval diagnostic (its exact questions
are answered), retain permanent1263 guard regression, then reproduce the two
reservation admission timelines against original ledger/context/epoch. Separate
source work, observation/poll scheduling, immutable appointment, and intervening
Emergency transactions before a producer repair. No unchanged failed-live repeat.
Runtime stopped, protected artifacts restored. [Evidence](render-cap-live-evidence.json).
