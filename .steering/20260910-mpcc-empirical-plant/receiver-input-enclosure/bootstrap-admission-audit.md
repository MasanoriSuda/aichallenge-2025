# Exact bootstrap admission rejection, two independent gates

2026-09-13, original live control7ecd0b36; replay after observer-only deletion.
R846 uses the saved501/856 original solver source, full wall/model/nominal proof,
packet programme, raw observation and complete Emergency transaction. It grants
a fresh diagnostic generation lifetime only to reconstruct the original source;
actual generation availability is separately read from the immutable capture.
Relative ledger sequence numbers are reconstructed; all packet/epoch/history
values and the input fingerprint match exactly.

壁区間observerを撤去し、実記録1263の境界回帰を残した。制御本文は観測ブロック等を除き完全一致。
R844 native103/source118、build143全26package、package130全2660tests合格。
R846で予約501/856の元の証明・指紋・全履歴を再生。双方で期限を約5ms超過し、未宣言の
実停止指令1件によりActualPrefixMismatch。856は加えて世代も失効。正しい拒否であり緩和しない。
初期候補が「計算中の追加指令なし」を前提に作られる一方、実系は停止を送る点が不整合。
次は元の物理・停止・履歴・期限条件を保つ候補生成の比較。停止中の将来入力は既存の通常認証済み
予約と同一視せず、新たな完全証明と実履歴の照合が必要。実装への昇格は比較・回帰の後。
現行削除後のliveは未実施。期限問題・通常移行・全コース/M4–M6は未完。

501: source history499 packets, nominal0.414999991/deadline0.439999991,
current0.444999990.856: source history24 packets,
nominal10.409999767/deadline10.434999767/current10.439999766.
Before the additional transaction, the original prefix is Consistent(0).
After the actual Emergency transaction, the complete current history matches
but the original declared prefix returns ActualPrefixMismatch(4), matching the
live record. Original no-reservation admission also requires no ledger changes;
the actual1transaction independently fails that condition.856additionally has
source_context_active_at_capture=false. No original current physical request
was made or saved, so none is fabricated by the replay.

Recorded worker33.428505/29.853700ms includes optional StartingDomain
10.630556/9.668469ms. Replay source evaluation19.317283/18.880353ms is
offline and overlaps package testing; it is not a live performance bound.
Precise worker completion/collection times were not captured. Moving optional
work, earlier notification or future composite inputs are hypotheses that each
must preserve exact history and original25ms windows; simply increasing lead,
grace or dropping intermediate Emergency records is not an accepted repair.

Bounded next comparison uses the same sealed world/source and original common
program evaluator. Contrast zero-prior construction with explicitly declared
actual Emergency prefix(es), only after proving the new entire prefix/suffix
population through rest. Retain generation, current-world, clock and exact
actual-source/wire/history obligations. This is distinct from SourceReservation,
whose pending packets already have a normal source certificate and frozen IDs.
Unattempted/unsupported architecture arms remain Unknown/inconclusive.
No production schedule or normal authority change yet.
[Evidence](interval-observer-retirement-validation.json).
