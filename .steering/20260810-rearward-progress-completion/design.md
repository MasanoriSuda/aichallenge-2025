# Design

`PassShortHorizonGuard` に `rearward_completion_progress_active` を追加する。

次の全条件を満たす場合、予測重複確認の0.25秒を超えてもrear-clear完遂を継続する。

1. `SideBySideCommitted`
2. forward completion latch済み
3. 対象車が自車位置以下（`target_s <= 0`）
4. 現在車体が非重複
5. execution corridorが非block
6. 実測前進進捗がfresh
7. 通常のhard guardが正常

これは予測を無視する一般的な猶予ではない。対象が後方へ移動し続けている完遂局面だけ、実測進捗を予測重複より優先する。進捗停止時は既存のfreshness判定で即座に無効化される。SafeSeparationのlocal/absolute budget、Return条件は既存のままとする。

