# Design

## 問題

現行はbody-clear可能な入口候補を`entry_setup_mission`として保持するが、これは縦速度の準備にしか使われない。実際のOvertakeLine開始にはShiftOut/Pass/rear-clear/Returnを含む全区間の成立が必要である。このall-or-nothing条件により、短時間予測しかない状況や曲率反転が多い区間では、見えている空きへ横移動せずFollowへ残る。

## 方針

候補を次の二層に分ける。

1. Complete Mission
   - 従来どおり全区間を検証済み。
   - 左右選択では最優先する。
2. Progressive Entry Mission
   - 初回entryに限定する。
   - local ShiftOut preflight、body-clear deadline、横加速度、wall boundsが成立している。
   - rear-clear/Returnの完全成立はentry blockerにせず、Pass中のrolling replanへ委ねる。

静的wall fallbackで観測corridorがない場合、従来の1.5 m上限を超えても2.2 m以内ならProgressive Entryに限り許可する。Complete Missionへ昇格させる既存guardは変えない。

Progressive Entryは既存のfrozen `OvertakePassPlan`へ変換し、Pass holdは既存の12 m extension windowを使用する。40 m/10 sの絶対Mission budget、runtime wall preplan、SafeSeparation、hard faultは維持する。

## Return抑制

tactical revalidationによるspeed-preserving Returnは、従来の2.0 mだけでなく既存の再選択安全距離4.0 mを満たしてから許可する。距離不足時はPass側を維持して再評価し、前方車の直後へ早戻りして再追従する負ループを避ける。

## 設定

- `v2x_overtake_progressive_entry_enabled: true`
- `v2x_overtake_progressive_entry_static_fallback_max_lateral_shift: 2.2`

## 失敗時の方針

Progressive Entry後も壁warningではsame-side replan、実wall violation・実車体hard conflict・進捗消失では既存SafeSeparation/Recoveryを使用する。「必ず突入」ではなく、「完全Missionの予測不足だけでは入口を諦めない」が境界である。
