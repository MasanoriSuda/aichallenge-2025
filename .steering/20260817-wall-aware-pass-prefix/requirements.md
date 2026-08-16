# Requirements

## Goal

Pass 中に target-only horizon conflict を保持している最中でも、壁接近警告を検出したら同じ横経路の長時間延長を止め、rear-clear 前に前進を維持した内向き補正へ切り替える。

## Evidence

- 対象走行: `output/20260817-081104`
- `Idle -> ShiftOut`: 5 回
- `ShiftOut -> Pass`: 5 回
- `Pass -> Return`: 1 回
- `Return -> Idle`: 0 回
- Recovery 遷移: 3 回
- solver failure / recovery: 0 回
- target-bound hold は 7 回開始し、3 回は fresh horizon で解決したため前段修正は有効。
- 一方、Pass の progress extension が 9.60 m / 14.06 m まで同じ側を保持した。
- その途中で `runtime wall preplan` が `ttc=0.35 s` を通知したが、rear-clear 前のため Return が抑止され、最終的に static wall contact / Recovery へ移った。

## Required behavior

1. runtime wall preplan warning 中は target-bound hold の forward-progress extension を許可しない。
2. 短い optimizer repair budget は維持するが、wall warning を前進延長の根拠にしない。
3. robust target clearance と wall reserve が同時に成立しない場合でも、現在車体が非重複で同じ pass side を保てるなら、物理車体境界までを下限とする内向き補正候補を評価する。
4. 内向き補正は wall / lateral acceleration / full execution preflight を通過した場合だけ Mission として置換する。
5. 補正中も Pass と target identity、front-cap release を可能な範囲で保持し、Follow / Recovery への不要な失速を避ける。
6. wall contact、hard wall margin、wall sample unavailable、current-body overlap、emergency、solver recovery は従来どおり hard fault とする。

## Non-goals

- wall hard margin や車体寸法の縮小
- 接触を許可する新規 attack mode
- Recovery / Reverse の変更
- ROS topic / service / result schema の変更
- フル MPCC への置換

## Definition of Done

- target-bound hold の progress extension が wall warning で閉じる。
- runtime wall center contraction が nominal clearance 不成立時に physical clearance 候補を評価できる。
- nominal / physical の採用区分をログで確認できる。
- core 単体テストと `multi_purpose_mpc_ros` build/test が成功する。
