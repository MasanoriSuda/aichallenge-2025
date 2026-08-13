# Requirements

## 背景

`output/20260814-003851` の1回目の追い越しでは、`ShiftOut -> Pass` 到達後、
locked target がまだ約4.2 m前方にいる状態で runtime wall warning が発生し、
`Pass -> Return` へ遷移した。その後、前車の後方へ戻って SafetyBrake が発生した。

同じ走行では、MPCC-lite の同側 Mission 置換が、hold に対するスコア差
`0.01`、`0.15` のような微小差でも発生していた。

## 必須要件

1. runtime wall warning だけを理由に、rear-clear前の Return を開始しない。
2. rear-clear前に同側の縮退候補が得られない場合、hard wall guardを無効化せず、
   現在の同側Missionを継続する。
3. MPCC-lite の同側Mission置換は、現Mission holdより候補が有意に優れる場合に限定する。
4. 同側Mission置換直後の短時間の再置換を抑止する。
5. wall contact、hard wall margin、EmergencyBrakeなど既存のhard guardは維持する。

## 変更しないもの

- ROS 2 topic、message、service契約
- V2X target選択契約
- Recovery / Reverse処理
- cross-side prefixのhard-feasible条件
- 評価基盤と`aichallenge_system/`

