# Requirements

## Purpose

`make gate2`でtrajectory変更後の開始横位置から遠い側の回廊へ移る際、MPCが
基準線headingを優先して十分に転舵せず、PASS zoneへ進入できない問題を解消する。

## Scope

- 停止前車に対する低速局所経路の自動左右選択
- 選択後のside lock、最低gap幅、壁・車両marginは維持する
- 一般走行のOvertake side選択やSafetyBrake判定は変更しない

## Acceptance criteria

- 両側が幅条件を満たす場合、現在横位置から目標までの移動量が小さい側を選ぶ
- 移動量が同等の場合のみ広い側を選ぶ
- 低速停止車回避では専用の最低gap幅を使い、一般走行用gap幅で上書きしない
- 選択回廊へ入る前はpass targetをsoft targetとして扱い、未到達回廊を即時hard boundにしない
- 選択回廊へ入った後は従来のhard boundを有効にする
- シフト中だけ速度を1.0 m/sに抑え、選択回廊へ入った後は通常のpass速度へ戻す
- 停止車列へ横移動する間は低速の横・heading feedbackで操舵し、通常走行のMPC重みは維持する
- 一時的なfront/side検出欠落ではMPCへ戻さず、車列全体のclearanceと連続clear時間を確認して復帰する
- pure core単体テストと`make autoware-build`が成功する
- `make gate2`でOSQP連続失敗せずPASS zoneへ進入する
