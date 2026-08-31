# End to End AI 部門ベース仕様

> Automotive AI Challenge 2026 End to End AI 部門向けのローカル整理。
> 正本は公式ドキュメント:
> <https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/ai-class.html>
>
> 確認日: 2026-09-01
>
> 公式ページは WIP。提出期限はチームへ Slack で共有された 2026-09-07 を
> 作業計画に使用するが、公開仕様として確定した値ではない。

## Runtime Input Contract

走行時に End to End AI が使用できる入力は次の範囲とする。

- Camera
- LiDAR
- Steer Angle
- Wheel Odometry
- Gear Status

GNSS、IMU、V2X、地図上の自己位置、計画 trajectory、MPC の出力は、E2E 推論の
入力に使用しない。評価ハンドシェイクを維持する目的で既存 node が起動していることと、
モデルがその出力を利用することは分けて監査する。

大会説明では入力から横方向制御まで ML を使用することが前提である。現ローカルの
最初の baseline は 2D LiDAR から steering を直接出力する TinyLidarNet とし、
longitudinal は固定加速度から開始する。

## Safety Gates

End to End AI 部門でも次を評価できる構成を維持する。

- 障害物停止
- NPC 追い越し
- 車線維持

単車周回を成立させた後、停止車両、低速車両、他車両を含む順に評価範囲を広げる。

## Local Development Stages

1. 固定スタート・単車で 3 周連続走行
2. 複数 start / seed で単車再現性確認
3. 停止障害物と sensor stale の安全確認
4. 教師 bag の同期・分割・失敗区間を監査
5. recovery data を追加して再学習
6. NPC / 他車両を含む動的障害物対応
7. 4 台・6 周の決勝参考条件

MPC / MPCC は教師データ生成と比較評価に使えるが、E2E controller の推論入力には
しない。

## Teacher Dataset Contract

教師データはbag/runを最小のidentityおよびsplit単位とする。sample単位で同一runを
trainとvalidationへ分割してはならない。

- 入力: `/sensing/lidar/scan`
- 教師label: `/control/command/control_cmd`
- 既定同期上限: 50 ms
- 既定scan契約: 750点、最大30 m
- 既定教師出所: `mpc`、`mpcc`、`human`

抽出時に教師出所を必ず記録し、同期timestamp、同期差、元bag、topic型、scan shape、
採用/reject件数をsequence metadataへ保存する。E2E自身のcommandを記録した失敗bagは
`student`として観測・解析できるが、corrective labelなしに教師へ混ぜない。

trainerはmetadata欠損、shape/type不一致、同期上限超過、非finite値、教師出所不一致、
train/validationのsequence重複を学習開始前に拒否する。古いdatasetを暗黙補完せず、
契約対応extractorで再生成する。

MPC教師収集は`make e2e-teacher`を使う。これは`e2e-single`と同じ1台・NPCなし・固定
start・3周・LiDAR onの条件でcontrollerだけをMPCへ切り替え、localizationに必要なIMUを
追加する。IMU/GNSSは教師controllerとAWSIM infrastructureだけが利用し、student model
featureへは追加しない。抽出時は`--label-source mpc`を指定する。

## Submission Artifacts

公開案内では、取り組みスライドと走行動画を提出する。スライドには少なくとも、
走行データ、他車両への回避・停止、モデル構成、学習データと評価、独自性を記載する。
提出フォーマットと期限は WIP のため、最新の公式ページと運営連絡を提出前に再確認する。
