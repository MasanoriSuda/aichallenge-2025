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

## Submission Artifacts

公開案内では、取り組みスライドと走行動画を提出する。スライドには少なくとも、
走行データ、他車両への回避・停止、モデル構成、学習データと評価、独自性を記載する。
提出フォーマットと期限は WIP のため、最新の公式ページと運営連絡を提出前に再確認する。
