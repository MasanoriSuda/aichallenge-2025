---
name: error-analyzer
description: colcon、Docker Compose、Autoware/AWSIM起動、DDS接続の失敗をログから切り分ける。lap/penalty集計はevaluation-analyzer。
---

# Runtime and build failures

ユーザー指定のログを優先する。未指定なら`output/latest/`の実体を解決し、
同じrun/Domainの`autoware.log`、`ros/log/`、build logを必要な範囲だけ読む。
colconの正規workspaceは`aichallenge/workspace/`。過去installとの混在を確認する。

- Build: 最初の依存解決・CMake・message生成・entry pointの失敗を追う。
- Launch: package、include、引数、param/remapの実体を追う。
- DDS: Domain、QoS、CycloneDDS、clock、publisher/subscriberを照合。
- Eval起動: initial pose、engage、AWSIM state、outputと終了処理を追う。

後続エラーを原因と決めつけず、最初の不整合とproducerをログ・コードで示す。
Docker/ROSの確認はMakefile/Compose経由。原因、根拠、次の最小検証を簡潔に報告する。
解析のみなら変更しない。修正依頼なら原因を確定し、修正と検証まで続ける。
