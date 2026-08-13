# Requirements

## 背景

`output/20260814-081546`ではprogressive prefix entryにより、`Idle -> ShiftOut`は前回の1回から
6回へ増えた。一方、6回のうち正常な`Return -> Idle`完遂は1回で、2回は実行中prefixの
target-separation不成立から`FollowPrepare`へ移り、横経路とOvertake速度ownershipを失った。

現行のdynamic Mission waitは「停止して完全Missionを待つ」構造であり、追い越し中に空いた側の
fresh prefixへ連続接続するrolling plannerとしては動作していない。

## 目的

- softな実行prefix不成立では追い越し意図、target、現在の横位置を保持する。
- dynamic Mission wait中も左右と同側継続prefixを再評価し、admitted prefixへ直接置換する。
- fresh prefixを待つ短時間は、Followへ戻さず壁内の物理hold lineを出力する。
- 現Missionがinvalidatedされた場合、stale hold候補をMPCC-liteの勝者として扱わない。
- 同側継続はno-return後も許可し、反対側への全幅切替は従来どおりno-return前だけに制限する。

## 制約

- actual wall contact/margin violation、EmergencyBrake、solver recovery、target discontinuityは従来どおり
  hard faultとして中断する。
- dynamic waitは既存の時間・距離上限内に限定し、無期限に不成立経路を保持しない。
- FollowPrepareの一般用途とSafetyBrake pauseは変更せず、DynamicMissionWait原因だけを対象にする。
- topic/service/Domain/提出インターフェースは変更しない。

## 完了条件

- DynamicMissionWait中のhard-feasible progressive prefixがsame-side/cross-side replacement authorityを得る。
- invalidated current Missionのstale holdがfresh左右候補を隠さない。
- replacement待ちの周期でも横経路が消えず、BehaviorはOvertakeを維持する。
- 対象packageのbuild/testが成功する。
