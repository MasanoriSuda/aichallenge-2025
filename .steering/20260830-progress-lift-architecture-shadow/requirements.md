# Requirements

## Objective

`output/20260830-150910` のPass中に発生した
`progress-lift-rejected -> Emergency Stop -> progress stalled`を、production authorityを
変更せずarchitecture escape-hatch比較で分類する。

## Frozen failures

- episode 1、sequence 1278:
  - stateless sibling side `-1 -> +1` をdecision 1920でpublish。
  - decision 1945でphysical progress 115.859 m、artifact 117.563 m、差-1.704 m。
- episode 3、sequence 2966:
  - stateless sibling side `+1 -> -1` をdecision 3607でpublish。
  - decision 3643でphysical progress 279.772 m、artifact 281.279 m、差-1.506 m。

両方とも約0.6秒後に既存1.5 m continuity gateを越え、normal authorityが消失した。

## Hypotheses

### H1: persistent artifact clock / current-world Bundle lifecycle mismatch

同側または反対側のartifact controlsは現在状態から再証明可能でも、publish後は元artifactの
time cursorへ戻り、現在状態とのprogress差が蓄積する。

反証: progress gateを観測専用で越えたstateless rebaseも、wall/dynamic/terminal proofの
いずれかで不成立になる。

確信度: high。

### H2: control-origin progressとexact nonlinear progressのmodel mismatch

`physical_course_progress()`は現在のassociated waypointを基準にpredicted control poseを
投影する一方、artifactはcontinuous seven-state nonlinear Frenetを積分する。tight curve、
large lateral offset、cross-side adoptionで差が拡大する可能性がある。

反証: current-world rebaseの幾何joinも不成立で、両表現が同じ物理不成立を示す。

確信度: medium-high。

### H3: physical infeasibility

反対側へ採用したBundle自体が、その後の実車状態から壁・相手・Stop suffixを成立させられない。

反証: Bの全証明がAcceptedになる。

確信度: medium。

## Constraints

- production authority、publisher、Stop authorityを変更しない。
- Mission resume、lease、grace、timeout、fallbackを追加しない。
- progress tolerance、solver tolerance、wall/vehicle clearanceを変更しない。
- Bの成功をそのままcommand publicationへ接続しない。

## Acceptance

- AがProgressLiftRejectedとなる同一Requestを、Bが最後まで評価した結果を記録できる。
- Bはwall、dynamic obstacle、terminal Stop suffixを迂回しない。
- production resultは従来どおりProgressLiftRejectedのままである。
- dev2で各frozen failureを`A fails/B succeeds`または`A/B fail`へ分類できる。
