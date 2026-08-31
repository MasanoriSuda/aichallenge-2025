# E2E Imitation Baseline Requirements

## Objective

既存TinyLidarNetの2周走行能力を保持しながら、MPC教師の独立runでsteeringをfine-tuneし、
固定start単車3周を完了できる候補weightを作る。

## Root Causes Addressed

- runtime checkpointがNumPy形式しかなく、trainerがwarm-startできない。
- 1本のteacher runだけではrun-level validationを構成できない。
- training outputが固定filenameへ上書きされ、dataset/checkpoint provenanceを追えない。
- random seedが固定されず、学習結果を再現できない。

## Constraints

- accelerationは固定runtime policyのままとし、lossはsteeringだけに適用する。
- student model入力は750点LiDARだけとする。
- train/validationはrun単位で分離する。
- 既存production weightを直接上書きせず、候補を別pathへ生成する。
- candidateはPyTorch/NumPy推論parityとclosed-loop 3周確認後だけ昇格する。

## Acceptance

- `.npy`runtime checkpointをkey/shape/finite検証付きでtrainerへloadできる。
- 2本以上のMPC teacher runがtrain/validationへ分離される。
- training manifestにdataset sequence、config、warm-start hashを残す。
- candidate `.pth`をruntime `.npy`へ変換し、推論parityを確認する。
- production weightを変更せずcandidateで`e2e-single`を試走できる。
