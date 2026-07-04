# multi_purpose_mpc_ros

このパッケージはリポジトリ内（`aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`）に直接収録されています。別途 `git clone` は不要です。

## build

autoware コンテナ内で実行します（`make autoware-bash` または `make autoware-build`）：

```bash
cd /aichallenge/workspace
colcon build --symlink-install --allow-overriding gyro_odometer \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- 通常の MPC 実行系は C++ executable `mpc_controller_cpp` です。
- Python 補助スクリプト用に、ビルド時に仮想環境が `${ROS_WS}/install/multi_purpose_mpc_ros/.venv` に作成されます。

## run

### MPC コントローラー
```bash
ros2 run multi_purpose_mpc_ros mpc_controller_cpp \
  --config_path $(ros2 pkg prefix --share multi_purpose_mpc_ros)/config/config.yaml \
  --ref_vel_path $(ros2 pkg prefix --share multi_purpose_mpc_ros)/config/ref_vel.yaml
```

Python 版は比較・検証用に残しています。

```bash
ros2 run multi_purpose_mpc_ros run_mpc_controller.bash
```

### MPC シミュレーション
```bash
ros2 run multi_purpose_mpc_ros run_mpc_simulation.bash
```

### Trajectory editor
MPC の `env/final_ver3/traj_mincurv.csv` を Lanelet2 map 上で編集します。

```bash
ros2 run multi_purpose_mpc_ros trajectory_editor
```

Pure Pursuit 用の `simple_trajectory_generator/data/raceline_awsim_30km_from_garage.csv` を開く場合:

```bash
ros2 run multi_purpose_mpc_ros pure_pursuit_trajectory_editor
```

### まとめて起動（コントローラー + シミュレーション）
```bash
ros2 launch multi_purpose_mpc_ros test.launch.xml
```

### Attribution
This repository includes code derived from:

Multi-Purpose-MPC  
Author: Mats Steinweg  
Original repository: https://github.com/matssteinweg/Multi-Purpose-MPC

Used with permission from the author.
