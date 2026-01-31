# ROS2 / SLAM 手順 v1（タスクA）

## 目的
- SLAM/可視化（RViz含む）環境を **再現可能** に用意する
- 別PC/別人でも同じ手順で地図生成できる状態にする
- bag 運用を含む標準I/Fを固定し、B/C の並行開発を可能にする

## 前提
- ROS2: Humble（推奨）
- 方式: Docker優先（OS差分を最小化）
- ホスト: **Ubuntuを主**、**Macも同等に対応**
- 入力: LiDARのログ（IPC/ログから変換）

---

## 期待する入力
- `sensor_msgs/LaserScan` 形式の bag を用意する
- 元データは `IPC_LIDAR_SCAN` から生成する

**変換の最小仕様**
- frame_id: `laser`
- angle_min/max: 0〜2π（または -π〜π）で統一
- range_min/max: LiDAR仕様に合わせる

---

## 手順（v1）

## 配置先（確定）
- Dockerfile/compose: `tools/ros2/`
  - `tools/ros2/Dockerfile`
  - `tools/ros2/compose.yml`
- 変換ツール: `tools/ros2/scripts/`
  - `tools/ros2/scripts/convert_ipc_lidar_to_rosbag.py`

### 1) Docker環境の用意
- `tools/ros2/` に Dockerfile/compose を配置する
- コンテナ内で `ros2` と `slam_toolbox` と `rviz2` を使用可能にする

**例（方針）**
```
docker build -t robo-racer-ros2 -f tools/ros2/Dockerfile .
docker run --rm -it -v $PWD:/ws robo-racer-ros2
```

#### GUI（RViz）を使う場合
**Ubuntu（X11）**
- `xhost +local:root`
- `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw`

**Mac（Docker Desktop + XQuartz）**
- XQuartz を起動し「接続を許可」を有効化
- `DISPLAY=host.docker.internal:0` を指定
- 必要に応じて `xhost + 127.0.0.1` を許可

### 2) bag の準備
- `training/data/` または `logs/` から bag を生成
- 変換ツールは `tools/` に追加する（v1では最小変換でOK）

### 3) slam_toolbox（オフライン）
- `slam_toolbox` の offline モードで地図生成
- 生成した地図は `docs/slam/maps/` に保存する

**例（方針）**
```
ros2 launch slam_toolbox offline_launch.py
ros2 bag play <bag_path>
ros2 run nav2_map_server map_saver_cli -f docs/slam/maps/track
```

---

## 再現性チェック（DoD）
- 別PCで同じ手順で map が生成できる
- 同じbagで同じ地図そのよが得られる（大きな差分がない）

---

## TODO（v1で確定する内容）
- `tools/ros2/` の Dockerfile/compose を追加
- bag 変換ツールの最小実装（`tools/ros2/scripts/convert_ipc_lidar_to_rosbag.py`）
- slam_toolbox のパラメータセット（scan_matcher など）
- RViz の起動手順（Ubuntu/Mac）を確定
