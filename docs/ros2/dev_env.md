# ROS2 開発環境（Docker/RViz）v0.1

## 目的
- Ubuntu/Mac の両方で ROS2 + RViz を再現可能にする
- bag 運用や SLAM 検証をコンテナ内で統一する

## 前提
- ROS2: Humble
- Docker/compose を利用
- リポジトリを `/ws` にマウント

## 1) Docker イメージのビルド
```
docker compose -f tools/ros2/compose.yml build
```

## 2) シェルに入る
```
docker compose -f tools/ros2/compose.yml run --rm ros2 bash
```

## 3) RViz 起動（GUI）
### 事前注意（DISPLAY）
`docker compose up` は **DISPLAY を自動設定しない**。ホストの環境変数に従うため、
Ubuntu は `DISPLAY=:0`、Mac は `DISPLAY=host.docker.internal:0` を **事前に設定**してから起動すること。
一方で `make ros2-up` / `make ros2-rviz` は Mac の場合に DISPLAY などを自動設定する。

### Ubuntu（X11）
```
xhost +local:root
export DISPLAY=:0
```
コンテナ内で：
```
rviz2
```

### Mac（Docker Desktop + XQuartz）
1. XQuartz を起動し「接続を許可」を有効化
2. ホスト側で以下を設定
```
export DISPLAY=host.docker.internal:0
xhost + 127.0.0.1
```
3. コンテナ内で `rviz2`

### Mac 自動設定（Make 経由）
`make ros2-rviz` / `make ros2-up` は Mac の場合に以下の環境変数を自動設定する：
- `DISPLAY=host.docker.internal:0`
- `LIBGL_ALWAYS_SOFTWARE=1`
- `QT_XCB_GL_INTEGRATION=none`
- `XDG_RUNTIME_DIR=/tmp/runtime-root`
XQuartz の設定と `xhost` は必要。

### RViz デフォルト設定
```
make ros2-rviz
```
`tools/ros2/rviz/default.rviz` を使用して起動する。

## 4) rpi/ros2_ws のビルド
```
./tools/ros2/scripts/ros2_build.sh
```
`rpi/ros2_ws/src/mc_msgs` がメッセージ定義の雛形です。

## 5) demo publisher（/scan）と static TF
```
ros2 run mc_tf_static mc_tf_static --ros-args --params-file /ws/rpi/config/frames.yaml
ros2 run mc_demo_pub mc_demo_pub
```
`/scan` は `frame_id=laser` で publish される。

## 6) bag の record/play
- record: `./tools/ros2/scripts/bag_record.sh`
- session: `./tools/ros2/scripts/session_up.sh`（run_id 伝播を固定したい場合）
- play: `./tools/ros2/scripts/bag_play.sh <bag_path> [rate]`
