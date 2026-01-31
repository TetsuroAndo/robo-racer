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

## 4) ros2_ws のビルド
```
./tools/ros2/scripts/ros2_build.sh
```
`ros2_ws/src/mc_msgs` がメッセージ定義の雛形です。

## 5) bag の record/play
- record: `./tools/ros2/scripts/bag_record.sh`
- play: `./tools/ros2/scripts/bag_play.sh <bag_path> [rate]`
