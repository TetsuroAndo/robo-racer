# ROS2 headless record 対応ステータス

## 目的
- RPi (arm64) 上で headless の rosbag 記録を行う
- 余分な GUI/SLAM パッケージを避ける

## 状態
| 項目 | 状態 | 日付 | メモ |
| --- | --- | --- | --- |
| headless 向け Dockerfile を追加 | 完了 | 2026-02-05 | `tools/ros2/Dockerfile.record` |
| compose に `ros2-record` を追加 | 完了 | 2026-02-05 | GUI 依存を除外 |
| Make でサービス切替を追加 | 完了 | 2026-02-05 | `ROS2_SERVICE` |
| docs 更新（headless 手順） | 完了 | 2026-02-05 | `docs/ros2/dev_env.md` / `docs/ros2/bag_ops.md` |
