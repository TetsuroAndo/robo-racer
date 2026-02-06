# ros2-mapping 修正ステータス

## 目的
- `ros2-mapping` 実行時の `colcon build` 失敗（merge-install 不一致）を解消する
- ROS2 ビルドを既存スクリプトに統一して再現性を上げる

## 状態
| 項目 | 状態 | 日付 | メモ |
| --- | --- | --- | --- |
| `ros2-mapping` のビルドを `ros2_build.sh` 経由に変更 | 完了 | 2026-02-05 | `--merge-install` を統一 |
| `ros2-mapping` の失敗時停止を有効化 | 完了 | 2026-02-05 | `set -e` を付与 |
