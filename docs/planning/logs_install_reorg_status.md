# logs/install 整理ステータス

## 目的
- `logs/`（ランタイム成果物）と `log/`（ROS2 buildログ）の混同を防ぐ
- ROS2 の `install` 出力を `rpi/ros2_ws/install` に固定する
- ルート直下の誤生成物（`install/`, `log/`）を整理する

## 状態

| 項目 | 状態 | 日付 | メモ |
| --- | --- | --- | --- |
| ROS2 build ログの出力先を `rpi/ros2_ws/colcon_log` に変更 | 完了 | 2026-02-05 | `ros2_build.sh` と `Makefile` を更新 |
| ルート直下の `install/` `log/` を除去・再発防止 | 完了 | 2026-02-05 | ルートの生成物を削除し、`.gitignore` を追加 |
| `logs/` と ROS2 buildログの役割分離を文書化 | 完了 | 2026-02-05 | `docs/ros2/dev_env.md` を更新 |
