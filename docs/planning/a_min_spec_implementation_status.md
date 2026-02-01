# A 最小仕様 v0.1 実装ステータス

## 目的
`docs/planning/a_min_spec_plan.md` の Day1〜Day3 相当を実装し、B/C が並行開発できる最小仕様を固定する。

## 前提（確認済み）
- ROS2 Docker 環境は `tools/ros2/` を利用（`docs/slam/ros2_setup_v1.md` 方針に合わせる）
- 既存の rpi/ros2_ws/mc_msgs は維持し、契約/運用を不足分で補う

## ステータス
| 項目 | 状態 | 期限 | 備考 |
| --- | --- | --- | --- |
| docs/contracts/ros_topics_v0.1.md 追加 | 完了 | 2026-02-08 | topic/型/frame_id 凍結を明文化 | 
| rpi/ros2_ws/mc_msgs の契約整合 | 完了 | 2026-02-01 | 既存 msg を維持 | 
| mc_tf_static 追加（base_link->laser） | 完了 | 2026-02-08 | rpi/config/frames.yaml を利用 | 
| mc_demo_pub 追加（/scan 疑似発行） | 完了 | 2026-02-08 | RViz 表示の最短経路 | 
| tools/ros2/rviz/default.rviz 追加 | 完了 | 2026-02-08 | Fixed Frame=base_link | 
| Makefile に ros2-rviz 追加 | 完了 | 2026-02-08 | RViz 起動導線 | 
| bag_record/play を規約化（run_id/metadata.json/--clock） | 完了 | 2026-02-08 | metadata.json を必須化 | 
| tools/ros2/scripts/run_id.sh 追加 | 完了 | 2026-02-08 | run_id 生成規則の統一 | 
| docs/contracts/bag_ops_v0.1.md 追加 | 完了 | 2026-02-08 | bag 運用凍結 | 
| docs/contracts/ipc_uds_v0.1.md 追加（推奨） | 完了 | 2026-02-08 | telemetry socket 契約 | 
| docs/ros2/bag_ops.md の整合更新 | 完了 | 2026-02-08 | metadata.json/保存先 | 

## 残タスク（順序付き TODO）
| 順番 | タスク | 状態 | 依存 | 備考 |
| --- | --- | --- | --- | --- |
| 1 | run_id.sh 追加と bag_record.sh 更新 | 完了 | - | metadata.json 出力まで含む |
| 2 | bag_play.sh の --clock 対応 | 完了 | 1 | Makefile への反映も含む |
| 3 | ros_topics_v0.1.md 作成 | 完了 | - | topic_spec.md と整合 | 
| 4 | bag_ops_v0.1.md 作成 | 完了 | 1 | docs/ros2/bag_ops.md も更新 | 
| 5 | mc_tf_static / mc_demo_pub 追加 | 完了 | - | rpi/ros2_ws ビルド対象 | 
| 6 | rviz/default.rviz 追加 + ros2-rviz 導線 | 完了 | 5 | RViz で /scan を確認 | 
| 7 | ipc_uds_v0.1.md 作成 | 完了 | - | 推奨（P0） |

## 差異・留意
- RViz GUI は既存の X11/XQuartz 手順を維持（noVNC は今回未対応）。

## 進行メモ
- 2026-02-01: a_min_spec_plan.md と min_spec_v0_1.md を確認。
- 2026-02-01: ros2/topic_spec.md・bag_ops.md・dev_env.md を確認。
- 2026-02-01: ros_topics/bag_ops 契約文書と run_id/metadata.json を実装。
- 2026-02-01: mc_tf_static / mc_demo_pub / rviz config / ros2-rviz 導線を追加。
- 2026-02-01: config/ と ros2_ws を rpi/ 配下へ移動し、参照パスを更新。
