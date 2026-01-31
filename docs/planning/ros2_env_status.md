# ROS2 開発環境（Docker/Make/bag）ステータス（担当A）

## 目的
ROS2（RViz含む）を Ubuntu/Mac の Docker で再現可能にし、B/C が bag 前提で並行開発できる基盤を作る。

## 範囲（Aの追加責務）
- ROS2 Docker 環境（Ubuntu/Mac 対応）
- RViz 実行導線
- rosbag2 record/play の雛形
- topic/bag 規格（ICDのROS版）

## ステータス
| 項目 | 状態 | 期限 | 備考 |
| --- | --- | --- | --- |
| 既存 docs の精査 | 完了 | 2026-02-01 | ros2_setup_v1.md を確認 |
| docs 修正（A責務化・RViz・Mac対応明記） | 完了 | 2026-02-01 | ros2_setup_v1.md 更新 |
| tools/ros2 Dockerfile/compose 追加 | 完了 | 2026-02-03 | Ubuntu/Mac 両対応 |
| rosbag2 record/play スクリプト雛形 | 完了 | 2026-02-03 | run_id を付与 |
| Make 導線（ros2-up/shell/build/record/play） | 完了 | 2026-02-04 | 既存Makeと干渉しない |
| docs/ros2/{dev_env,bag_ops,topic_spec}.md | 完了 | 2026-02-04 | B/Cが質問なしで進める |
| ros2_ws/mc_msgs（msg雛形） | 完了 | 2026-02-04 | topic_spec に対応 |

## 残タスク（順序付きTODO）
| 順番 | タスク | 状態 | 期限 | 依存 | 根拠 | 備考 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | tools/ros2 Dockerfile/compose 追加 | 完了 | 2026-02-03 | - | 実行環境が先に必要（基盤） | Ubuntu/Mac 両対応 |
| 2 | docs/ros2/dev_env.md | 完了 | 2026-02-03 | 1 | 環境が固まらないと手順が書けない | RViz手順含む |
| 3 | docs/ros2/topic_spec.md | 完了 | 2026-02-03 | - | B/Cを早期に動かすため | ICDのROS版 |
| 4 | rosbag2 record/play スクリプト雛形 | 完了 | 2026-02-04 | 1 | 実行環境が必要 | run_id付与 |
| 5 | docs/ros2/bag_ops.md | 完了 | 2026-02-04 | 4 | スクリプトが決まらないと運用が書けない | 命名/保存規則 |
| 6 | Make 導線（ros2-up/shell/build/record/play） | 完了 | 2026-02-04 | 1 | Docker環境が必要 | 既存Makeと干渉しない |
| 7 | ros2_ws/mc_msgs（msg雛形） | 完了 | 2026-02-04 | 3 | topic_spec と整合させる | Drive/Status/Hils/Log |

## 進行メモ
- 2026-02-01: ros2_setup_v1.md を精査開始。
- 2026-02-01: ros2_setup_v1.md をA責務化＋RViz＋Mac対応で更新。
- 2026-02-01: tools/ros2（Docker/compose）実装を開始。
- 2026-02-01: tools/ros2 と docs/ros2、Make導線、bagスクリプトまで実装完了。
- 2026-02-01: ros2_ws/mc_msgs の雛形を追加。
