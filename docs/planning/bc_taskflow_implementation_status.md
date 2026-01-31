# タスクB（ルールベース/SLAM/安全）・タスクC（AI/データ/推論入口） タスクフロー&実装計画 ステート管理

目的: タスクB（ルールベース/SLAM/センサー安全）とタスクC（AI/データ/推論入口）のタスクフローと実装計画の進行状況を追跡する。

## 状態凡例
- 未着手
- 進行中
- 完了
- 再検討
- 保留

## 進捗サマリ
- 作成日: 2026-01-27
- 更新日: 2026-01-29
- 対象ドキュメント: docs/planning/bc_taskflow_plan.md

## ステータス一覧

### ドキュメント作成
| 項目 | 状態 | メモ |
| --- | --- | --- |
| タスクB/タスクCタスクフロー&実装計画書の新規作成 | 完了 | docs/planning/bc_taskflow_plan.md を作成 |
| FTG実装計画（v1） | 完了 | docs/planning/ftg_v1.md を作成 |
| ROS2/SLAM手順（v1） | 完了 | docs/slam/ros2_setup_v1.md を作成 |
| Dataset schema（v1） | 完了 | docs/ml/dataset_schema_v1.md を作成 |
| Policy plugin 仕様（v1） | 完了 | docs/ml/policy_plugin_v1.md を作成 |
| IPC payload 定義（v1） | 完了 | docs/interfaces/ipc_payloads_v1.md を作成 |
| タスクA/B/C 定義書（v1） | 完了 | docs/planning/role_definitions_v1.md を作成 |

### タスクB（ルールベース/SLAM/安全）
| 項目 | 状態 | メモ |
| --- | --- | --- |
| FTGタスクフロー定義 | 進行中 | 入出力・周期・安全条件を明記 |
| LIDAR入力/IPC設計反映 | 進行中 | IPC topicはv1凍結仕様に合わせる |
| racerd ルールベース統合フロー | 進行中 | DRIVE TTL前提を固定 |
| セーフティ拡張（single-point/IMU） | 進行中 | ESP32責務優先 |
| ROS2/SLAM手順 | 進行中 | 再現可能な手順を優先 |

### タスクC（AI/データ/推論入口）
| 項目 | 状態 | メモ |
| --- | --- | --- |
| データ収集フロー定義 | 進行中 | ログ仕様v1を参照 |
| dataset schema 設計 | 進行中 | timestamp整列・欠損扱い |
| 学習パイプライン v0 | 進行中 | imitation優先 |
| policy plugin 仕様 | 進行中 | FTGフェイルセーフ必須 |
