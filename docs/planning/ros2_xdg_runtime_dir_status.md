# ROS2 サービスの XDG_RUNTIME_DIR 初期化ステータス

## 目的
- ros2 サービス起動時に XDG_RUNTIME_DIR を作成し、Qt/RViz が失敗しないようにする。
- novnc_start.sh と同等の mkdir/chmod をエントリーポイントで実行する。

## 状態
| 項目 | 状態 | 日付 | メモ |
| --- | --- | --- | --- |
| 既存挙動の確認 | 完了 | 2026-02-01 | ros2 サービスで XDG_RUNTIME_DIR が未作成 | 
| エントリーポイントの追加 | 完了 | 2026-02-01 | mkdir -p と chmod 700 を追加 | 
| Dockerfile への組み込み | 完了 | 2026-02-01 | ラッパーを ENTRYPOINT に設定 | 
| ステート更新 | 完了 | 2026-02-01 | 反映済み | 

## 進行メモ
- 対象: tools/ros2/Dockerfile / tools/ros2/scripts/ros2_entrypoint.sh
- 参考: tools/ros2/scripts/novnc_start.sh
