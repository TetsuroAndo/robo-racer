# ROS2 Telemetry 出力統合 ステート

## 目的
- ROS2 の `/odom` を正として位置・姿勢・速度を取得し、telemetry に出力する。
- `/imu` の生値（加速度/角速度）を telemetry に含める。
- `/tf` / `/tf_static` を用いた座標変換（map->odom があれば map 系も併記）を行う。
- TSD20 は ESP32 LOG（`tsd20:` 行）から抽出して telemetry に含める。

## ステート

| 項目 | 状態 | 更新日 | メモ |
| --- | --- | --- | --- |
| 関連資料の確認 | 完了 | 2026-02-06 | `docs/ros2/topic_spec.md` / `docs/contracts/ros_topics_v0.1.md` |
| ROS2購読とtelemetry出力 | 完了 | 2026-02-06 | mc_bridge に /odom /imu /tf /tf_static を追加 |
| TSD20抽出 | 完了 | 2026-02-06 | LOG から `tsd20:` を解析して telemetry に反映 |
