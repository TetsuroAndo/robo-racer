# rosbag 運用 v0.1

## 保存場所
- `training/data/bags/<date>_<run_id>/`

## run_id 付与
- `bag_record.sh` が UUID を自動付与
- `PUBLISH_RUN_ID=1` の場合、`/mc/run_id` を transient_local で publish
- `/mc/log.run_id` は **必須**（`/mc/run_id` と一致させる）
- `OUT_DIR` を指定すれば任意の場所に保存可能

## record
```
RUN_ID=<uuid> TOPICS="/scan /mc/status /mc/drive_cmd" \
  ./tools/ros2/scripts/bag_record.sh
```
- `TOPICS` が最優先（完全手動）
- `TOPICS` 未指定なら `PROFILE` に従う（デフォルト: `core`）
- `PROFILE=all` のときのみ `--all` で記録
- `meta.txt` に run_id / git_sha / host / timestamp を保存
- `meta.txt` に profile / topics / topics_file も保存
- `meta.txt` に `present_topics`（記録開始時に見えていた topic）も保存

## play
```
./tools/ros2/scripts/bag_play.sh <bag_path> [rate]
```

## topics プロファイル
- `tools/ros2/topics/<profile>.txt` で管理
- 例: `PROFILE=slam ./tools/ros2/scripts/bag_record.sh`

## session 起動（run_id 伝播を固定したい場合）
```
SESSION_CMD="ros2 launch <pkg> <file.launch.py>" \
  ./tools/ros2/scripts/session_up.sh
```
- Make 経由: `SESSION_CMD="ros2 launch <pkg> <file.launch.py>" make ros2-session-up`
- `session_up.sh` が `RUN_ID` を生成・publish し、同じ `RUN_ID` で `bag_record.sh` を起動
- `SESSION_CMD` 未指定なら bag 記録のみ

## 推奨トピック
- `/scan` (sensor_msgs/LaserScan)
- `/mc/status`
- `/mc/drive_cmd`
- `/mc/hils_state`
- `/mc/log`
- `/mc/run_id`
- `/tf_static`（推奨）
- `/tf`（推奨、出ている場合）
