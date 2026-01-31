# rosbag 運用 v0.1

## 保存場所
- `training/data/bags/<date>_<run_id>/`

## run_id 付与
- `bag_record.sh` が UUID を自動付与
- `OUT_DIR` を指定すれば任意の場所に保存可能

## record
```
RUN_ID=<uuid> TOPICS="/scan /mc/status /mc/drive_cmd" \
  ./tools/ros2/scripts/bag_record.sh
```
- `TOPICS` 未指定なら `--all` で記録
- `meta.txt` に run_id / git_sha / host / timestamp を保存

## play
```
./tools/ros2/scripts/bag_play.sh <bag_path> [rate]
```

## 推奨トピック
- `/scan` (sensor_msgs/LaserScan)
- `/mc/status`
- `/mc/drive_cmd`
- `/mc/hils_state`
- `/mc/log`

