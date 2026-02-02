# rosbag 運用契約 v0.1（凍結）

## 目的
Mac/RPi の双方で同一の記録・再生フローを固定し、学習/SLAM/可視化の前提を揃える。

## 保存規約
- 保存先: `training/data/bags/<run_id>/`
- `metadata.json` を **同階層に必ず生成**する

## run_id 規約
- 生成: `tools/ros2/scripts/run_id.sh`
- 形式: `YYYYMMDD_HHMMSS_<short>`（例: `20260201_123456_a1b2c3d4`）
- `/mc/run_id` と bag/ログの run_id は一致させる

## metadata.json（必須）
最低限のキー:
- `run_id`
- `git_sha`
- `host`
- `notes`
追加フィールドは可（例: `created_at`）。

## record（固定）
- 記録対象（必須）: `/scan`, `/tf_static`, `/mc/status`, `/mc/drive_cmd`, `/mc/log`
- 追加推奨: `/mc/hils_state`, `/mc/run_id`, `/tf`
- `tools/ros2/scripts/bag_record.sh` を使用する

## play（固定）
- `tools/ros2/scripts/bag_play.sh` を使用する
- `--clock` を有効化した再生を標準とする
