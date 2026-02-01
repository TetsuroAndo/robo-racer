# noVNC バインド方針と bag コピー運用ステータス

## 目的
- noVNC をデフォルトで 127.0.0.1 にバインドし、LAN 公開は明示的に指定する。
- RPi → PC への bag コピー運用をドキュメント化する。

## 状態
| 項目 | 状態 | 日付 | メモ |
| --- | --- | --- | --- |
| 既存 noVNC の公開範囲確認 | 完了 | 2026-02-01 | compose の ports と起動スクリプトを確認 | 
| noVNC のデフォルト bind 制限 | 完了 | 2026-02-01 | NOVNC_BIND を 127.0.0.1 に固定 | 
| LAN 公開の明示手順 | 完了 | 2026-02-01 | NOVNC_BIND=0.0.0.0 を記載 | 
| RPi から bag コピー手順の記載 | 完了 | 2026-02-01 | scp 例を追加 | 

## 進行メモ
- 対象: tools/ros2/scripts/novnc_start.sh / tools/ros2/compose.yml / docs/ros2/dev_env.md
