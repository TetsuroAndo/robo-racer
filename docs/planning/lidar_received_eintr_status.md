# lidar_received EINTR 対応 状態管理

## 目的
- write_to_mem() の sem_wait() で EINTR を正しく処理し、共有メモリアクセスの破綻を防ぐ。

## ステータス
- [完了] write_to_mem() の sem_wait() を EINTR リトライ付きに変更

## メモ
- sem_wait() が中断された場合は再試行し、致命的エラー時のみ終了する。
