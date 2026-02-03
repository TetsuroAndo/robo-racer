# process 意思決定テレメトリ 状態管理

## 目的
- `Process::proc()` の意思決定過程（best角/候補/override）を観測可能にする
- ターミナルでリアルタイムに状況把握できる最小UIを提供する

## 状態一覧
- [完了] `Process` のテレメトリログ（best/min/path/cmd/override）の追加
- [完了] ASCIIコンパス + top-3 + override表示の追加
- [完了] JSONL 形式の telemetry/event 出力の追加
- [完了] run_id/tick/scan_id 付与（run_id は起動時生成）
- [見送り] ログ頻度の間引き/レート制御（推奨だが今回未実装）

## メモ
- 参照資料: `docs/observability/logging_v1.md`
