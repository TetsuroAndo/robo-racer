# Repository Guidelines

## Project Structure & Module Organization
- `src/`: Raspberry Pi 5 runtime (Python) with `common/`, `control/`,
  `perception/`, and `driver/`.
- `firmware/`: ESP32 firmware (C++) with `src/` and shared `lib/`.
- `training/`: training and analysis workspace (`training/src`, notebooks, data).
- `config/`: shared YAML configuration such as `vehicle.yaml`.
- `models/`: trained model artifacts (`production/` and `archive/`).
- `cad/`: KiCad hardware design files.
- `test/`: pytest-based Python tests.
- `docs/`, `tools/`, `playground/`: documentation, utilities, and experiments.

## Build, Test, and Development Commands
- `make init`: set up git hooks and Python env via `uv`.
- Python-related setup and commands should use `uv` (e.g., `uv sync`,
  `uv run pytest -q`) rather than calling `pip` directly.
- `make pysync`: sync Python dependencies (`pyproject.toml` extras).
- `make activate`: open a shell with the repo virtualenv.
- `make test`: run pytest quietly against `test/`.
- `make all`: build the host C++ binary `bin` (if C++ sources exist).
- `platformio run`: build ESP32 firmware using `platformio.ini`.
- `platformio run -t upload`: flash firmware to the board.
- `make tidy` / `make check`: clang-tidy or cppcheck static analysis.

## Coding Style & Naming Conventions
- C++: formatted by `.clang-format` (tabs, width 4, 80 columns). Run
  `clang-format -i` or use `make tidy` for static checks.
- Python: lint/format with `ruff` (e.g., `uv run ruff check .`).
- Tests: name files `test_*.py` or `*_test.py`, and test functions `test_*`.

## Testing Guidelines
- Framework: `pytest` configured in `pyproject.toml`.
- Location: keep Python tests in `test/` with clear unit coverage.
- Run locally with `make test` or `uv run pytest -q`.

## Commit & Pull Request Guidelines
- Commit history uses short, imperative subject lines (no required prefix).
- PRs should include a brief summary, testing notes, and any firmware or model
  impact. Add logs, plots, or screenshots when behavior changes are visible.

## Configuration & Data Notes
- Keep shared parameters in `config/*.yaml` and review changes carefully.
- Keep ESP32 servo calibration constants (min/max/center pulse widths) in
  `firmware/src/config/Config.h` as macros or `static const` values, and read
  them from `ServoSteering::begin(...)`.
- Treat `training/data/` as local data; avoid committing large datasets.
- Update `models/production/` only for release-ready model artifacts.

## Development Workflow Expectations
- 開発を始める前に `docs/` 内の関連資料を必ず参照し、仕様を確認してください。
- 実装前には、進捗と差異を追跡するためのステート管理用 Markdown ファイル（例: `docs/planning/ftg_implementation_status.md`）を作成し、進行中/完了/再検討などの状態を記録した上で実装に入ってください。
- 実装が完了したら、対応するステートファイルをこまめに更新し、どの機能がどの状態かを明示してください。
- これらの追加メモやステート記録は、**日本語**で記述してください。
