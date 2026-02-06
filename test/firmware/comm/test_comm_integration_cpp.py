import shutil
import subprocess
from pathlib import Path

import pytest


def test_firmware_comm_integration_cpp(tmp_path: Path):
    """
    @brief
      Build and run firmware comm integration tests on host.
    """
    if shutil.which("g++") is None:
        pytest.skip("g++ not available")

    repo_root = Path(__file__).resolve().parents[3]
    test_cpp = repo_root / "test/firmware/comm/cpp/comm_integration_tests.cpp"
    proto_src = [
        repo_root / "shared/proto/src/mcproto.cpp",
        repo_root / "firmware/src/comm/registry.cpp",
        repo_root / "firmware/src/comm/handlers/DriveHandler.cpp",
        repo_root / "firmware/src/comm/handlers/ModeHandler.cpp",
        repo_root / "firmware/src/comm/handlers/KillHandler.cpp",
        repo_root / "firmware/src/comm/handlers/PingHandler.cpp",
        repo_root / "firmware/src/log/AsyncLogger.cpp",
        repo_root / "test/firmware/comm/cpp/UartTx_stub.cpp",
    ]

    output = tmp_path / "firmware_comm_tests"
    cmd = [
        "g++",
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        "-O2",
        "-I",
        str(repo_root / "test/firmware/comm/include"),
        "-I",
        str(repo_root / "firmware/src"),
        "-I",
        str(repo_root / "shared/proto/include"),
        "-I",
        str(repo_root / "shared/config/include"),
        str(test_cpp),
        *[str(path) for path in proto_src],
        "-o",
        str(output),
    ]

    build = subprocess.run(cmd, capture_output=True, text=True)
    print("\n[INFO] build cmd:", " ".join(cmd))
    assert build.returncode == 0, f"build failed rc={build.returncode}"

    run = subprocess.run([str(output)], capture_output=False, text=True)
    print("\n[INFO] run bin:", output)
    assert run.returncode == 0, (run.stdout or "") + (run.stderr or "")
