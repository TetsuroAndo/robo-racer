import shutil
import sys
import subprocess
from pathlib import Path

import pytest


@pytest.mark.skipif(
    sys.platform != "linux",
    reason="requires Linux AF_UNIX SOCK_SEQPACKET support",
)
def test_sender_cpp(tmp_path: Path):
    """
    @brief
      Build and run Sender behavior tests on host.
    """
    if shutil.which("g++") is None:
        pytest.skip("g++ not available")

    repo_root = Path(__file__).resolve().parents[3]
    test_cpp = repo_root / "test/rpi/comm/cpp/sender_tests.cpp"
    src = [
        repo_root / "rpi/src/Sender.cpp",
        repo_root / "shared/proto/src/mcproto.cpp",
        repo_root / "rpi/lib/mc_ipc/src/UdsSeqPacket.cpp",
        repo_root / "rpi/lib/mc_core/src/Log.cpp",
        repo_root / "rpi/lib/mc_core/src/Time.cpp",
    ]

    output = tmp_path / "sender_tests"
    cmd = [
        "g++",
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        "-O2",
        "-pthread",
        "-I",
        str(repo_root / "rpi/src"),
        "-I",
        str(repo_root / "shared/proto/include"),
        "-I",
        str(repo_root / "rpi/lib/mc_ipc/include"),
        "-I",
        str(repo_root / "rpi/lib/mc_core/include"),
        str(test_cpp),
        *[str(path) for path in src],
        "-o",
        str(output),
    ]

    build = subprocess.run(cmd, capture_output=True, text=True)
    print("\n[INFO] build cmd:", " ".join(cmd))
    assert build.returncode == 0, build.stdout + build.stderr

    run = subprocess.run([str(output)], capture_output=True, text=True)
    print("\n[INFO] run bin:", output)
    assert run.returncode == 0, (run.stdout or "") + (run.stderr or "")
