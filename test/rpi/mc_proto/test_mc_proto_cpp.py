import shutil
import subprocess
from pathlib import Path

import pytest


def test_rpi_mc_proto_cpp(tmp_path: Path):
    if shutil.which("g++") is None:
        pytest.skip("g++ not available")

    repo_root = Path(__file__).resolve().parents[3]
    test_cpp = repo_root / "test/rpi/mc_proto/cpp/proto_tests.cpp"
    proto_src = [
        repo_root / "rpi/lib/mc_proto/src/mcproto.cpp",
    ]

    output = tmp_path / "mc_proto_tests"
    cmd = [
        "g++",
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        "-O2",
        "-I",
        str(repo_root / "rpi/lib/mc_proto/include"),
        "-I",
        str(repo_root / "shared/proto"),
        str(test_cpp),
        *[str(path) for path in proto_src],
        "-o",
        str(output),
    ]

    build = subprocess.run(cmd, capture_output=True, text=True)
    assert build.returncode == 0, build.stdout + build.stderr

    run = subprocess.run([str(output)], capture_output=False, text=True)
    assert run.returncode == 0, run.stdout + run.stderr
