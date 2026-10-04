from pathlib import Path
import re
from typing import Final


EXPECTED_ENCODER_TPR: Final = 227.0
WORKSPACE_ROOT: Final = Path(__file__).resolve().parents[3]
ACTIVE_FIRMWARE_FILES: Final = (
    WORKSPACE_ROOT / "kaia_leonardo_robot/kaia_esp32/config.h",
    WORKSPACE_ROOT / "src/test_slam_nav2/firmware/robot_lowlevel_v2.ino",
    WORKSPACE_ROOT
    / "src/mục tiêu dự án robot phone/code micro-ROS esp32/robot_lowlevel_v2_3.ino",
)
HOST_ODOMETRY_FILE: Final = WORKSPACE_ROOT / "src/test_slam_nav2/scripts/motor_odom.py"
FIRMWARE_TPR_PATTERN: Final = re.compile(
    r"^\s*#define\s+ENCODER_TPR\s+([0-9]+(?:\.[0-9]+)?f?)\b",
    re.MULTILINE,
)
HOST_TPR_PATTERN: Final = re.compile(
    r"declare_parameter\(\s*['\"]ticks_per_rev['\"]\s*,\s*"
    r"([0-9]+(?:\.[0-9]+)?)\s*\)",
)


def _read_encoder_tpr(source_path: Path) -> float:
    source = source_path.read_text(encoding="utf-8")
    match = FIRMWARE_TPR_PATTERN.search(source)
    assert match is not None, f"ENCODER_TPR is missing from {source_path}"
    return float(match.group(1).removesuffix("f"))


def _read_host_default_tpr() -> float:
    source = HOST_ODOMETRY_FILE.read_text(encoding="utf-8")
    match = HOST_TPR_PATTERN.search(source)
    assert match is not None, f"ticks_per_rev default is missing from {HOST_ODOMETRY_FILE}"
    return float(match.group(1))


def test_active_firmware_and_host_encoder_tpr_are_consistent() -> None:
    # Given the confirmed active firmware sources and host odometry source.
    firmware_tprs = tuple(_read_encoder_tpr(path) for path in ACTIVE_FIRMWARE_FILES)

    # When their encoder ticks-per-revolution defaults are extracted.
    host_tpr = _read_host_default_tpr()

    # Then every producer uses the approved, shared encoder scale.
    assert firmware_tprs == (EXPECTED_ENCODER_TPR,) * len(ACTIVE_FIRMWARE_FILES)
    assert host_tpr == EXPECTED_ENCODER_TPR
