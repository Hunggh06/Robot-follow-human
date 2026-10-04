from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).parents[1]


def test_cmd_vel_bridge_uses_safe_incremental_speed_limits() -> None:
    path = PACKAGE_DIR / "scripts" / "cmd_vel_bridge.py"
    source = path.read_text()

    assert "MAX_RPM = 100.0" in source
    assert "ACCEL_LIM = 60.0" in source


def test_slam_nav_allows_reverse_at_point_one_meters_per_second() -> None:
    config_path = PACKAGE_DIR / "config" / "nav2_params_slam.yaml"
    config = yaml.safe_load(config_path.read_text())
    follow_path = config["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["min_vel_x"] == -0.10
