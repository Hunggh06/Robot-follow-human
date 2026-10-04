from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).parents[1]


def test_cmd_vel_bridge_uses_safe_incremental_speed_limits() -> None:
    path = PACKAGE_DIR / "scripts" / "cmd_vel_bridge.py"
    source = path.read_text()

    assert "MAX_RPM = 126.0" in source
    assert "ACCEL_LIM = 60.0" in source


def test_slam_nav_disables_reverse_motion() -> None:
    config_path = PACKAGE_DIR / "config" / "nav2_params_slam.yaml"
    config = yaml.safe_load(config_path.read_text())
    follow_path = config["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["min_vel_x"] == -0.10
    assert follow_path["max_vel_x"] == 0.43
    assert follow_path["max_speed_xy"] == 0.43
    assert follow_path["max_vel_theta"] == 0.4
    assert follow_path["acc_lim_theta"] == 1.0
    assert follow_path["decel_lim_theta"] == -1.0
    assert follow_path["min_speed_theta"] == 0.05
    assert follow_path["vtheta_samples"] == 12
    assert follow_path["sim_time"] == 1.5
    assert follow_path["RotateToGoal.slowing_factor"] == 3.0
    assert follow_path["PathAlign.scale"] == 8.0
    assert follow_path["GoalAlign.scale"] == 24.0

    smoother = config["velocity_smoother"]["ros__parameters"]
    assert smoother["scale_velocities"] is False

    assert smoother["max_velocity"][0] == 0.43
    assert smoother["min_velocity"][0] == -0.10
    assert smoother["max_accel"][2] == 3.2
    assert smoother["max_decel"][2] == -3.2
    assert smoother["max_velocity"][2] == 1.0
    assert smoother["min_velocity"][2] == -1.0
