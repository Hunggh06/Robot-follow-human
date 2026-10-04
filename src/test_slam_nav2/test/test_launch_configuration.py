import importlib.util
from pathlib import Path
from types import ModuleType

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


PACKAGE_DIR = Path(__file__).parents[1]


def _load_launch_module(filename: str) -> ModuleType:
    path = PACKAGE_DIR / "launch" / filename
    spec = importlib.util.spec_from_file_location(filename.removesuffix(".py"), path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _launch_nodes(description: LaunchDescription) -> list[Node]:
    return [entity for entity in description.entities if isinstance(entity, Node)]


def _find_node(
    nodes: list[Node],
    package: str,
    executable: str,
    context: str,
) -> Node:
    matches = [
        node
        for node in nodes
        if node.__dict__["_Node__package"] == package
        and node.__dict__["_Node__node_executable"] == executable
    ]
    assert len(matches) == 1, (
        f"{context}: expected one {package}/{executable}, found {len(matches)}"
    )
    return matches[0]


def _node_parameter_dict(node: Node) -> dict[str, bool | float | int | str]:
    parameters: dict[str, bool | float | int | str] = {}
    for parameter_group in node.__dict__["_Node__parameters"]:
        if not isinstance(parameter_group, dict):
            continue
        for key, value in parameter_group.items():
            if isinstance(value, bool):
                parameters[key[0].text] = value
                continue
            if isinstance(value, (float, int)):
                parameters[key[0].text] = value
                continue
            substitution = value[0]
            if isinstance(substitution, list):
                substitution = substitution[0]
            parsed_value = yaml.safe_load(substitution.text)
            assert isinstance(parsed_value, (bool, float, int, str))
            parameters[key[0].text] = parsed_value
    return parameters


def _find_static_lidar_transform(nodes: list[Node], context: str) -> Node:
    matches = [
        node
        for node in nodes
        if node.__dict__["_Node__package"] == "tf2_ros"
        and node.__dict__["_Node__node_executable"] == "static_transform_publisher"
    ]
    assert len(matches) == 1, (
        f"{context}: expected one static lidar transform, found {len(matches)}"
    )
    return matches[0]


def test_ekf_uses_only_yaw_rate_from_remapped_imu() -> None:
    config_path = PACKAGE_DIR / "config" / "ekf.yaml"
    config = yaml.safe_load(config_path.read_text())
    parameters = config["ekf_filter_node"]["ros__parameters"]

    assert parameters["imu0"] == "/imu/yaw"
    assert parameters["imu0_config"] == [
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        True,
        False,
        False,
        False,
    ]
    assert parameters["publish_tf"] is False
    assert parameters["odom0"] == "/wheel/odom"


def test_both_live_slam_launches_start_remap_and_ekf() -> None:
    for filename in ("slam.launch.py", "slam_nav.launch.py"):
        module = _load_launch_module(filename)
        nodes = _launch_nodes(module.generate_launch_description())

        remap = _find_node(nodes, "test_slam_nav2", "imu_remap.py", filename)
        remap_parameters = _node_parameter_dict(remap)
        assert remap_parameters["use_sim_time"] is False, filename
        _find_node(nodes, "robot_localization", "ekf_node", filename)


def test_both_live_slam_launches_keep_filtered_host_timestamped_scan() -> None:
    for filename in ("slam.launch.py", "slam_nav.launch.py"):
        module = _load_launch_module(filename)
        nodes = _launch_nodes(module.generate_launch_description())
        telemetry = _find_node(nodes, "kaiaai_telemetry", "telem", filename)
        parameters = _node_parameter_dict(telemetry)

        assert parameters["laser_scan.topic_name_pub"] == "scan", filename
        assert parameters["laser_scan.frame_id"] == "laser", filename
        assert parameters["lidar.clockwise"] is False, filename
        assert parameters["use_host_time_stamp"] is True, filename
        assert parameters["laser_scan.discard_broken_scans"] is True, filename
        assert parameters["laser_scan.mask_radius_meters"] == 0.1, filename
        assert parameters["lidar.range_min_meters"] == 0.15, filename
        assert parameters["lidar.range_max_meters"] == 6.0, filename

        transform = _find_static_lidar_transform(nodes, filename)
        assert transform.__dict__["_Node__arguments"][-2:] == [
            "lidar_link",
            "laser",
        ], filename


def test_live_slam_launches_use_matching_laser_range_configuration() -> None:
    config_path = PACKAGE_DIR / "config" / "slam_toolbox.yaml"
    config = yaml.safe_load(config_path.read_text())
    parameters = config["slam_toolbox"]["ros__parameters"]

    assert parameters["scan_topic"] == "/scan"
    assert parameters["base_frame"] == "base_link"
    assert parameters["odom_frame"] == "odom"
    assert parameters["map_frame"] == "map"
    assert parameters["min_laser_range"] == 0.15
    assert parameters["max_laser_range"] == 6.0


def test_slam_nav_bridge_reads_filtered_odometry_without_scan_relay() -> None:
    module = _load_launch_module("slam_nav.launch.py")
    nodes = _launch_nodes(module.generate_launch_description())
    bridge = _find_node(
        nodes,
        "test_slam_nav2",
        "odom_tf_bridge.py",
        "slam_nav.launch.py",
    )
    parameters = _node_parameter_dict(bridge)

    assert parameters["odom_topic"] == "/odometry/filtered"
    assert parameters["relay_scan"] is False


def test_saved_map_navigation_keeps_imu_remap() -> None:
    module = _load_launch_module("navigation.launch.py")
    nodes = _launch_nodes(module.generate_launch_description())

    remap = _find_node(
        nodes,
        "test_slam_nav2",
        "imu_remap.py",
        "navigation.launch.py",
    )
    assert _node_parameter_dict(remap)["use_sim_time"] is False

