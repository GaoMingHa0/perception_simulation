import numpy as np

from lidar_sim.lidar_simulator import (
    LidarConfig,
    LidarSimulator,
    angle_judge,
    distance_judge,
    front,
    lidar_to_world,
    load_track_yaml,
    normalize_cone,
    occlusion_judge,
    world_to_lidar,
)


def test_coordinate_round_trip():
    pose = [1.0, 2.0, 0.0, np.pi / 4.0]
    point = np.array([5.0, 3.0, 0.0])

    lidar_point = world_to_lidar(point, pose)
    assert np.allclose(lidar_to_world(lidar_point, pose), point)


def test_visibility_gates():
    origin = np.zeros(3)

    assert front(origin, 0.0, [5.0, 0.0, 0.0]) is True
    assert angle_judge(origin, 0.0, [5.0, 0.0, 0.0]) is True
    assert angle_judge(origin, 0.0, [0.0, 5.0, 0.0]) is True
    assert angle_judge(origin, 0.0, [0.0, 5.0, 0.0], fov_deg=120.0) is False
    assert distance_judge(origin, [5.0, 0.0, 0.0]) is True
    assert distance_judge(origin, [1.0, 0.0, 0.0]) is False


def test_cone_aabb_occlusion():
    origin = np.zeros(3)

    assert (
        occlusion_judge(
            origin,
            [5.0, 0.0, -2.0],
            [{"position": [2.5, 0.0, -1.1], "color": "yellow"}],
        )
        is True
    )
    assert (
        occlusion_judge(
            origin,
            [5.0, 0.0, -2.0],
            [{"position": [2.5, 0.2, -1.1], "color": "yellow"}],
        )
        is False
    )


def test_scan_is_reproducible():
    cones = [
        {"position": [5.0, 0.0, 0.0], "color": "yellow"},
        {"position": [8.0, 1.0, 0.0], "color": "blue"},
        {"position": [-5.0, 0.0, 0.0], "color": "red"},
    ]
    config = LidarConfig(ground_points_min=10, ground_points_max=10)

    scan1 = LidarSimulator(config, seed=7).simulate_scan(cones, [0.0, 0.0, 0.0])
    scan2 = LidarSimulator(config, seed=7).simulate_scan(cones, [0.0, 0.0, 0.0])

    assert len(scan1["visible_cones"]) == 3
    assert scan1["point_cloud"].shape[1] == 3
    assert np.allclose(scan1["point_cloud"], scan2["point_cloud"])


def test_load_track_yaml_directly():
    cones, start_pose = load_track_yaml("tracks/acceleration.yaml")

    assert start_pose == [-0.3, 0.0, 0.0, 0.0]
    assert len(cones) == 70
    assert cones[0]["color"] == "blue"


def test_skidpad_uses_four_cone_classes_and_sizes():
    cones, start_pose = load_track_yaml("tracks/skidpad.yaml")

    assert start_pose == [-15.0, 0.0, 0.0, 0.0]
    assert len(cones) == 66
    assert sum(cone["type"] == "small_blue" for cone in cones) == 29
    assert sum(cone["type"] == "small_red" for cone in cones) == 29
    assert sum(cone["type"] == "small_yellow" for cone in cones) == 4
    assert sum(cone["type"] == "large_yellow" for cone in cones) == 4

    high_yellow = normalize_cone(
        next(cone for cone in cones if cone["type"] == "large_yellow")
    )
    low_yellow = normalize_cone(
        next(cone for cone in cones if cone["type"] == "small_yellow")
    )
    assert high_yellow.color == low_yellow.color == "yellow"
    assert np.allclose(low_yellow.size, [0.20, 0.20, 0.30])
    assert np.allclose(high_yellow.size, [0.35, 0.35, 0.70])

    outer_arc_endpoints = [
        cone["position"]
        for cone in cones
        if cone["type"] in {"small_blue", "small_red"}
        and np.isclose(abs(cone["position"][0]), 7.399, atol=1e-3)
        and np.isclose(abs(cone["position"][1]), 1.5)
    ]
    assert len(outer_arc_endpoints) == 4
    for x, y, _ in outer_arc_endpoints:
        circle_center_y = 9.125 if y > 0.0 else -9.125
        assert np.isclose(x**2 + (y - circle_center_y) ** 2, 10.625**2, atol=2e-2)
