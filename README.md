# track_loader

## ROS2 LiDAR simulator

This repository now contains a Python ROS2 package named `lidar_sim`.
Design notes are collected in `docs/`.

### Ubuntu / ROS2 build

Use this repository as a ROS2 workspace root, then run:

```bash
cd ~/track_loader
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select lidar_sim
source install/setup.bash
```

Quick core check without ROS:

```bash
python3 -m pytest tests/test_lidar_core.py
```

### Run

```bash
ros2 launch lidar_sim lidar_simulator.launch.py
```

Default behavior:

- subscribes: `/sim/ground_truth` (`nav_msgs/msg/Odometry`)
- publishes: `/hesai/pandar` (`sensor_msgs/msg/PointCloud2`)
- publishes debug markers: `/sim/lidar/visible_cones` (`visualization_msgs/msg/MarkerArray`);
  these sensor-frame markers use stamp zero so RViz resolves the latest TF, while
  `/hesai/pandar` keeps the ground-truth acquisition timestamp for perception.
- publishes the complete ground-truth map loaded directly from YAML: `/sim/lidar/track_cones`
  (`visualization_msgs/msg/MarkerArray`, `map` frame).  It is transient-local,
  so RViz can display it even when opened after the simulator.  Each cone uses
  its YAML color and physical size, with a label containing its ID, type and
  dimensions.  This topic is independent of `/mapping/cone_map_viz`.
  It is published once at node startup and remains available independently of
  LiDAR scans, odometry, or vehicle motion.
- loads track YAML directly from: `share/lidar_sim/tracks/trackdrive.yaml`

Override the track file:

```bash
ros2 launch lidar_sim lidar_simulator.launch.py track_file:=skidpad
ros2 launch lidar_sim lidar_simulator.launch.py track_file:=acceleration
ros2 launch lidar_sim lidar_simulator.launch.py track_file:=/absolute/path/to/custom_track.yaml
```

Track YAML files live in `tracks/`; there is no separate `track_loader` layer.

Core-only demo:

```bash
ros2 run lidar_sim lidar_simulator_demo
```
