# stage_ros2_stageros

A compact ROS 2 compatibility port of the classic ROS 1 `stage_ros/stageros` bridge.

It keeps the original bridge shape:

- subscribes: `cmd_vel`
- publishes: `odom`, `base_pose_ground_truth`, `base_scan`, `/clock`
- publishes camera topics when Stage camera models are present: `image`, `depth`, `camera_info`
- broadcasts TF: `odom -> base_footprint -> base_link`, plus laser/camera frames
- supports multiple Stage `position` models by prefixing topics/frames with `robot_N/`
- exposes `reset_positions` as `std_srvs/srv/Empty`

## Important caveat

This is a source-level port intended as a starting point. I could not compile it in this environment because ROS 2 and libstage development headers are not installed here. Expect small API fixes depending on the exact Stage fork/version you build against.

There is also an existing maintained ROS 2 bridge, `tuw-robotics/stage_ros2`, which may be a better choice if you need multi-robot and Ackermann support immediately.

## Build

Put the package in a ROS 2 workspace:

```bash
mkdir -p ~/stage_ws/src
cp -r stage_ros2_stageros ~/stage_ws/src/
cd ~/stage_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select stage_ros2_stageros
source install/setup.bash
```

You need a working libstage installation that provides either a CMake `stage` package or a `pkg-config` module named `stage`.

## Run

```bash
ros2 launch stage_ros2_stageros stage.launch.py
```

Run headless, preserving the old ROS 1 `-g` behavior:

```bash
ros2 launch stage_ros2_stageros stage.launch.py gui:=false
```

Run with your own world:

```bash
ros2 launch stage_ros2_stageros stage.launch.py world:=/absolute/path/to/my.world
```

Drive the robot:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.4}}"
```

For multiple robots, topics become `robot_0/cmd_vel`, `robot_0/base_scan`, `robot_1/cmd_vel`, and so on.

## Parameters

- `base_watchdog_timeout` default `0.2`: if no `cmd_vel` arrives before this simulated-time timeout, robot speeds are set to zero. Use `0.0` to disable.
- `is_depth_canonical` default `true`: publishes depth as REP-118-style `32FC1`; set false for `16UC1` millimeters.
- `use_model_names` default `false`: use Stage model tokens instead of `robot_N` prefixes when possible.
- `delay_odom_tf_by_one_update` default `true`: publishes `odom -> base_footprint` and `/odom` using the previous Stage pose. This is useful when the Stage ranger data is one world update behind the position model pose, which otherwise makes the laser appear rotated by one command/update step during rotation. Disable it with `delay_odom_tf_by_one_update:=false` for comparison.

For SLAM demos with `slam_toolbox`, keep `delay_odom_tf_by_one_update:=true` if the laser appears to rotate slightly in RViz while the fixed frame is `odom`.

## License

GPL-2.0-or-later, because this is a derivative/compatibility port of the classic `stageros` implementation.
