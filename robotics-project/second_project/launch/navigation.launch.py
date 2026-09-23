import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _truthy(value: str) -> bool:
    return value.lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    world                     = LaunchConfiguration("world").perform(context)
    gui                       = _truthy(LaunchConfiguration("gui").perform(context))
    use_model_names            = _truthy(LaunchConfiguration("use_model_names").perform(context))
    base_watchdog_timeout      = float(LaunchConfiguration("base_watchdog_timeout").perform(context))
    is_depth_canonical         = _truthy(LaunchConfiguration("is_depth_canonical").perform(context))
    delay_odom_tf_by_one_update = _truthy(LaunchConfiguration("delay_odom_tf_by_one_update").perform(context))

    # ── Stage ─────────────────────────────────────────────────────────────────
    stage_args = []
    if not gui:
        stage_args.append("-g")
    if use_model_names:
        stage_args.append("-u")
    stage_args.append(world)

    stage_node = Node(
        package="stage_ros2_stageros",
        executable="stageros",
        name="stageros",
        output="screen",
        arguments=stage_args,
        parameters=[{
            "base_watchdog_timeout":        base_watchdog_timeout,
            "is_depth_canonical":           is_depth_canonical,
            "use_model_names":              use_model_names,
            "delay_odom_tf_by_one_update":  delay_odom_tf_by_one_update,
            "use_sim_time": True,
        }],
    )

    pkg          = FindPackageShare("second_project")
    nav2_bringup = FindPackageShare("nav2_bringup")

    # Resolve to strings once — PathJoinSubstitution cannot be passed as a
    # params file path directly in all Nav2 Humble versions.
    map_yaml_str    = PathJoinSubstitution([pkg, "map",    "map.yaml"]).perform(context)
    nav2_params_str = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"]).perform(context)

    # ── Nav2 navigation stack ────────────────────────────────────────────────
    # Use navigation_launch.py (same as professor).
    # This file starts: controller_server, planner_server, bt_navigator,
    # behavior_server, waypoint_follower, velocity_smoother, lifecycle_manager_navigation.
    # It does NOT start map_server or amcl, and does NOT accept a 'map' argument.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup, "launch", "navigation_launch.py"]).perform(context)
        ),
        launch_arguments={
            "use_sim_time":    "true",
            "params_file":     nav2_params_str,
            "autostart":       "true",
            "use_composition": "False",
            "use_respawn":     "False",
        }.items(),
    )

    # ── Localization (map_server + amcl + their lifecycle manager) ───────────
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            nav2_params_str,
            {
                "use_sim_time":  True,
                "yaml_filename": map_yaml_str,
            },
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            nav2_params_str,
            {
                "use_sim_time":              True,
                "scan_topic":                LaunchConfiguration("scan_topic"),
                "base_frame_id":             LaunchConfiguration("base_frame"),
                "odom_frame_id":             LaunchConfiguration("odom_frame"),
                "global_frame_id":           LaunchConfiguration("map_frame"),
                "set_initial_pose":          ParameterValue(LaunchConfiguration("set_initial_pose"), value_type=bool),
                "always_reset_initial_pose": ParameterValue(LaunchConfiguration("set_initial_pose"), value_type=bool),
                "initial_pose": {
                    "x":   ParameterValue(LaunchConfiguration("initial_x"),   value_type=float),
                    "y":   ParameterValue(LaunchConfiguration("initial_y"),   value_type=float),
                    "z":   0.0,
                    "yaw": ParameterValue(LaunchConfiguration("initial_yaw"), value_type=float),
                },
            },
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    lifecycle_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart":    True,
            "node_names":   ["map_server", "amcl"],
        }],
    )

    # ── Goal publisher ───────────────
    goals_csv = PathJoinSubstitution([pkg, "csv", "goals.csv"]).perform(context)

    goal_pub = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="second_project",
                executable="goal_publisher",
                name="goal_publisher",
                output="screen",
                parameters=[{
                    "csv_path":     goals_csv,
                    "use_sim_time": True,
                }],
            )
        ],
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([pkg, "rviz", "navigation.rviz"]).perform(context)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    return [
        stage_node,
        map_server,
        amcl,
        lifecycle_localization,
        nav2,
        goal_pub,
        rviz,
    ]


def generate_launch_description():
    pkg = FindPackageShare("second_project")

    default_world = PathJoinSubstitution([pkg, "config", "robot.world"])

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),

        DeclareLaunchArgument("world",                       default_value=default_world),
        DeclareLaunchArgument("gui",                         default_value="true"),
        DeclareLaunchArgument("use_model_names",             default_value="false"),
        DeclareLaunchArgument("base_watchdog_timeout",       default_value="0.2"),
        DeclareLaunchArgument("is_depth_canonical",          default_value="true"),
        DeclareLaunchArgument("delay_odom_tf_by_one_update", default_value="true"),

        DeclareLaunchArgument("scan_topic",     default_value="/base_scan"),
        DeclareLaunchArgument("odom_frame",     default_value="odom"),
        DeclareLaunchArgument("map_frame",      default_value="map"),
        DeclareLaunchArgument("base_frame",     default_value="base_footprint"),
        DeclareLaunchArgument("set_initial_pose", default_value="true"),

        DeclareLaunchArgument("initial_x",   default_value="0.0"),
        DeclareLaunchArgument("initial_y",   default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),

        OpaqueFunction(function=_launch_setup),
    ])
