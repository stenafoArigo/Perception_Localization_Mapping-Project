from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _truthy(value: str) -> bool:
    return value.lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    gui = _truthy(LaunchConfiguration("gui").perform(context))
    use_model_names = _truthy(LaunchConfiguration("use_model_names").perform(context))
    base_watchdog_timeout = float(LaunchConfiguration("base_watchdog_timeout").perform(context))
    is_depth_canonical = _truthy(LaunchConfiguration("is_depth_canonical").perform(context))
    delay_odom_tf_by_one_update = _truthy(LaunchConfiguration("delay_odom_tf_by_one_update").perform(context))

    node_args = []
    if not gui:
        # Keep the classic ROS 1 stageros convention: -g disables the GUI.
        node_args.append("-g")
    if use_model_names:
        node_args.append("-u")
    node_args.append(world)

    return [
        Node(
            package="stage_ros2_stageros",
            executable="stageros",
            name="stageros",
            output="screen",
            arguments=node_args,
            parameters=[{
                "base_watchdog_timeout": base_watchdog_timeout,
                "is_depth_canonical": is_depth_canonical,
                "use_model_names": use_model_names,
                "delay_odom_tf_by_one_update": delay_odom_tf_by_one_update,
                "use_sim_time": True,
            }],
        )
    ]


def generate_launch_description():
    default_world = PathJoinSubstitution([
        FindPackageShare("stage_ros2_stageros"), "world", "maze.world"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world, description="Stage .world file"),
        DeclareLaunchArgument("gui", default_value="true", description="Start the Stage GUI"),
        DeclareLaunchArgument("use_model_names", default_value="false", description="Use Stage model names as topic/frame prefixes"),
        DeclareLaunchArgument("base_watchdog_timeout", default_value="0.2", description="Seconds before stale cmd_vel commands are zeroed; <=0 disables"),
        DeclareLaunchArgument("is_depth_canonical", default_value="true", description="Use REP-118 32FC1 depth image encoding"),
        DeclareLaunchArgument(
            "delay_odom_tf_by_one_update",
            default_value="true",
            description="Use previous Stage pose for odom/base TF to compensate one-update-delayed LaserScan data",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
