from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_obstacle_avoidance = LaunchConfiguration("use_obstacle_avoidance")
    use_boost_acceleration = LaunchConfiguration("use_boost_acceleration")
    use_stats = LaunchConfiguration("use_stats")
    config_package = LaunchConfiguration("config_package").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    ref_vel_file = LaunchConfiguration("ref_vel_file").perform(context)

    config_path = (
        Path(get_package_share_directory(config_package))
        / config_file
    )
    ref_vel_path = (
        Path(get_package_share_directory(config_package))
        / ref_vel_file
    )

    mpc_controller = Node(
        package="multi_purpose_mpc_ros_cpp",
        executable="mpc_controller_cpp",
        name="mpc_controller",
        output="both",
        emulate_tty=True,
        sigterm_timeout="10",
        parameters=[
            {"config_path": str(config_path)},
            {"ref_vel_path": str(ref_vel_path)},
            {"use_boost_acceleration": use_boost_acceleration},
            {"use_obstacle_avoidance": use_obstacle_avoidance},
            {"use_stats": use_stats},
        ],
    )

    boost_commander = Node(
        package="multi_purpose_mpc_ros",
        executable="boost_commander",
        name="boost_commander",
        output="both",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", "info"],
        condition=IfCondition(use_boost_acceleration),
    )

    path_constraints_provider = Node(
        package="multi_purpose_mpc_ros",
        executable="path_constraints_provider.bash",
        name="path_constraints_provider",
        output="both",
        emulate_tty=True,
        arguments=[
            "--config_path",
            str(config_path),
            "--ros-args",
            "--log-level",
            "info",
        ],
        parameters=[
            {"use_boost_acceleration": use_boost_acceleration},
            {"use_obstacle_avoidance": use_obstacle_avoidance},
        ],
        condition=IfCondition(use_obstacle_avoidance),
    )

    return [
        SetParameter("use_sim_time", use_sim_time),
        mpc_controller,
        boost_commander,
        path_constraints_provider,
    ]


def generate_launch_description():
    arg_configs = [
        ("use_sim_time", "true", "Use simulation time or not"),
        (
            "config_package",
            "multi_purpose_mpc_ros",
            "Package that owns config/config.yaml and config/ref_vel.yaml",
        ),
        (
            "config_file",
            "config/config.yaml",
            "Config path relative to config_package share directory",
        ),
        (
            "ref_vel_file",
            "config/ref_vel.yaml",
            "Reference velocity config path relative to config_package share directory",
        ),
        (
            "use_boost_acceleration",
            "false",
            "Use the boost acceleration for AWSIM simulation",
        ),
        (
            "use_obstacle_avoidance",
            "false",
            "Use the functionality of obstacle avoidance",
        ),
        ("use_stats", "false", "Use the execution statistics"),
    ]

    declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in arg_configs
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
