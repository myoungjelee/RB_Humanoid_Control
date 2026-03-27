"""Native estimator + safety stack launch.

active path에서 rb_controller 패키지 의존을 끊기 위해,
rb_bringup이 estimator/safety launch와 기본 YAML 소유권을 가진다.
legacy controller/bridge는 rb_controller 안에 남겨 두고,
이 launch는 native plugin 경로에 필요한 노드만 올린다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """rb_estimator + rb_safety 노드를 native stack 기본값으로 띄운다."""
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    default_params_file = PathJoinSubstitution(
        [FindPackageShare("rb_bringup"), "config", "m7_stack_safety_on.yaml"]
    )

    estimator_node = Node(
        package="rb_estimation",
        executable="rb_estimator_node",
        name="rb_estimator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    safety_node = Node(
        package="rb_safety",
        executable="rb_safety_node",
        name="rb_safety",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to the native estimator/safety stack parameter YAML.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock as ROS time source.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level for rb_estimator and rb_safety.",
            ),
            estimator_node,
            safety_node,
        ]
    )
