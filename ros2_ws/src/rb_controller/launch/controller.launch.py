"""Launch rb_controller + rb_safety with a default safety parameter file.

This launch file provides one-line startup for controller + safety and loads
`config/safety.yaml` by default. Operators can still override `params_file`.
"""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_params_for_node(params_file: str, node_name: str) -> dict:
    try:
        loaded = yaml.safe_load(Path(params_file).read_text()) or {}
    except OSError:
        return {}
    except yaml.YAMLError:
        return {}

    node_block = loaded.get(node_name, {})
    ros_params = node_block.get("ros__parameters", {})
    return ros_params if isinstance(ros_params, dict) else {}


def _launch_setup(context, *args, **kwargs):
    """Build nodes after resolving optional controller-only parameter overrides."""
    params_file = LaunchConfiguration("params_file").perform(context)
    estimator_params_from_file = _load_params_for_node(params_file, "rb_estimator")
    controller_params_from_file = _load_params_for_node(params_file, "rb_controller")

    estimator_fallback_parameters = {}
    if not estimator_params_from_file:
        estimator_fallback_parameters = {
            "input_joint_states_topic": controller_params_from_file.get(
                "input_joint_states_topic", "/rb/joint_states"
            ),
            "input_imu_topic": controller_params_from_file.get(
                "input_imu_topic", "/rb/imu"
            ),
            "output_estimated_state_topic": controller_params_from_file.get(
                "output_estimated_state_topic", "/rb/estimated_state"
            ),
            "imu_frame_mode": controller_params_from_file.get(
                "imu_frame_mode",
                controller_params_from_file.get("tilt_axis_mode", "identity"),
            ),
        }

    estimator_node = Node(
        package="rb_controller",
        executable="rb_estimator_node",
        name="rb_estimator",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            estimator_fallback_parameters,
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    controller_parameters = [
        LaunchConfiguration("params_file"),
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]
    controller_override = LaunchConfiguration(
        "enable_tilt_feedback_override"
    ).perform(context)
    if controller_override in ("true", "false"):
        controller_parameters.append(
            {"enable_tilt_feedback": controller_override == "true"}
        )

    controller_node = Node(
        package="rb_controller",
        executable="rb_controller_node",
        name="rb_controller",
        output="screen",
        parameters=controller_parameters,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    safety_node = Node(
        package="rb_controller",
        executable="rb_safety_node",
        name="rb_safety",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    return [estimator_node, controller_node, safety_node]


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for rb_estimator + rb_controller + rb_safety."""
    # 패키지 설치 경로 기준 기본 파라미터 파일(안전 설정)을 찾는다.
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("rb_controller"), "config", "safety.yaml"]
    )

    # params_file: 필요 시 외부 YAML로 교체할 수 있도록 launch 인자로 노출한다.
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the controller parameter YAML file.",
    )
    # use_sim_time: Isaac의 /clock을 ROS 시간 기준으로 사용할지 선택한다.
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock as ROS time source.",
    )
    # log_level: 디버깅 시 verbosity를 런타임에서 조절하기 위한 인자다.
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for rb_controller node.",
    )
    enable_tilt_feedback_override_arg = DeclareLaunchArgument(
        "enable_tilt_feedback_override",
        default_value="keep",
        description=(
            "Optional controller-only override for enable_tilt_feedback. "
            "Use true/false to force M8 OFF/ON comparison, or keep to use YAML."
        ),
    )

    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            log_level_arg,
            enable_tilt_feedback_override_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )
