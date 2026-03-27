"""rb_estimation + rb_safety(+ optional legacy controller/bridge) launch.

현재 기본 active path는 native ros2_control plugin이므로,
이 launch의 기본값은 `rb_estimator + rb_safety`만 올린다.
legacy `rb_controller_node`와 `rb_command_bridge_node`는 fallback 검증용이라
명시적으로 켰을 때만 올라온다.
"""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_params_for_node(params_file: str, node_name: str) -> dict:
    """YAML에서 특정 노드의 ros__parameters 블록만 안전하게 읽어온다."""
    try:
        loaded = yaml.safe_load(Path(params_file).read_text()) or {}
    except OSError:
        return {}
    except yaml.YAMLError:
        return {}

    node_block = loaded.get(node_name, {})
    ros_params = node_block.get("ros__parameters", {})
    return ros_params if isinstance(ros_params, dict) else {}


def _load_ros2_control_joint_names(controllers_file: str) -> list[str]:
    """standing controller baseline YAML에서 forward controller joint 순서를 읽어온다."""
    try:
        loaded = yaml.safe_load(Path(controllers_file).read_text()) or {}
    except OSError:
        return []
    except yaml.YAMLError:
        return []

    controller_block = loaded.get("rb_effort_forward_controller", {})
    ros_params = controller_block.get("ros__parameters", {})
    joints = ros_params.get("joints", [])
    if not isinstance(joints, list):
        return []
    return [str(name) for name in joints]


def _launch_setup(context, *args, **kwargs):
    """launch 인자를 실제 값으로 풀고 estimator/controller/safety 노드를 구성한다."""
    params_file = LaunchConfiguration("params_file").perform(context)
    enable_command_bridge = LaunchConfiguration("enable_command_bridge").perform(context)
    start_controller = LaunchConfiguration("start_controller")
    command_bridge_input_topic = LaunchConfiguration("command_bridge_input_topic").perform(context)
    command_bridge_output_topic = LaunchConfiguration("command_bridge_output_topic").perform(context)
    ros2_control_controllers_file = LaunchConfiguration("ros2_control_controllers_file").perform(context)
    estimator_params_from_file = _load_params_for_node(params_file, "rb_estimator")
    controller_params_from_file = _load_params_for_node(params_file, "rb_controller")
    ros2_control_joint_names = _load_ros2_control_joint_names(ros2_control_controllers_file)

    estimator_fallback_parameters = {}
    if not estimator_params_from_file:
        # scenario YAML에 rb_estimator 블록이 없더라도 controller 쪽 토픽 설정은 그대로 따라가게 한다.
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
        }

    estimator_node = Node(
        package="rb_estimation",
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
    # M8에서는 같은 base YAML을 쓰고 tilt feedback만 ON/OFF override 하므로 launch에서 한 번 더 훅을 둔다.
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
        condition=IfCondition(start_controller),
        parameters=controller_parameters,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    safety_node = Node(
        package="rb_safety",
        executable="rb_safety_node",
        name="rb_safety",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            (
                {"output_command_topic": command_bridge_input_topic}
                if enable_command_bridge == "true"
                else {}
            ),
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    nodes = [estimator_node, controller_node, safety_node]
    if enable_command_bridge == "true":
        command_bridge_node = Node(
            package="rb_controller",
            executable="rb_command_bridge_node",
            name="rb_command_bridge",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"input_command_topic": command_bridge_input_topic},
                {"output_command_topic": command_bridge_output_topic},
                {"target_joint_names": ros2_control_joint_names},
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        )
        nodes.append(command_bridge_node)

    return nodes


def generate_launch_description() -> LaunchDescription:
    """launch 인자 선언과 노드 구성을 묶어 최종 LaunchDescription을 반환한다."""
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
    enable_command_bridge_arg = DeclareLaunchArgument(
        "enable_command_bridge",
        default_value="false",
        description=(
            "Enable the legacy adapter path only when you want "
            "rb_safety -> rb_command_bridge -> rb_effort_forward_controller."
        ),
    )
    start_controller_arg = DeclareLaunchArgument(
        "start_controller",
        default_value="false",
        description=(
            "Start legacy rb_controller_node. Default is false because the native "
            "rb_standing_controller plugin is now the active path."
        ),
    )
    command_bridge_input_topic_arg = DeclareLaunchArgument(
        "command_bridge_input_topic",
        default_value="/rb/command_safe_source",
        description="Intermediate JointState topic produced by rb_safety before ros2_control conversion.",
    )
    command_bridge_output_topic_arg = DeclareLaunchArgument(
        "command_bridge_output_topic",
        default_value="/rb_effort_forward_controller/commands",
        description="Float64MultiArray topic consumed by rb_effort_forward_controller.",
    )
    ros2_control_controllers_file_arg = DeclareLaunchArgument(
        "ros2_control_controllers_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rb_bringup"), "config", "standing_controller_baseline.yaml"]
        ),
        description="Path to ros2_control controller YAML used to load target joint order for rb_command_bridge.",
    )

    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            log_level_arg,
            enable_tilt_feedback_override_arg,
            enable_command_bridge_arg,
            start_controller_arg,
            command_bridge_input_topic_arg,
            command_bridge_output_topic_arg,
            ros2_control_controllers_file_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )
