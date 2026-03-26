"""rb_estimator + rb_controller + rb_safety를 한 번에 띄우는 기본 launch.

기본값은 `config/safety.yaml`을 읽고, 필요하면 `params_file`로 scenario YAML을 덮어쓴다.
즉 tmux/스크립트 쪽에서는 이 launch 하나만 호출하면 제어 스택 전체를 재현할 수 있다.
"""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


def _launch_setup(context, *args, **kwargs):
    """launch 인자를 실제 값으로 풀고 estimator/controller/safety 노드를 구성한다."""
    params_file = LaunchConfiguration("params_file").perform(context)
    estimator_params_from_file = _load_params_for_node(params_file, "rb_estimator")
    controller_params_from_file = _load_params_for_node(params_file, "rb_controller")

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

    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            log_level_arg,
            enable_tilt_feedback_override_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )
