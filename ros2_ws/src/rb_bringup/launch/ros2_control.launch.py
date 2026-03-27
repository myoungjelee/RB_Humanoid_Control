"""G1 native ros2_control bringup.

현재 기본 active path는 native standing controller plugin 기준이다.
즉 controller_manager 아래 rb_standing_controller를 올리고,
RBHardwareSystem이 /rb/command_raw를 publish한 뒤 rb_safety를 거쳐
/rb/command_safe로 Isaac apply까지 연결하는 구성을 기본값으로 둔다.
legacy relay controller/bridge는 fallback 검증용으로만 남긴다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description() -> LaunchDescription:
    """control-only xacro를 읽어 ros2_control 최소 bringup을 실행한다."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    controller_manager_use_sim_time = LaunchConfiguration("controller_manager_use_sim_time")
    start_joint_state_broadcaster = LaunchConfiguration("start_joint_state_broadcaster")
    start_effort_forward_controller = LaunchConfiguration("start_effort_forward_controller")
    start_standing_controller = LaunchConfiguration("start_standing_controller")
    bridge_enabled = LaunchConfiguration("bridge_enabled")
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    command_topic = LaunchConfiguration("command_topic")
    controllers_file = LaunchConfiguration("controllers_file")
    enable_tilt_feedback = LaunchConfiguration("enable_tilt_feedback")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("rb_bringup"), "urdf", "g1_ros2_control.xacro"]
    )
    default_controllers_file = PathJoinSubstitution(
        [FindPackageShare("rb_bringup"), "config", "standing_controller_baseline.yaml"]
    )

    # xacro에서 control-only robot_description을 생성한다.
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " bridge_enabled:=",
                bridge_enabled,
                " joint_state_topic:=",
                joint_state_topic,
                " command_topic:=",
                command_topic,
            ]
        ),
        value_type=str,
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            # Humble 환경에서는 controller_manager가 sim time을 직접 물고 있을 때
            # load_controller 경로가 멈추는 현상이 있어, 여기서는 별도 arg로 분리한다.
            {"use_sim_time": controller_manager_use_sim_time},
            controllers_file,
            {
                "rb_standing_controller": {
                    "ros__parameters": {
                        # M8 pair처럼 동일 baseline에서 enable_tilt_feedback만 토글할 수 있게 한다.
                        "enable_tilt_feedback": ParameterValue(
                            enable_tilt_feedback, value_type=bool
                        ),
                    }
                }
            },
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
        ],
    )

    # joint_state_broadcaster는 native/legacy 경로와 무관하게 state 관측 기본값으로 유지한다.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=IfCondition(start_joint_state_broadcaster),
    )

    # legacy bridge 검증용 표준 relay controller.
    # 기본 active path는 custom standing plugin이므로 기본값은 false다.
    effort_forward_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["rb_effort_forward_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(start_effort_forward_controller),
    )

    standing_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["rb_standing_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(start_standing_controller),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock as ROS time source for robot_state_publisher and surrounding sim nodes.",
            ),
            DeclareLaunchArgument(
                "controller_manager_use_sim_time",
                default_value="false",
                description="Keep controller_manager on wall time by default. In current Humble setup, load_controller hangs when this is true.",
            ),
            DeclareLaunchArgument(
                "start_joint_state_broadcaster",
                default_value="true",
                description="Spawn joint_state_broadcaster during bringup.",
            ),
            DeclareLaunchArgument(
                "start_effort_forward_controller",
                default_value="false",
                description="Spawn the legacy relay controller for adapter-path testing only.",
            ),
            DeclareLaunchArgument(
                "start_standing_controller",
                default_value="true",
                description="Spawn the native rb_standing_controller plugin. Do not enable together with start_effort_forward_controller because both claim the same effort interfaces.",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=default_controllers_file,
                description="Controller parameter YAML loaded by ros2_control_node.",
            ),
            DeclareLaunchArgument(
                "enable_tilt_feedback",
                default_value="true",
                description="Override rb_standing_controller.enable_tilt_feedback without duplicating controller YAMLs.",
            ),
            DeclareLaunchArgument(
                "bridge_enabled",
                default_value="true",
                description="Enable /rb/joint_states read and command_topic write bridge inside RBHardwareSystem. This is on by default for the active native path.",
            ),
            DeclareLaunchArgument(
                "joint_state_topic",
                default_value="/rb/joint_states",
                description="JointState topic that RBHardwareSystem.read() mirrors into ros2_control state buffers.",
            ),
            DeclareLaunchArgument(
                "command_topic",
                default_value="/rb/command_raw",
                description="Topic that RBHardwareSystem.write() publishes effort commands to. Default /rb/command_raw keeps rb_safety in the active path before Isaac apply.",
            ),
            controller_manager_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            effort_forward_controller_spawner,
            standing_controller_spawner,
        ]
    )
