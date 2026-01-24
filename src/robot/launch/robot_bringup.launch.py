import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # =========================================================
    # 1. Launch Arguments
    # =========================================================
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="True: Run mock hardware. False: Run real robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for real robot hardware.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Baudrate for serial communication.",
        )
    )
    declared_arguments.append(DeclareLaunchArgument("launch_gui", default_value="true"))
    gui_pkg_path = os.path.join(
        get_package_share_directory('robot_gui') # Thay 'robot_gui' bằng tên package thật của bạn
    )
    gui_script_path = os.path.join(gui_pkg_path, 'main.py')
    # =========================================================
    # 2. XACRO -> robot_description
    # =========================================================
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("my_arm_robot"), "urdf", "robot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=", LaunchConfiguration("use_mock_hardware"),
            " ",
            "serial_port:=", LaunchConfiguration("serial_port"),
            " ",
            "baud_rate:=", LaunchConfiguration("baud_rate"), # THÊM DÒNG NÀY
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    # =========================================================
    # 3. Controller configuration
    # =========================================================
    controller_config = PathJoinSubstitution(
        [
            FindPackageShare("my_robot_controllers"),
            "config",
            "controllers.yaml",
        ]
    )

    # =========================================================
    # 4. RViz config
    # =========================================================
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("my_arm_robot"),
            "rviz",
            "default.rviz",
        ]
    )

    # =========================================================
    # 5. Nodes
    # =========================================================

    # A. ros2_control (Controller Manager)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controller_config,
        ],
    )

    # B. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # C. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    # D. GUI Python Process
    gui_node = ExecuteProcess(
        cmd=['python3', gui_script_path],
        cwd=gui_pkg_path, # Rất quan trọng để tìm thấy các module tabs, shm_manager
        output='screen',
        condition=None # Có thể thêm logic IfCondition(LaunchConfiguration("launch_gui"))
    )
    # =========================================================
    # 6. Controller Spawners
    # =========================================================

    # Joint State Broadcaster (BẮT BUỘC)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Custom Cartesian Controller
    cartesian_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_cartesian_controller"],
        output="screen",
    )

    # =========================================================
    # 7. Controller startup order
    # =========================================================
    delay_cartesian_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[cartesian_controller_spawner],
        )
    )

    # =========================================================
    # 8. Launch Description
    # =========================================================
    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_publisher_node,
            rviz_node,
            joint_state_broadcaster_spawner,
            delay_cartesian_controller,
            gui_node,
        ]
    )
