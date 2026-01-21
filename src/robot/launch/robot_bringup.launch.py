import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Định nghĩa tên Package
    description_pkg = 'my_arm_robot'
    controllers_pkg = 'my_robot_controllers'

    # 2. Lấy đường dẫn file
    # URDF (trong package robot)
    urdf_file = os.path.join(get_package_share_directory(description_pkg), 'urdf', 'robot.urdf')
    
    # Config (trong package my_robot_controllers)
    # Nếu bạn muốn để config trong package robot thì sửa đường dẫn lại tại đây
    controller_config = os.path.join(get_package_share_directory(controllers_pkg), 'config', 'controllers.yaml')

    # RViz config
    rviz_config = os.path.join(get_package_share_directory(description_pkg), 'rviz', 'default.rviz')

    # 3. Đọc nội dung URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # 4. Tạo các Node

    # A. Controller Manager (Core: Chạy Mock Hardware)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_desc},
            controller_config
        ],
        output="screen",
    )

    # B. Robot State Publisher (Tính toán TF, publish robot_description cho RViz)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_desc}],
    )

    # C. RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=['-d', rviz_config],
    )

    # D. Spawner: Joint State Broadcaster (Bắt buộc để có feedback vị trí)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # E. Spawner: Custom Controller của bạn
    # Lưu ý: Tên "my_cartesian_controller" phải khớp trong file controllers.yaml
    cartesian_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_cartesian_controller"],
    )
    # gripper_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["gripper_controller"],
    # )
    # 5. Xử lý thứ tự chạy (Delay)
    # Chạy cartesian_controller SAU KHI joint_state_broadcaster đã chạy xong
    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[cartesian_controller],
        )
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster,
        # gripper_controller_spawner,
        delay_controller,
    ])