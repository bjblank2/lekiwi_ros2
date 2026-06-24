import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lekiwi_pkg = get_package_share_directory('lekiwi_ros2')

    urdf_file = os.path.join(lekiwi_pkg, 'urdf', 'lekiwi.urdf.xacro')
    controllers_file = os.path.join(lekiwi_pkg, 'config', 'lekiwi_controllers.yaml')

    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    use_joy = LaunchConfiguration('use_joy')
    leader_arm_id = LaunchConfiguration('leader_arm_id')

    declare_use_joy = DeclareLaunchArgument(
        'use_joy',
        default_value='true',
        description='Enable joystick input for base control',
    )

    declare_leader_arm_id = DeclareLaunchArgument(
        'leader_arm_id',
        default_value='leader_arm',
        description='Namespace of the leader arm (must match so101_leader.launch.py arm_id)',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file,
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    wheel_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller', '--controller-manager', '/controller_manager'],
    )

    # Start arm and wheel controllers after joint_state_broadcaster is active
    delay_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, wheel_velocity_controller_spawner],
        )
    )

    wheel_kinematics_node = Node(
        package='lekiwi_ros2',
        executable='wheel_kinematics_node',
        output='screen',
    )

    arm_teleop_node = Node(
        package='lekiwi_ros2',
        executable='arm_teleop_node',
        parameters=[{
            'leader_joint_states_topic': [leader_arm_id, '/joint_states'],
        }],
        output='screen',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        condition=IfCondition(use_joy),
    )

    joy_to_twist_node = Node(
        package='joy_to_twist',
        executable='joy_to_twist_node',
        condition=IfCondition(use_joy),
    )

    return LaunchDescription([
        declare_use_joy,
        declare_leader_arm_id,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_arm_after_jsb,
        wheel_kinematics_node,
        arm_teleop_node,
        joy_node,
        joy_to_twist_node,
    ])
