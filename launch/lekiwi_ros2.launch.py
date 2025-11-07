from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Serial port (shared by all servos)
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for unified servo bus'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='1000000',
            description='Serial communication baudrate'
        ),
        
        # Arm parameters
        DeclareLaunchArgument(
            'arm_id',
            default_value='follower_arm',
            description='Arm ID for namespace'
        ),
        DeclareLaunchArgument(
            'leader_arm_id',
            default_value='leader_arm',
            description='leader arm ID to follow'
        ),
        DeclareLaunchArgument(
            'max_relative_target',
            default_value='20.0',
            description='Maximum relative target movement (degrees)'
        ),
        DeclareLaunchArgument(
            'calibration_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('lekiwi_ros2'),
                'params',
                'lekiwi_so101_calibration.yaml',
            ]),
            description='ROS 2 parameter file that defines follower arm calibration data'
        ),
        
        # Base parameters - joystick mapping
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='1.0',
            description='Maximum linear speed (m/s)'
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='2.0',
            description='Maximum angular speed (rad/s)'
        ),
        DeclareLaunchArgument(
            'axis_linear_x',
            default_value='1',
            description='Joystick axis for linear X (forward/back)'
        ),
        DeclareLaunchArgument(
            'axis_linear_y',
            default_value='0',
            description='Joystick axis for linear Y (strafe)'
        ),
        DeclareLaunchArgument(
            'axis_angular_z',
            default_value='3',
            description='Joystick axis for angular Z (rotation)'
        ),
        
        # Base parameters - wheel geometry
        DeclareLaunchArgument(
            'wheel_separation_x',
            default_value='0.3',
            description='Wheel separation in X direction (m)'
        ),
        DeclareLaunchArgument(
            'wheel_separation_y',
            default_value='0.3',
            description='Wheel separation in Y direction (m)'
        ),
        DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.05',
            description='Wheel radius (m)'
        ),
        DeclareLaunchArgument(
            'wheel_count',
            default_value='3',
            description='Number of wheels (3 for omni, 4 for mecanum)'
        ),
        DeclareLaunchArgument(
            'ticks_per_rev',
            default_value='4096',
            description='Encoder ticks per revolution'
        ),
        
        Node(
            package='lekiwi_ros2',
            executable='lekiwi_ros2',
            name='lekiwi_ros2_node',
            parameters=[
                LaunchConfiguration('calibration_params'),
                {
                    # Serial port
                    'port': LaunchConfiguration('port'),
                    'baudrate': LaunchConfiguration('baudrate'),

                    # Arm parameters
                    'arm_id': LaunchConfiguration('arm_id'),
                    'leader_arm_id': LaunchConfiguration('leader_arm_id'),
                    'max_relative_target': LaunchConfiguration('max_relative_target'),

                    # Base parameters
                    'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                    'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                    'axis_linear_x': LaunchConfiguration('axis_linear_x'),
                    'axis_linear_y': LaunchConfiguration('axis_linear_y'),
                    'axis_angular_z': LaunchConfiguration('axis_angular_z'),
                    'wheel_separation_x': LaunchConfiguration('wheel_separation_x'),
                    'wheel_separation_y': LaunchConfiguration('wheel_separation_y'),
                    'wheel_radius': LaunchConfiguration('wheel_radius'),
                    'wheel_count': LaunchConfiguration('wheel_count'),
                    'ticks_per_rev': LaunchConfiguration('ticks_per_rev'),
                },
            ],
            output='screen'
        ),
    ])

