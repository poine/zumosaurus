from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    joy_config_filepath = LaunchConfiguration('joy_config_filepath')
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('joy_config', default_value='ps3'))
    declared_arguments.append(DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'))
    declared_arguments.append(
        DeclareLaunchArgument('joy_config_filepath', default_value=PathJoinSubstitution([FindPackageShare('zumosaurus_bringup'), "config", "teleop_twist_joy_MOCUTE.config.yaml"])))
        
    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }])
        
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name='teleop_twist_joy_node',
        parameters=[joy_config_filepath, {'require_enable_button':False}, {'axis_angular.yaw':2}],
        remappings=[("/cmd_vel", "/zumosaurus_base_controller/cmd_vel_unstamped")]
    )


    return LaunchDescription(declared_arguments + [joy_node, teleop_node])
