from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'padwq_description'
    pkg_share_dir = FindPackageShare(pkg_name) 

    default_robot_name = 'padwq'

    # declare launch arguments
    prefix_arg = DeclareLaunchArgument(
            name='prefix',
            default_value='""',
            description='',
    )

    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        default_value=default_robot_name,
        description='absolute path to the robot urdf file'
    )

    # initialize launch arguments
    prefix = LaunchConfiguration('prefix')
    robot_name = LaunchConfiguration('robot_name')
    
    # define nodes
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([pkg_share_dir, 'urdf', robot_name]),
        '.urdf.xacro',
        " ",
        "prefix:=",
        prefix,
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_config_file = [
            PathJoinSubstitution([pkg_share_dir, 'config', robot_name]),
            '.rviz'
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    ld = LaunchDescription([
        prefix_arg,
        robot_name_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])

    return ld
